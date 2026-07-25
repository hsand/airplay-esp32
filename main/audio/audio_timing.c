#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "audio_timing.h"

#include "audio_output.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ntp_clock.h"
#include "ptp_clock.h"

#define DEFAULT_BUFFER_LATENCY_US 2000 // 2ms startup jitter buffer
// Additional pipeline latency to account for task scheduling, I2S write
// blocking, and resampler processing.  Without this, frames pass the
// timing check "on time" but actually exit the speaker several ms later.
#define PIPELINE_LATENCY_US 5000 // ~5ms scheduling + write delay
#define MIN_STARTUP_FRAMES  4

// Drift servo tuning.  DRIFT_FILTER_DIV is the IIR divisor applied to the
// per-frame timing error; larger = smoother but slower to react.  At ~125
// frames/s a divisor of 16 gives a ~0.13 s time constant.
// DRIFT_DEADBAND_US is how far the smoothed error may wander before a
// one-sample trim is applied.  It must sit well inside the drop threshold
// (10 ms by default) so the servo corrects long before a frame would be
// discarded, but comfortably above one sample period (22.7 µs at 44.1 kHz)
// so the servo is not chasing quantisation noise.
#define DRIFT_FILTER_DIV  16
#define DRIFT_DEADBAND_US 1500

// Every audio output backend (I2S, S/PDIF, USB) allocates its read buffer as
// (FRAME_SAMPLES + 1) * 2 int16 samples — i.e. interleaved stereo — and the
// output stage is stereo regardless of what an incoming frame header claims.
// Writes into the caller's buffer must therefore be bounded by this constant,
// never by hdr->channels.
#define AUDIO_OUT_CHANNELS 2

// Early/late threshold: how far a frame may be early (held as pending) or late
// (dropped) before the timing engine acts.  Buffered AirPlay 2 streams have a
// deep jitter buffer so a tight threshold keeps sync without drop-outs.
// Unbuffered realtime streams (ALAC/UDP) have almost no buffer to absorb
// scheduling hiccups — e.g. when artwork/metadata arrives on the RTSP
// connection — so they need a much looser threshold to avoid audible
// drop-outs.  Both are configurable via Kconfig.
#ifdef CONFIG_AIRPLAY_TIMING_THRESHOLD_MS
#define TIMING_THRESHOLD_US (CONFIG_AIRPLAY_TIMING_THRESHOLD_MS * 1000)
#else
#define TIMING_THRESHOLD_US 25000 // 25ms early/late threshold (buffered)
#endif

#ifdef CONFIG_AIRPLAY_RT_TIMING_THRESHOLD_MS
#define RT_TIMING_THRESHOLD_US (CONFIG_AIRPLAY_RT_TIMING_THRESHOLD_MS * 1000)
#else
#define RT_TIMING_THRESHOLD_US 50000 // 50ms early/late threshold (realtime)
#endif
// MAX_CONSECUTIVE_EARLY: safety valve — counts how many consecutive calls to
// audio_timing_read returned silence because the pending frame was still too
// early.  Each call corresponds to one DMA period (~46 ms on I2S at 44100 Hz).
// 50 calls × 46 ms ≈ 2.3 s: long enough that a legitimate pre-buffer of any
// realistic depth will never hit it, short enough to detect a genuinely stuck
// or invalid anchor in a few seconds.
#define MAX_CONSECUTIVE_EARLY 50

static const char *TAG = "audio_time";
// consecutive_early_frames is now a field in audio_timing_t so it resets
// automatically whenever a new anchor is set.

static uint32_t frame_samples_from_format(const audio_format_t *format) {
  if (format->frame_size > 0) {
    return (uint32_t)format->frame_size;
  }
  if (format->max_samples_per_frame > 0) {
    return format->max_samples_per_frame;
  }
  return AAC_FRAMES_PER_PACKET;
}

static void update_timing_targets(audio_timing_t *timing,
                                  const audio_format_t *format) {
  timing->nominal_frame_samples = frame_samples_from_format(format);

  if (format->sample_rate <= 0 || timing->nominal_frame_samples == 0) {
    timing->target_buffer_frames = MIN_STARTUP_FRAMES;
    return;
  }

  uint64_t latency_samples =
      ((uint64_t)timing->output_latency_us * (uint64_t)format->sample_rate) /
      1000000ULL;
  uint32_t target_frames =
      (uint32_t)((latency_samples + timing->nominal_frame_samples - 1) /
                 timing->nominal_frame_samples);
  if (target_frames < MIN_STARTUP_FRAMES) {
    target_frames = MIN_STARTUP_FRAMES;
  }
  timing->target_buffer_frames = target_frames;
}

typedef enum {
  SYNC_MODE_NONE, // No clock sync, use local anchor time
  SYNC_MODE_PTP,  // AirPlay 2 PTP sync
  SYNC_MODE_NTP,  // AirPlay 1 NTP sync
} sync_mode_t;

// Compute how early (positive) or late (negative) a frame is in microseconds
static bool compute_early_us(const audio_timing_t *timing,
                             const audio_format_t *format,
                             uint32_t rtp_timestamp, sync_mode_t sync_mode,
                             int64_t *early_us) {
  if (!timing->anchor_valid || format->sample_rate <= 0) {
    return false;
  }

  int32_t rtp_delta = (int32_t)(rtp_timestamp - timing->anchor_rtp_time);
  int64_t frame_offset_ns =
      ((int64_t)rtp_delta * 1000000000LL) / format->sample_rate;

  int64_t target_ns;
  switch (sync_mode) {
  case SYNC_MODE_PTP:
    // AirPlay 2: use network time with PTP offset for multi-room sync
    target_ns = (int64_t)timing->anchor_network_time_ns -
                ptp_clock_get_offset_ns() + frame_offset_ns;
    break;
  case SYNC_MODE_NTP:
    // AirPlay 1: use network time with NTP offset for multi-room sync
    // offset = remote_time - local_time, so local = remote - offset
    target_ns = (int64_t)timing->anchor_network_time_ns -
                ntp_clock_get_offset_ns() + frame_offset_ns;
    break;
  default:
    // Fallback: use local anchor time (no multi-room sync)
    target_ns = timing->anchor_local_time_ns + frame_offset_ns;
    break;
  }

  // Subtract hardware latency to account for I2S DMA delay
  // and pipeline latency for task scheduling and write blocking.
  // The hardware latency is computed from the DMA descriptor/frame
  // configuration rather than being hard-coded.
  target_ns -=
      (int64_t)(audio_output_get_hardware_latency_us() + PIPELINE_LATENCY_US) *
      1000LL;

  int64_t now_ns = (int64_t)esp_timer_get_time() * 1000LL;
  *early_us = (target_ns - now_ns) / 1000LL;

  return true;
}

void audio_timing_init(audio_timing_t *timing, size_t pending_capacity) {
  if (!timing) {
    return;
  }

  memset(timing, 0, sizeof(*timing));
  timing->output_latency_us = DEFAULT_BUFFER_LATENCY_US;
  timing->playing = true;

  if (pending_capacity > 0) {
    timing->pending_frame = (uint8_t *)malloc(pending_capacity);
    if (timing->pending_frame) {
      timing->pending_frame_capacity = pending_capacity;
    }
  }
}

void audio_timing_reset(audio_timing_t *timing) {
  if (!timing) {
    return;
  }

  timing->playout_started = false;
  timing->anchor_valid = false;
  timing->pending_valid = false;
  timing->pending_frame_len = 0;
  timing->ready_time_us = 0;
  timing->consecutive_early_frames = 0;
  timing->quick_start = false;
  timing->deferred_flush_pending = false;
  timing->flush_until_ts = 0;
  timing->drift_err_filtered_us = 0;
  timing->drift_corrections = 0;
}

void audio_timing_set_format(audio_timing_t *timing,
                             const audio_format_t *format) {
  if (!timing || !format) {
    return;
  }

  update_timing_targets(timing, format);
}

void audio_timing_set_output_latency(audio_timing_t *timing,
                                     const audio_format_t *format,
                                     uint32_t latency_us) {
  if (!timing || !format) {
    return;
  }

  timing->output_latency_us = latency_us;
  update_timing_targets(timing, format);
}

uint32_t audio_timing_get_output_latency(const audio_timing_t *timing) {
  if (!timing) {
    return 0;
  }

  return timing->output_latency_us;
}

uint32_t audio_timing_get_hardware_latency(void) {
  return audio_output_get_hardware_latency_us();
}

uint32_t audio_timing_get_advertised_latency(const audio_timing_t *timing) {
  // Total end-to-end latency between the phone scheduling a frame and the
  // DAC emitting it.  Reported to the phone in outputLatencyMicros so it
  // schedules sends to land in our sorted buffer at the right time.
  //
  //   output_latency_us         — controller target (jitter-buffer depth)
  // + audio_output_get_hardware_latency_us() — I2S DMA delay (dynamic)
  // + PIPELINE_LATENCY_US — scheduling + write delay constant
  uint32_t base =
      timing ? timing->output_latency_us : DEFAULT_BUFFER_LATENCY_US;
  return base + audio_output_get_hardware_latency_us() + PIPELINE_LATENCY_US;
}

void audio_timing_set_anchor(audio_timing_t *timing,
                             const audio_format_t *format, uint64_t clock_id,
                             uint64_t network_time_ns, uint32_t rtp_time) {
  if (!timing || !format) {
    return;
  }

  (void)clock_id;

  int64_t now_ns = (int64_t)esp_timer_get_time() * 1000LL;

  timing->anchor_rtp_time = rtp_time;
  timing->anchor_network_time_ns = network_time_ns;
  timing->anchor_local_time_ns = now_ns;
  timing->ptp_locked = ptp_clock_is_locked();
  timing->anchor_valid = true;
  // Reset frame counters so pre-buffered audio after a pause/resume or
  // track skip does not accumulate into the new anchor's counts.
  timing->consecutive_early_frames = 0;
  // The servo's error estimate is relative to the old anchor; a new anchor
  // redefines the reference, so carrying the old error over would make the
  // servo trim against a offset that no longer exists.
  timing->drift_err_filtered_us = 0;

  // Compute lead time: how far in the future this anchor's network timestamp
  // is relative to now.  Negative means the anchor is already in the past
  // (normal: the phone pre-buffers and the anchor is 200–800 ms old by the
  // time we receive it).
  int64_t lead_ms = ((int64_t)network_time_ns -
                     (int64_t)(ptp_clock_get_offset_ns() + now_ns)) /
                    1000000LL;
  ESP_LOGI(
      TAG,
      "Anchor set: rtp=%" PRIu32 " lead=%lld ms ptp_locked=%d quick_start=%d",
      rtp_time, (long long)lead_ms, timing->ptp_locked, timing->quick_start);
}

void audio_timing_set_playing(audio_timing_t *timing, bool playing) {
  if (!timing) {
    return;
  }

  ESP_LOGI(TAG, "set_playing: %s -> %s", timing->playing ? "playing" : "paused",
           playing ? "playing" : "paused");

  timing->playing = playing;
  if (!playing) {
    // Discard any partially-pending frame so resume starts cleanly from
    // the oldest frame in the sorted buffer.
    timing->pending_valid = false;
    timing->pending_frame_len = 0;
  }
}

size_t audio_timing_read(audio_timing_t *timing, audio_buffer_t *buffer,
                         const audio_stream_t *stream, audio_stats_t *stats,
                         int16_t *out, size_t samples) {
  if (!timing || !buffer || !stream || !out || samples == 0) {
    return 0;
  }

  if (!timing->playing) {
    return 0;
  }

  const audio_format_t *format = &stream->format;
  int buffered_frames = audio_buffer_get_frame_count(buffer);

  // Unbuffered realtime streams (ALAC/UDP) get a looser early/late threshold
  // than buffered AirPlay 2 streams, because they have little jitter buffer to
  // absorb scheduling hiccups and would otherwise drop frames (audible
  // drop-outs) whenever the pipeline stalls — e.g. while artwork/metadata is
  // received on the RTSP connection.
  const int64_t timing_threshold_us = audio_stream_uses_buffer(stream->type)
                                          ? TIMING_THRESHOLD_US
                                          : RT_TIMING_THRESHOLD_US;

  // Wait for enough buffer before starting.
  // In quick_start mode (after a seek/skip), start as soon as 1 frame is
  // available to minimise the gap between tracks.  Anchor-based timing
  // still applies — if the frame is early, silence is output until its
  // scheduled play time, just like shairport-sync.
  // Normal startup waits for target_buffer_frames to build jitter margin.
  if (!timing->playout_started && !timing->pending_valid) {
    int required = timing->quick_start ? 1 : (int)timing->target_buffer_frames;
    if (buffered_frames < required) {
      return 0;
    }
    // Wait for anchor before playing.
    // Normal startup: allow a 1-second fallback so a stream with no anchor
    // (e.g. AirPlay 1 without NTP) can still start.
    if (!timing->anchor_valid) {
      int64_t now_us = esp_timer_get_time();
      if (timing->ready_time_us == 0) {
        timing->ready_time_us = now_us;
      }
      if (now_us - timing->ready_time_us < 1000000) {
        return 0; // Still waiting for anchor
      }
      // Waited 1 second, no anchor - proceed without sync
    }
  }

  // Determine sync mode: PTP (AirPlay 2), NTP (AirPlay 1), or local fallback
  sync_mode_t sync_mode = SYNC_MODE_NONE;
  if (ptp_clock_is_locked()) {
    sync_mode = SYNC_MODE_PTP;
  } else if (ntp_clock_is_locked()) {
    sync_mode = SYNC_MODE_NTP;
  }

  // Drain up to MAX_DRAIN_ATTEMPTS late/invalid frames within a SINGLE
  // DMA callback.  The previous limit of 8 was the root cause of run-away
  // lateness: each call that returned silence (instead of playing a frame)
  // forfeited ~23 ms of RTP advancement while wall time kept moving, so
  // every late frame we dropped MADE us more late.  Draining many frames
  // in one pass advances RTP at zero wall-time cost and lets the buffer
  // skip past stale data without the DMA ever idling.
  enum { MAX_DRAIN_ATTEMPTS = 256 };
  for (int attempt = 0; attempt < MAX_DRAIN_ATTEMPTS; attempt++) {
    size_t item_size = 0;
    void *item = NULL;
    bool from_pending = false;

    // Get frame from pending or buffer
    if (timing->pending_valid) {
      item_size = timing->pending_frame_len;
      if (item_size < sizeof(audio_frame_header_t)) {
        timing->pending_valid = false;
        timing->pending_frame_len = 0;
        continue;
      }
      item = timing->pending_frame;
      from_pending = true;
    } else {
      if (!audio_buffer_take(buffer, &item, &item_size, 0)) {
        if (stats) {
          stats->buffer_underruns++;
        }
        return 0;
      }
      buffered_frames = audio_buffer_get_frame_count(buffer);

      if (item_size < sizeof(audio_frame_header_t)) {
        audio_buffer_return(buffer, item);
        continue;
      }
    }

    audio_frame_header_t *hdr = (audio_frame_header_t *)item;
    size_t frame_samples = hdr->samples_per_channel;
    size_t channels = hdr->channels ? hdr->channels : format->channels;
    int16_t *pcm = (int16_t *)(hdr + 1);

    // Validate frame
    if (frame_samples == 0 || channels == 0) {
      if (from_pending) {
        timing->pending_valid = false;
        timing->pending_frame_len = 0;
      } else {
        audio_buffer_return(buffer, item);
      }
      continue;
    }

    size_t expected_bytes =
        sizeof(*hdr) + frame_samples * channels * sizeof(int16_t);
    if (item_size < expected_bytes) {
      if (from_pending) {
        timing->pending_valid = false;
        timing->pending_frame_len = 0;
      } else {
        audio_buffer_return(buffer, item);
      }
      continue;
    }

    if (frame_samples > samples) {
      frame_samples = samples;
    }

    // Deferred flush check (AirPlay 2 FLUSHBUFFERED with flushFromSeq):
    // keep playing until the frame whose RTP timestamp reaches flush_until_ts,
    // then bulk-flush the remainder of the buffer and start fresh.
    // Signed 32-bit subtraction handles RTP wraparound correctly.
    if (timing->deferred_flush_pending) {
      if ((int32_t)(hdr->rtp_timestamp - timing->flush_until_ts) >= 0) {
        ESP_LOGI(TAG,
                 "Deferred flush triggered at ts=%" PRIu32 " (until_ts=%" PRIu32
                 ")",
                 hdr->rtp_timestamp, timing->flush_until_ts);
        if (from_pending) {
          timing->pending_valid = false;
          timing->pending_frame_len = 0;
        } else {
          audio_buffer_return(buffer, item);
        }
        audio_buffer_flush(buffer);
        timing->deferred_flush_pending = false;
        timing->playout_started = false;
        timing->ready_time_us = 0;
        timing->consecutive_early_frames = 0;
        // quick_start so the first frame of the next track starts playing
        // as soon as 1 frame arrives, with normal anchor timing applied.
        timing->quick_start = true;
        return 0;
      }
    }

    // Handle early/late frames based on anchor timing.
    //
    // After a seek/flush, anchor-based timing is applied immediately from the
    // first frame — no bypass.  With a stable PTP clock the anchor is
    // accurate, so early frames are held as pending (silence output) until
    // their scheduled play time, and late frames are dropped.  This mirrors
    // shairport-sync's approach and guarantees the first audible sample is
    // correctly synchronised.
    if (timing->anchor_valid && format->sample_rate > 0) {
      int64_t early_us = 0;
      if (compute_early_us(timing, format, hdr->rtp_timestamp, sync_mode,
                           &early_us)) {
        if (early_us > timing_threshold_us) {
          // Only advance the stuck-anchor counter for NEW frames taken from
          // the buffer — not for pending re-checks of the same early frame.
          // A pending frame is re-examined every DMA callback (~8 ms) while
          // we wait for wall-clock to reach its scheduled play time.  Counting
          // those re-checks would fire the stuck-anchor detector in
          // (MAX_CONSECUTIVE_EARLY × 8 ms) = 6 s even for a legitimately
          // early frame that just needs to wait its pre-buffer depth (~1.5 s).
          if (!from_pending) {
            timing->consecutive_early_frames++;
            // Log the first early frame after each anchor set (shows lead
            // time before audio starts) and every 50 new frames after that
            // (confirms the counter only counts real buffer reads, not
            // pending re-checks).
            if (timing->consecutive_early_frames == 1) {
              ESP_LOGI(TAG,
                       "First early frame: rtp=%" PRIu32 " early=%.1f ms"
                       " quick_start=%d buffered=%d",
                       hdr->rtp_timestamp, (float)early_us / 1000.0f,
                       timing->quick_start, buffered_frames);
            } else if (timing->consecutive_early_frames % 50 == 0) {
              ESP_LOGD(TAG, "Early counter: %d/%d early=%.1f ms rtp=%" PRIu32,
                       timing->consecutive_early_frames, MAX_CONSECUTIVE_EARLY,
                       (float)early_us / 1000.0f, hdr->rtp_timestamp);
            }
          }

          // If we have had an implausibly long run of early frames the anchor
          // is probably stuck or wrong — give up on it so playback can
          // continue.  This threshold is high enough (~17 s at 23 ms/frame)
          // that it never fires during normal pre-buffered-audio scenarios.
          if (timing->consecutive_early_frames > MAX_CONSECUTIVE_EARLY) {
            ESP_LOGW(TAG,
                     "Invalidating stuck anchor: consecutive=%d, early=%lld ms",
                     timing->consecutive_early_frames, early_us / 1000LL);
            timing->anchor_valid = false;
            timing->consecutive_early_frames = 0;
            // Fall through to play the frame normally
          } else {
            // Frame is early — store it as pending and output silence.
            // The pending frame is re-checked on every subsequent call;
            // once wall-clock catches up it will be played on time.
            // This is the normal path for pre-buffered audio after a pause.
            static int early_count = 0;
            early_count++;
            if (early_count % 100 == 1) {
              ESP_LOGD(TAG,
                       "Frame too early #%d: %lld ms, buffered=%d, pending=%d",
                       early_count, early_us / 1000LL, buffered_frames,
                       timing->pending_valid ? 1 : 0);
            }
            if (!from_pending && timing->pending_frame &&
                item_size <= timing->pending_frame_capacity) {
              memcpy(timing->pending_frame, item, item_size);
              timing->pending_frame_len = item_size;
              timing->pending_valid = true;
              audio_buffer_return(buffer, item);
            }
            // Emit one frame's worth of silence, not the caller's full
            // capacity.  Callers allocate exactly `samples` stereo frames
            // (see audio_output.c), so the write must be bounded by
            // AUDIO_OUT_CHANNELS rather than the frame header's channel
            // count — a malformed header claiming >2 channels would
            // otherwise overrun the caller's buffer.  Returning
            // frame_samples (not `samples`) also keeps the silence path's
            // output length consistent with the normal playback path, so
            // holding a frame pending does not advance the output clock
            // faster than playing one.
            memset(out, 0,
                   frame_samples * AUDIO_OUT_CHANNELS * sizeof(int16_t));
            return frame_samples;
          }
        } else if (early_us < -timing_threshold_us) {
          // Reset consecutive early counter on late/normal frames
          timing->consecutive_early_frames = 0;

          // Late frame — drop it and continue draining within the SAME call.
          // The 256-attempt drain loop chews through stale frames at zero
          // wall-time cost, skipping past arbitrarily many stale frames in
          // one pass without the DMA ever idling.
          ESP_LOGW(TAG, "Dropping late frame: %lld ms", -early_us / 1000LL);
          if (stats) {
            stats->late_frames++;
          }
          if (from_pending) {
            timing->pending_valid = false;
            timing->pending_frame_len = 0;
          } else {
            audio_buffer_return(buffer, item);
          }
          continue;
        }
      }
    }

    // Frame is on time (or anchor-invalid) — reset counter.
    timing->consecutive_early_frames = 0;

    // ---- Drift servo -------------------------------------------------
    // The I2S clock is derived from the local crystal and free-runs against
    // the sender's PTP timebase.  A typical crystal is off by 10-40 ppm,
    // which accumulates ~40-140 ms of error per hour.  Without correction
    // the error walks until it crosses timing_threshold_us and an entire
    // frame is dropped (or a whole silence period inserted) — an audible
    // click every few minutes on an otherwise healthy network.
    //
    // Instead, track a heavily-smoothed estimate of the on-time error and
    // trim ONE sample from the frame when it drifts outside a band well
    // inside the drop threshold.  One sample at 44.1 kHz is 22.7 µs, far
    // below audibility, and correcting continuously means the error never
    // reaches the point where a frame has to be discarded.
    //
    // Only runs while an anchor is valid and a network clock is locked —
    // with no reference there is nothing meaningful to servo against.
    int sample_adjust = 0;
    if (timing->anchor_valid && sync_mode != SYNC_MODE_NONE &&
        timing->playout_started) {
      int64_t on_time_err_us = 0;
      if (compute_early_us(timing, format, hdr->rtp_timestamp, sync_mode,
                           &on_time_err_us)) {
        // Playout report (diagnostic).  Logged about once a second at
        // ~125 frames/s.  Three quantities, together enough to tell where a
        // constant sync offset comes from:
        //   err      — how far this frame is from its anchored play time.
        //              Near 0 means we are playing exactly where the anchor
        //              says, so a constant offset must come from the anchor
        //              or from latency negotiation, not from this gate.
        //   buffered — frames still queued behind this one.
        //   depth_ms — how far the frame we are playing sits behind the
        //              NEWEST frame in the buffer.  This is the real
        //              end-to-end delay we are adding on top of the anchor;
        //              if it is ~1 s while err is ~0, we are faithfully
        //              playing a frame that is a second stale.
        if (timing->playout_reports++ % 125 == 0) {
          uint32_t newest_rtp = 0;
          int64_t depth_ms = -1;
          if (audio_buffer_peek_newest_rtp(buffer, &newest_rtp)) {
            depth_ms =
                ((int64_t)(int32_t)(newest_rtp - hdr->rtp_timestamp) * 1000LL) /
                format->sample_rate;
          }
          ESP_LOGI(
              TAG,
              "Playout: err=%lld ms buffered=%d depth=%lld ms rtp=%" PRIu32,
              (long long)(on_time_err_us / 1000LL), buffered_frames,
              (long long)depth_ms, hdr->rtp_timestamp);
        }
        // First-order IIR: err_filtered += (err - err_filtered) / 16.
        // At ~125 frames/s this has a time constant of ~0.13 s, long enough
        // to reject per-frame network jitter but fast enough to track a
        // real clock offset.
        timing->drift_err_filtered_us +=
            (on_time_err_us - timing->drift_err_filtered_us) / DRIFT_FILTER_DIV;

        // Positive filtered error => we are playing EARLY (frame's scheduled
        // time is still ahead of now) => stretch by duplicating one sample.
        // Negative => playing LATE => shrink by dropping one sample.
        if (timing->drift_err_filtered_us > DRIFT_DEADBAND_US) {
          sample_adjust = 1;
        } else if (timing->drift_err_filtered_us < -DRIFT_DEADBAND_US) {
          sample_adjust = -1;
        }

        if (sample_adjust != 0) {
          // Credit the correction back to the filter so the servo does not
          // keep trimming for an error it has already begun to cancel.
          int64_t corrected_us =
              (int64_t)sample_adjust * 1000000LL / format->sample_rate;
          timing->drift_err_filtered_us -= corrected_us;
          timing->drift_corrections++;
          if (timing->drift_corrections % 500 == 1) {
            ESP_LOGI(TAG,
                     "Drift servo: %+d sample, filtered_err=%lld us, "
                     "corrections=%" PRIu32,
                     sample_adjust, (long long)timing->drift_err_filtered_us,
                     timing->drift_corrections);
          }
        }
      }
    }

    // Copy PCM data to output, applying the servo's one-sample trim.
    // Shrink: copy one sample fewer.  Stretch: copy the frame then repeat
    // its final sample once.  Both stay within the caller's capacity, which
    // is at least frame_samples + 1 (see AUDIO_OUT_CHANNELS note above).
    size_t out_samples = frame_samples;
    if (sample_adjust < 0 && out_samples > 1) {
      out_samples--;
    } else if (sample_adjust > 0 && out_samples + 1 <= samples) {
      out_samples++;
    } else {
      sample_adjust = 0;
    }

    // The output buffer is interleaved stereo of exactly `samples` frames,
    // so never write more than AUDIO_OUT_CHANNELS per sample regardless of
    // what the frame header claims.  `channels` is still used to step
    // through the SOURCE frame, which was length-validated above against
    // expected_bytes.
    size_t out_ch =
        channels < AUDIO_OUT_CHANNELS ? channels : AUDIO_OUT_CHANNELS;
    size_t copy_samples =
        out_samples < frame_samples ? out_samples : frame_samples;
    if (out_ch == channels) {
      memcpy(out, pcm, copy_samples * out_ch * sizeof(int16_t));
    } else {
      // Source has more channels than we emit — take the leading out_ch.
      for (size_t i = 0; i < copy_samples; i++) {
        for (size_t ch = 0; ch < out_ch; ch++) {
          out[i * out_ch + ch] = pcm[i * channels + ch];
        }
      }
    }
    if (out_samples > frame_samples) {
      // Duplicate the last sample of each channel to stretch by one.
      const int16_t *last = pcm + (frame_samples - 1) * channels;
      int16_t *dst = out + frame_samples * out_ch;
      for (size_t ch = 0; ch < out_ch; ch++) {
        dst[ch] = last[ch];
      }
    }

    // Cleanup
    if (from_pending) {
      timing->pending_valid = false;
      timing->pending_frame_len = 0;
    } else {
      audio_buffer_return(buffer, item);
    }

    if (!timing->playout_started) {
      timing->playout_started = true;
      bool was_quick = timing->quick_start;
      timing->quick_start = false;
      ESP_LOGI(TAG, "Playout started%s: rtp=%" PRIu32,
               was_quick ? " (quick_start)" : "", hdr->rtp_timestamp);
    }

    return out_samples;
  }

  return 0;
}

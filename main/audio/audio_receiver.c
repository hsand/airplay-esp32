// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "audio_receiver.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "audio_buffer.h"
#include "audio_decoder.h"
#include "audio_output.h"
#include "audio_playout_clock.h"
#include "audio_receiver_internal.h"
#include "audio_stream.h"
#include "audio_timing.h"
#include "ntp_clock.h"
#include "ptp_clock.h"

#define DEFAULT_SAMPLE_RATE     44100
#define DEFAULT_CHANNELS        2
#define DEFAULT_BITS_PER_SAMPLE 16
#define DEFAULT_FRAME_SIZE      352
#define DECRYPT_BUFFER_SIZE     8192

static const char *TAG = "audio_recv";

static audio_receiver_state_t receiver = {0};

// Set when an AirPlay session holds the audio path. Reported on the edges only:
// SETUP can restate the format mid-session, and the arbitration downstream ends
// a Sendspin stream, which is not something to repeat per track.
static audio_receiver_activity_cb_t s_activity_cb = NULL;
static bool s_session_active = false;
// The claim and the release arrive from different RTSP tasks -- a TEARDOWN on
// one connection races the SETUP that replaced it -- so the edge test has to be
// atomic even though the flag is a bool.
static portMUX_TYPE s_activity_mux = portMUX_INITIALIZER_UNLOCKED;

void audio_receiver_set_activity_callback(audio_receiver_activity_cb_t cb) {
  s_activity_cb = cb;
}

static void set_session_active(bool active) {
  bool changed = false;
  taskENTER_CRITICAL(&s_activity_mux);
  if (s_session_active != active) {
    s_session_active = active;
    changed = true;
  }
  taskEXIT_CRITICAL(&s_activity_mux);
  // Outside the critical section: the listener hands the output over, which
  // joins a task and talks to the amplifier over I2C.
  if (changed && s_activity_cb) {
    s_activity_cb(active);
  }
}

void audio_receiver_end_session(void) {
  set_session_active(false);
}

static void audio_receiver_reset_stats(void) {
  memset(&receiver.stats, 0, sizeof(receiver.stats));
}

static void audio_receiver_reset_resend_state(void) {
  // The realtime receive task owns the tracker and resets it before its next
  // packet, so a flush cannot tear it from under a packet being counted.
  __atomic_add_fetch(&receiver.resend_generation, 1U, __ATOMIC_RELEASE);
  receiver.last_resend_error_time_us = 0;
}

static void audio_receiver_reset_blocks(void) {
  receiver.blocks_read = 0;
  receiver.blocks_read_in_sequence = 0;
}

static void audio_receiver_copy_stream_state(audio_stream_t *dst,
                                             const audio_stream_t *src) {
  if (!dst || !src) {
    return;
  }

  dst->format = src->format;
  dst->encrypt = src->encrypt;
}

// True when the PCM timeline owns playback.  Both stream types use it; they
// never run at the same time, and the engine is re-pointed at the right codec
// frame length when each one starts.
static inline bool engine_v2_active(void) {
  return receiver.engine_v2_ready && receiver.stream;
}

// AAC frames are always 1024 samples.  ALAC packet length comes from the SDP
// (`a=fmtp` frame length), 352 on every sender seen so far.
static uint32_t engine_v2_frame_samples(audio_stream_type_t type,
                                        const audio_stream_t *stream) {
  if (type == AUDIO_STREAM_BUFFERED) {
    return AUDIO_TIMELINE_FRAME_SAMPLES;
  }
  return stream->format.frame_size > 0 ? (uint32_t)stream->format.frame_size
                                       : AUDIO_TIMELINE_RT_FRAME_SAMPLES;
}

static void audio_receiver_reset_engine_v2(void);

// Create the engine on first use.  It costs ~790 KB of PSRAM, and it is never
// torn down afterwards: the playback task calls audio_engine_v2_render() on
// every I2S refill and freeing the timeline underneath it would race.  The
// slot pool is therefore sized once for the longest frame either codec uses,
// and switching streams only re-points the addressing quantum -- 192 slots
// hold ~4.5 s of AAC or ~1.5 s of ALAC.
static esp_err_t audio_receiver_ensure_engine_v2(audio_stream_type_t type) {
  audio_stream_t *stream = type == AUDIO_STREAM_BUFFERED
                               ? receiver.buffered_stream
                               : receiver.realtime_stream;
  if (!stream) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!receiver.decoder_mutex) {
    receiver.decoder_mutex = xSemaphoreCreateMutex();
    if (!receiver.decoder_mutex) {
      ESP_LOGE(TAG, "Failed to create decoder mutex");
      return ESP_ERR_NO_MEM;
    }
  }

  if (!receiver.engine_v2_ready) {
    esp_err_t err = audio_engine_v2_init(&receiver.engine_v2, &stream->format,
                                         AUDIO_TIMELINE_FRAME_SAMPLES,
                                         AUDIO_V2_TIMELINE_BLOCKS);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Engine init failed: %s", esp_err_to_name(err));
      return err;
    }
    receiver.engine_v2_ready = true;
  }

  const uint32_t frame_samples = engine_v2_frame_samples(type, stream);

  if (receiver.engine_v2.timeline.frame_samples != frame_samples) {
    // Bump the epoch and drop queued work *before* the pool is re-cut.  The
    // re-cut moves every slot address, and a decode worker or the playback
    // task can still be copying through a pointer it took under the old
    // stride.  A stale epoch makes new pushes fail immediately, which lets
    // the re-cut wait the stragglers out instead of racing them.  The
    // outgoing stream's clock map and cursor describe nothing the timeline
    // still holds either.
    audio_receiver_reset_engine_v2();
  }

  if (!audio_engine_v2_set_frame_samples(&receiver.engine_v2, frame_samples)) {
    return ESP_ERR_INVALID_ARG;
  }

  // Realtime decodes inline on the rx task; only buffered offloads.
  if (type == AUDIO_STREAM_BUFFERED && !receiver.decode_worker) {
    // The worker is torn down by audio_receiver_stop() together with the
    // decoder it uses, so it is recreated on every buffered SETUP.
    esp_err_t err =
        audio_decode_worker_create(&receiver, &receiver.decode_worker);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Decode worker create failed: %s", esp_err_to_name(err));
      return err;
    }
  }

  return ESP_OK;
}

// Discard everything buffered for the buffered path and wait for a fresh
// anchor.  Advancing the epoch invalidates in-flight decode jobs and the whole
// timeline in one step, so no stale PCM can reach the new segment.
static void audio_receiver_reset_engine_v2(void) {
  if (!receiver.engine_v2_ready) {
    return;
  }
  audio_decode_worker_discard_pending(receiver.decode_worker);
  (void)audio_engine_v2_begin_epoch(&receiver.engine_v2, esp_timer_get_time());
  // aac_diag_rtp_valid stays set: the first frame of the new epoch then
  // differs from the last one decoded in epoch, which primes the decoder
  // (audio_stream_decode_encoded_packet()). The AAC decoder is not reset by
  // a flush, so its overlap buffers still hold the old position.
  // aac_prime_left is left alone too: the decode worker owns it under the
  // decoder mutex, and may already have primed a frame of the new epoch.
  receiver.engine_v2_anchor_pending = false;
  taskENTER_CRITICAL(&receiver.anchor_lock);
  receiver.rate_anchor_pending = false;
  taskEXIT_CRITICAL(&receiver.anchor_lock);
  // A new epoch holds nothing a pending flush could still apply to, and the
  // sequence numbers it will see belong to whatever stream comes next.
  taskENTER_CRITICAL(&receiver.flush_lock);
  audio_flush_queue_reset(&receiver.flush_queue);
  taskEXIT_CRITICAL(&receiver.flush_lock);
}

// Local -> sender clock offset, and whether that clock is actually locked.
//
// The sender's anchor timestamps are PTP for AirPlay 2 (control packet 0x57)
// and NTP for AirPlay 1 (0x54), so the engine cannot assume PTP the way it did
// while only the buffered path used it.  Which one applies is decided by the
// anchor itself, not by whichever clock happens to be locked: a PTP lock left
// over from a previous session (or kept alive by another sender on the
// network) would otherwise map an AirPlay 1 anchor into the PTP timeline, and
// vice versa, putting every block decades from its real playout time.  Like
// compute_early_us() in audio_timing.c this must be re-read on every render:
// before a lock the offset is 0, and freezing that value places the anchor
// days from local time and wraps the int32 RTP delta.
static int64_t audio_receiver_network_offset_ns(bool *locked) {
  const bool have_lock = receiver.engine_v2_anchor_uses_ptp
                             ? ptp_clock_is_locked()
                             : ntp_clock_is_locked();
  if (locked) {
    *locked = have_lock;
  }
  if (!have_lock) {
    return 0;
  }
  return receiver.engine_v2_anchor_uses_ptp ? ptp_clock_get_offset_ns()
                                            : ntp_clock_get_offset_ns();
}

// Publish the pending anchor once a network clock is usable.  Before the first
// lock the offset is 0, which would place the anchor days away from local time
// and wrap the int32 RTP delta into a meaningless position.
// Retried from the playback task; re-arming with identical values is a no-op.
static void audio_receiver_arm_engine_v2_anchor(void) {
  bool locked = false;
  (void)audio_receiver_network_offset_ns(&locked);
  if (!receiver.engine_v2_anchor_pending || !locked) {
    return;
  }
  if (audio_engine_v2_set_anchor(&receiver.engine_v2,
                                 receiver.engine_v2_anchor_rtp,
                                 receiver.engine_v2_anchor_network_ns,
                                 receiver.engine_v2_playout_offset_ns)) {
    receiver.engine_v2_anchor_pending = false;
  }
}

// True once the clock can place a buffered anchor: locked, and on the
// timeline the anchor names.  This is the check Apple makes before it accepts
// a rate-1 SETRATEANCHORTIME (net-to-host conversion, then IsLocked).
static bool rate_anchor_placeable(const audio_engine_anchor_t *anchor) {
  if (!receiver.session_timing_ptp) {
    return ntp_clock_is_locked();
  }
  int64_t local_ns = 0;
  return ptp_clock_network_to_local(anchor->network_ns, anchor->timeline_id,
                                    &local_ns);
}

// Install the pending buffered anchor once the clock can place it.  Called
// on arrival and from every render after that.  Apple refuses a rate-1 anchor
// on an unlocked clock; the sender's reaction to that refusal is unknown, so
// this accepts it and waits, which is what the ESP32 always did.  Nothing is
// ever scheduled against an unlocked offset meanwhile: a stream already
// playing carries on on its old timeline, and a fresh one stays silent.
static void audio_receiver_arm_rate_anchor(void) {
  audio_engine_anchor_t anchor;
  taskENTER_CRITICAL(&receiver.anchor_lock);
  const bool pending = receiver.rate_anchor_pending;
  anchor = receiver.rate_anchor;
  taskEXIT_CRITICAL(&receiver.anchor_lock);
  if (!pending || !rate_anchor_placeable(&anchor)) {
    return;
  }
  (void)audio_engine_v2_set_rate_anchor(&receiver.engine_v2, &anchor);
  taskENTER_CRITICAL(&receiver.anchor_lock);
  // Only if no newer anchor replaced it meanwhile.
  if (receiver.rate_anchor_pending &&
      memcmp(&receiver.rate_anchor, &anchor, sizeof(anchor)) == 0) {
    receiver.rate_anchor_pending = false;
  }
  taskEXIT_CRITICAL(&receiver.anchor_lock);
}

esp_err_t audio_receiver_init(void) {
  if (receiver.buffer.decode_buffer) {
    return ESP_OK;
  }

  receiver.realtime_stream = audio_stream_create_realtime();
  if (receiver.realtime_stream) {
    receiver.realtime_stream->ctx = &receiver;
  }
  receiver.buffered_stream = audio_stream_create_buffered();
  if (receiver.buffered_stream) {
    receiver.buffered_stream->ctx = &receiver;
  }
  if (!receiver.realtime_stream || !receiver.buffered_stream) {
    ESP_LOGE(TAG, "Failed to allocate audio streams");
    audio_stream_destroy(receiver.realtime_stream);
    audio_stream_destroy(receiver.buffered_stream);
    receiver.realtime_stream = NULL;
    receiver.buffered_stream = NULL;
    return ESP_ERR_NO_MEM;
  }

  receiver.stream = receiver.realtime_stream;

  audio_format_t default_format = {0};
  strcpy(default_format.codec, "AppleLossless");
  default_format.sample_rate = DEFAULT_SAMPLE_RATE;
  default_format.channels = DEFAULT_CHANNELS;
  default_format.bits_per_sample = DEFAULT_BITS_PER_SAMPLE;
  default_format.frame_size = DEFAULT_FRAME_SIZE;

  receiver.realtime_stream->format = default_format;
  receiver.buffered_stream->format = default_format;

  esp_err_t err = audio_buffer_init(&receiver.buffer);
  if (err != ESP_OK) {
    audio_stream_destroy(receiver.realtime_stream);
    audio_stream_destroy(receiver.buffered_stream);
    receiver.realtime_stream = NULL;
    receiver.buffered_stream = NULL;
    return err;
  }

  receiver.decrypt_buffer_size = DECRYPT_BUFFER_SIZE;
#ifdef CONFIG_SPIRAM
  receiver.decrypt_buffer = heap_caps_malloc(
      receiver.decrypt_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
  if (!receiver.decrypt_buffer) {
    receiver.decrypt_buffer = malloc(receiver.decrypt_buffer_size);
  }
  if (!receiver.decrypt_buffer) {
    ESP_LOGE(TAG, "Failed to allocate decrypt buffer");
    audio_buffer_deinit(&receiver.buffer);
    audio_stream_destroy(receiver.realtime_stream);
    audio_stream_destroy(receiver.buffered_stream);
    receiver.realtime_stream = NULL;
    receiver.buffered_stream = NULL;
    return ESP_ERR_NO_MEM;
  }

  audio_timing_init(&receiver.timing);

  receiver.buffered_listen_socket = -1;
  receiver.buffered_client_socket = -1;
  receiver.flush_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  receiver.anchor_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  receiver.session_timing_ptp = true;
  audio_playout_clock_reset(&receiver.playout_clock, AUDIO_PLAYOUT_CLOCK_PTP);

  audio_receiver_reset_blocks();

  return ESP_OK;
}

void audio_receiver_set_format(const audio_format_t *format) {
  if (!format) {
    return;
  }
  if (!receiver.realtime_stream || !receiver.buffered_stream) {
    return;
  }

  receiver.realtime_stream->format = *format;
  receiver.buffered_stream->format = *format;

  // The decode worker task may be inside audio_decoder_decode() right now.
  if (receiver.decoder_mutex) {
    xSemaphoreTake(receiver.decoder_mutex, portMAX_DELAY);
  }
  audio_decoder_destroy(receiver.decoder);
  receiver.decoder = NULL;

  audio_decoder_config_t cfg = {.format = *format};
  receiver.decoder = audio_decoder_create(&cfg);
  if (receiver.decoder_mutex) {
    xSemaphoreGive(receiver.decoder_mutex);
  }
  if (!receiver.decoder) {
    ESP_LOGW(TAG, "Decoder not initialized for codec: %s", format->codec);
  }

  if (receiver.engine_v2_ready) {
    audio_engine_v2_set_format(&receiver.engine_v2, format);
  }
  audio_output_set_source_rate(format->sample_rate);
  set_session_active(true);
}

void audio_receiver_set_encryption(const audio_encrypt_t *encrypt) {
  if (!receiver.realtime_stream || !receiver.buffered_stream) {
    return;
  }
  if (encrypt) {
    receiver.realtime_stream->encrypt = *encrypt;
    receiver.buffered_stream->encrypt = *encrypt;
  } else {
    memset(&receiver.realtime_stream->encrypt, 0,
           sizeof(receiver.realtime_stream->encrypt));
    memset(&receiver.buffered_stream->encrypt, 0,
           sizeof(receiver.buffered_stream->encrypt));
  }
}

void audio_receiver_set_playout_latency_samples(uint32_t latency_samples) {
  audio_timing_set_playout_latency(&receiver.timing, latency_samples);
}

void audio_receiver_set_output_latency_us(uint32_t latency_us) {
  audio_timing_set_output_latency(&receiver.timing, latency_us);
}

uint32_t audio_receiver_get_output_latency_us(void) {
  return audio_timing_get_output_latency(&receiver.timing);
}

uint32_t audio_receiver_get_hardware_latency_us(void) {
  return audio_timing_get_hardware_latency();
}

uint32_t audio_receiver_get_advertised_latency_us(void) {
  return audio_timing_get_advertised_latency(&receiver.timing);
}

void audio_receiver_set_anchor_time(uint64_t clock_id, uint64_t network_time_ns,
                                    uint32_t rtp_time) {
  if (!receiver.stream) {
    return;
  }

  int sample_rate = receiver.stream->format.sample_rate;
  if (sample_rate <= 0) {
    sample_rate = 44100;
  }
  // Window size for the upper RTP gate: 10 s of samples.  Large enough that
  // a normal 2-4 s pre-buffer passes, but small enough to reject stale frames
  // left in the TCP socket buffer after a backward seek.
  const uint32_t gate_window = (uint32_t)(10 * sample_rate);

  // The per-RTP gates filter by timestamp range.  They are armed before
  // anything else here: on single-core ESP32-S2, ESP_LOGI can yield to the
  // scheduler, and the TCP task must not queue stale frames in that gap.
  //
  // seek_flush set arm_gate_on_next_anchor because the buffer was already
  // empty when the flush happened (forward-seek).
  //
  // There used to be a second path here that flushed everything when an
  // anchor landed more than 5 s from where the old one predicted.  Apple's
  // receiver never does that -- buffered audio survives a SETRATEANCHORTIME,
  // and only FLUSHBUFFERED/FLUSH discard -- and it misfired on long pauses.
  if (receiver.arm_gate_on_next_anchor) {
    receiver.arm_gate_on_next_anchor = false;
    // Once the backlog gate has seen the sequence break, the reader is on the
    // new position, and an iPhone may have streamed well over 10 s of it by
    // now. The upper gate only disarms on a frame that passes it, so a bound
    // below what has already been read would reject every frame from here
    // on. It is measured from the newest frame read instead.
    uint32_t above_from = rtp_time;
    const uint32_t last_read = receiver.stats.last_timestamp;
    if (receiver.engine_v2_ready && receiver.seek_backlog_ended &&
        (int32_t)(last_read - rtp_time) > 0) {
      above_from = last_read;
    }
    receiver.seek_backlog_valid = false;
    receiver.seek_backlog_ended = false;
    receiver.discard_before_rtp = rtp_time;
    receiver.discard_before_rtp_valid = true;
    receiver.discard_above_rtp = above_from + gate_window;
    receiver.discard_above_rtp_valid = true;
    ESP_LOGI(
        TAG, "RTP gates armed on anchor: discard_before=%lu discard_above=%lu",
        (unsigned long)rtp_time, (unsigned long)(above_from + gate_window));
    // What arrived between the flush and this anchor was kept: an iPhone
    // streams the new position for seconds before it sends the anchor, and
    // rejecting it all left the start waiting for the clock to reach the
    // first frame kept, 6-9 s of silence after a seek.  Blocks before the
    // anchor are of no use.  Blocks past the window are not stale: the phone
    // fills far beyond 10 s, and dropping them cut a hole into the fill.
    if (receiver.engine_v2_ready) {
      audio_timeline_t *timeline = &receiver.engine_v2.timeline;
      const uint32_t epoch = audio_epoch_get(&receiver.engine_v2.epoch);
      const size_t stale =
          audio_timeline_trim_before(timeline, epoch, rtp_time);
      if (stale > 0U) {
        ESP_LOGI(TAG, "Dropped %u blocks before the anchor", (unsigned)stale);
      }
    }
  }

  // A second pass used to re-check the sorted buffer's oldest frame here,
  // because a frame queued before seek_flush could still be sitting below the
  // new anchor.  The timeline needs no such sweep: blocks are addressed by
  // RTP, so anything stranded below the cursor is never scheduled.

  // Pin the PTP clock to the master announced by the anchor packet's
  // clock_id field.  Without this, ptp_clock can lock to any PTP master
  // on the LAN (HomePods, AppleTVs, NTP-PTP gateways) and produce offsets
  // that have nothing to do with the AirPlay sender's clock domain.
  // clock_id == 0 happens on the AirPlay 1 NTP path; in that case leave
  // the filter as-is (set or cleared by a prior 0xD7 anchor / TEARDOWN).
  if (clock_id != 0) {
    ptp_clock_set_master_clock_id(clock_id);
  }

  audio_timing_set_anchor(&receiver.timing, &receiver.stream->format, clock_id,
                          network_time_ns, rtp_time);

  if (engine_v2_active()) {
    // Keep the clock map in the sender's clock domain and let
    // audio_receiver_read() convert local time into it with the live offset,
    // exactly as audio_timing.c's compute_early_us() re-reads the offset on
    // every frame.  Baking the offset into the anchor here instead strands the
    // stream whenever the anchor lands before a lock: SETPEERS zeroes the
    // offset, so the anchor would sit days in the future and never be reached.
    receiver.engine_v2_anchor_rtp = rtp_time;
    receiver.engine_v2_anchor_network_ns = network_time_ns;
    // A timeline ID is only present on the AirPlay 2 (PTP) anchors; the
    // AirPlay 1 sync packet path passes 0.
    receiver.engine_v2_anchor_uses_ptp = (clock_id != 0);
    receiver.engine_v2_playout_offset_ns =
        ((int64_t)receiver.timing.playout_latency_samples * 1000000000LL) /
        sample_rate;
    receiver.engine_v2_anchor_pending = true;
    audio_receiver_arm_engine_v2_anchor();
    if (receiver.engine_v2_anchor_pending) {
      // No usable clock yet.  Keep the buffered PCM and wait rather than
      // discarding a full pre-buffer.
      audio_engine_v2_wait_for_anchor(&receiver.engine_v2,
                                      esp_timer_get_time());
    }
  }
}

void audio_receiver_set_playing(bool playing) {
  audio_timing_set_playing(&receiver.timing, playing);
  if (receiver.engine_v2_ready) {
    audio_engine_v2_set_playing(&receiver.engine_v2, playing);
  }
  if (!playing) {
    receiver.blocks_read_in_sequence = 0;
  }
}

void audio_receiver_reset_timing(void) {
  audio_timing_reset(&receiver.timing);
}

bool audio_receiver_is_playing(void) {
  return receiver.timing.playing;
}

void audio_receiver_set_stream_type(audio_stream_type_t type) {
  if (!receiver.realtime_stream || !receiver.buffered_stream) {
    return;
  }
  audio_stream_t *target = receiver.realtime_stream;
  if (type == AUDIO_STREAM_BUFFERED) {
    target = receiver.buffered_stream;
  }

  if (!target) {
    return;
  }

  if (receiver.stream != target) {
    if (receiver.stream) {
      audio_receiver_copy_stream_state(target, receiver.stream);
      if (receiver.stream->running && receiver.stream->ops &&
          receiver.stream->ops->stop) {
        receiver.stream->ops->stop(receiver.stream);
      }
    }
    receiver.stream = target;
  }

  receiver.stream->type = type;
}

esp_err_t audio_receiver_start(uint16_t data_port, uint16_t control_port) {
  esp_err_t engine_err = audio_receiver_ensure_engine_v2(AUDIO_STREAM_REALTIME);
  if (engine_err != ESP_OK) {
    return engine_err;
  }

  audio_receiver_set_stream_type(AUDIO_STREAM_REALTIME);

  if (!receiver.stream || !receiver.stream->ops ||
      !receiver.stream->ops->start) {
    return ESP_FAIL;
  }

  // Always stop and restart fresh
  if (receiver.stream->running) {
    receiver.stream->ops->stop(receiver.stream);
  }

  receiver.data_port = data_port;
  receiver.control_port = control_port;

  // Starting a stream resets all timing state (including pause tracking)
  audio_receiver_reset_stats();
  audio_receiver_reset_engine_v2();
  audio_timing_reset(&receiver.timing);
  // audio_timing defaults to playing; mirror that onto the engine, which
  // starts paused, so a sender that never sends an explicit rate=1 still
  // produces audio.
  audio_engine_v2_set_playing(&receiver.engine_v2, receiver.timing.playing);
  audio_receiver_reset_resend_state();

  receiver.timing.ptp_locked = ptp_clock_is_locked();
  audio_receiver_reset_blocks();

  return receiver.stream->ops->start(receiver.stream, data_port);
}

esp_err_t audio_receiver_start_buffered(uint16_t tcp_port) {
  esp_err_t engine_err = audio_receiver_ensure_engine_v2(AUDIO_STREAM_BUFFERED);
  if (engine_err != ESP_OK) {
    return engine_err;
  }

  audio_receiver_set_stream_type(AUDIO_STREAM_BUFFERED);

  if (!receiver.stream || !receiver.stream->ops ||
      !receiver.stream->ops->start) {
    return ESP_FAIL;
  }

  // Buffered streams use a fixed port, no need to restart if running
  if (receiver.stream->running) {
    return ESP_OK;
  }

  // Starting a stream resets all timing state (including pause tracking)
  audio_receiver_reset_stats();
  audio_receiver_reset_engine_v2();
  audio_timing_reset(&receiver.timing);
  // audio_timing defaults to playing; mirror that onto the engine, which
  // starts paused, so a sender that never sends an explicit rate=1 still
  // produces audio.
  audio_engine_v2_set_playing(&receiver.engine_v2, receiver.timing.playing);
  audio_receiver_reset_resend_state();

  receiver.timing.ptp_locked = ptp_clock_is_locked();
  audio_receiver_reset_blocks();

  return receiver.stream->ops->start(receiver.stream, tcp_port);
}

esp_err_t audio_receiver_start_stream(uint16_t data_port, uint16_t control_port,
                                      uint16_t tcp_port) {
  if (!receiver.stream) {
    return ESP_FAIL;
  }
  if (receiver.stream->type == AUDIO_STREAM_BUFFERED) {
    return audio_receiver_start_buffered(tcp_port);
  }

  return audio_receiver_start(data_port, control_port);
}

uint16_t audio_receiver_get_stream_port(void) {
  if (!receiver.stream || !receiver.stream->ops ||
      !receiver.stream->ops->get_port) {
    return 0;
  }

  return receiver.stream->ops->get_port(receiver.stream);
}

void audio_receiver_set_client_control(uint32_t client_ip,
                                       uint16_t client_control_port) {
  if (client_ip == 0 || client_control_port == 0) {
    receiver.retransmit_enabled = false;
    audio_receiver_reset_resend_state();
    return;
  }
  memset(&receiver.client_control_addr, 0,
         sizeof(receiver.client_control_addr));
  receiver.client_control_addr.sin_family = AF_INET;
  receiver.client_control_addr.sin_addr.s_addr = client_ip;
  receiver.client_control_addr.sin_port = htons(client_control_port);
  receiver.retransmit_enabled = true;
  audio_receiver_reset_resend_state();
  ESP_LOGI(TAG, "NACK retransmission enabled, client control port %u",
           client_control_port);
}

// Stopping the receiver does not give up the claim on the output: a
// stream-level TEARDOWN is a pause, and handing the speaker to Sendspin there
// would have Music Assistant playing out loud the moment the phone pauses.
// Only audio_receiver_end_session() releases it.
void audio_receiver_stop(void) {
  if (receiver.realtime_stream && receiver.realtime_stream->ops &&
      receiver.realtime_stream->ops->stop) {
    receiver.realtime_stream->ops->stop(receiver.realtime_stream);
  }

  if (receiver.buffered_stream && receiver.buffered_stream->ops &&
      receiver.buffered_stream->ops->stop) {
    receiver.buffered_stream->ops->stop(receiver.buffered_stream);
  }

  // Stop the decode worker before the decoder it uses goes away.  The engine
  // itself is kept: the playback task renders from it on every I2S refill.
  // A new epoch first: the worker may be waiting on a full timeline that a
  // paused scheduler will never drain, and only an epoch change ends that.
  audio_receiver_reset_engine_v2();
  audio_decode_worker_destroy(receiver.decode_worker);
  receiver.decode_worker = NULL;

  if (receiver.decoder_mutex) {
    xSemaphoreTake(receiver.decoder_mutex, portMAX_DELAY);
  }
  audio_decoder_destroy(receiver.decoder);
  receiver.decoder = NULL;
  if (receiver.decoder_mutex) {
    xSemaphoreGive(receiver.decoder_mutex);
  }

  if (receiver.realtime_stream) {
    memset(&receiver.realtime_stream->encrypt, 0,
           sizeof(receiver.realtime_stream->encrypt));
  }
  if (receiver.buffered_stream) {
    memset(&receiver.buffered_stream->encrypt, 0,
           sizeof(receiver.buffered_stream->encrypt));
  }

  receiver.retransmit_enabled = false;
  memset(&receiver.client_control_addr, 0,
         sizeof(receiver.client_control_addr));
  audio_receiver_reset_resend_state();
  receiver.seek_backlog_valid = false;
  receiver.seek_backlog_ended = false;

  audio_receiver_flush();
}

void audio_receiver_stop_buffered_only(void) {
  if (receiver.buffered_stream && receiver.buffered_stream->ops &&
      receiver.buffered_stream->ops->stop) {
    receiver.buffered_stream->ops->stop(receiver.buffered_stream);
  }
}

void audio_receiver_get_stats(audio_stats_t *stats) {
  if (!stats) {
    return;
  }
  memcpy(stats, &receiver.stats, sizeof(receiver.stats));
}

size_t audio_receiver_read(int16_t *buffer, size_t samples) {
  if (!buffer || samples == 0) {
    return 0;
  }

  if (engine_v2_active()) {
    audio_receiver_arm_engine_v2_anchor();
    audio_receiver_arm_rate_anchor();
    // Which clock the anchor is read through: the session's for a buffered
    // stream, the anchor's own for a realtime one.
    const bool buffered = receiver.stream == receiver.buffered_stream;
    const bool ptp = buffered ? receiver.session_timing_ptp
                              : receiver.engine_v2_anchor_uses_ptp;
    const audio_playout_clock_kind_t kind =
        ptp ? AUDIO_PLAYOUT_CLOCK_PTP : AUDIO_PLAYOUT_CLOCK_NTP;
    const uint32_t reset_req =
        __atomic_load_n(&receiver.playout_clock_reset_req, __ATOMIC_ACQUIRE);
    if (receiver.playout_clock.kind != kind ||
        reset_req != receiver.playout_clock_reset_seen) {
      receiver.playout_clock_reset_seen = reset_req;
      audio_playout_clock_reset(&receiver.playout_clock, kind);
    }
    // The scheduler works in the sender's clock domain.  The output instant
    // is converted through the clock's offset+rate model by the engine, under
    // the lock that also guards the anchor.
    const int64_t playout_local_ns =
        audio_output_get_next_playout_time_ns(esp_timer_get_time());
    return audio_engine_v2_render_local(&receiver.engine_v2, playout_local_ns,
                                        &receiver.playout_clock, buffer,
                                        samples);
  }

  return 0;
}

bool audio_receiver_has_data(void) {
  return engine_v2_active() &&
         audio_timeline_count(&receiver.engine_v2.timeline) > 0;
}

void audio_receiver_flush(void) {
  // Flush is an explicit reset — clear all timing state including pause
  // tracking.  The sender will provide fresh anchor times after flush.
  // Also disarm any pending deferred flush so it does not fire on the
  // next track's frames.
  audio_receiver_reset_engine_v2();
  audio_timing_reset(&receiver.timing);
  audio_receiver_reset_resend_state();

  receiver.discard_before_rtp_valid = false;
  receiver.discard_above_rtp_valid = false;
  receiver.arm_gate_on_next_anchor = false;
  receiver.blocks_read_in_sequence = 1;
}

void audio_receiver_seek_flush(void) {
  // Mid-stream seek flush (FLUSH / immediate FLUSHBUFFERED).  Same as
  // audio_receiver_flush(); the timeline re-prerolls from the new anchor by
  // itself.  Also disarms any pending deferred flush (audio_timing_reset
  // clears it).
  //
  // What arrives before that anchor is kept.  An iPhone streams the new
  // position's audio straight after FLUSHBUFFERED and sends the anchor 2-3 s
  // later; a blanket gate here threw those seconds away, and the start then
  // waited for the clock to reach the first frame kept.  Only the old
  // position's backlog is dropped: it carries on the RTP sequence from the
  // last frame before the flush.  That gate is armed before the flush.  Armed
  // after it, the reader slipped a backlog frame into the new epoch between
  // the two, and as the timeline's first block it fixed the RTP phase, so the
  // new position's frames were phase-blocked and the decoder stalled.
  receiver.seek_backlog_next_rtp =
      receiver.stats.last_timestamp + receiver.engine_v2.timeline.frame_samples;
  receiver.seek_backlog_dropped = 0;
  receiver.seek_backlog_valid = receiver.engine_v2_ready;
  receiver.seek_backlog_ended = false;
  audio_receiver_flush();
  // Request that the RTP gate be armed as soon as the next anchor arrives.
  // This covers the forward-seek case where the buffer is already empty by
  // the time SETRATEANCHORTIME arrives, so the seek-detection heuristic
  // (which needs oldest_rtp from the buffer) would otherwise miss arming it.
  receiver.arm_gate_on_next_anchor = true;
}

void audio_receiver_realtime_flush(bool until_valid, uint32_t until_rtp) {
  if (receiver.stream != receiver.realtime_stream) {
    audio_receiver_seek_flush();
    return;
  }
  audio_receiver_flush();
  receiver.seek_backlog_valid = false;
  receiver.seek_backlog_ended = false;
  // A realtime sender streams the new audio over UDP straight after FLUSH,
  // and macOS sends its next anchor ~1.6 s later.  The blanket gate of a
  // buffered seek flush threw all of that away, and the start then waited
  // for the play position to reach the first packet it kept.  No old-track
  // backlog queues behind a UDP socket the way it does behind the buffered
  // TCP one: what is stale is from before the flush point, and below it.
  if (until_valid) {
    receiver.discard_before_rtp = until_rtp;
    receiver.discard_before_rtp_valid = true;
  }
  receiver.arm_gate_on_next_anchor = true;
}

bool audio_receiver_flush_buffered(const audio_flush_record_t *record,
                                   uint32_t *reply_rtp) {
  if (!record || !receiver.engine_v2_ready) {
    return false;
  }
  taskENTER_CRITICAL(&receiver.flush_lock);
  const bool at_once = audio_flush_queue_push(&receiver.flush_queue, record);
  const size_t pending = audio_flush_queue_pending(&receiver.flush_queue);
  taskEXIT_CRITICAL(&receiver.flush_lock);

  if (!record->has_from) {
    // Flush-all: rate 0 and no anchor until the next SETRATEANCHORTIME.  The
    // epoch is deliberately kept: blocks at or past untilSeq that already
    // arrived belong to the new position and stay.
    taskENTER_CRITICAL(&receiver.anchor_lock);
    receiver.rate_anchor_pending = false;
    taskEXIT_CRITICAL(&receiver.anchor_lock);
    audio_timing_set_playing(&receiver.timing, false);
    audio_engine_v2_stop(&receiver.engine_v2);
  }
  // Everything already decoded in the range goes now; what is still queued
  // or arriving is dropped by the decode worker as it gets there.
  const size_t dropped =
      audio_engine_v2_flush_seq_range(&receiver.engine_v2, record->has_from,
                                      record->from_seq, record->until_seq);

  bool have_rtp = false;
  if (at_once && reply_rtp) {
    have_rtp =
        audio_engine_v2_last_rendered_rtp(&receiver.engine_v2, reply_rtp);
    if (!have_rtp && receiver.stats.packets_received > 0U) {
      *reply_rtp = receiver.stats.last_timestamp;
      have_rtp = true;
    }
  }
  ESP_LOGI(TAG,
           "FLUSHBUFFERED %s: seq [%s%" PRIu32 ", %" PRIu32 ") ts [%" PRIu32
           ", %" PRIu32 ") dropped %u buffered, %u record(s) pending",
           record->has_from ? "deferred" : "immediate",
           record->has_from ? "" : "..", record->from_seq, record->until_seq,
           record->from_ts, record->until_ts, (unsigned)dropped,
           (unsigned)pending);
  return have_rtp;
}

void audio_receiver_set_session_timing(bool ptp) {
  receiver.session_timing_ptp = ptp;
  __atomic_add_fetch(&receiver.playout_clock_reset_req, 1U, __ATOMIC_RELEASE);
}

void audio_receiver_set_buffered_latency_samples(uint32_t latency_samples) {
  receiver.buffered_latency_samples = latency_samples;
}

void audio_receiver_set_rate_anchor(const audio_rate_anchor_t *anchor) {
  if (!anchor || !receiver.stream) {
    return;
  }
  if (anchor->rate == 0U) {
    audio_receiver_pause();
    return;
  }

  int sample_rate = receiver.stream->format.sample_rate;
  if (sample_rate <= 0) {
    sample_rate = 44100;
  }
  // The anchor names the timeline.  Without this, ptp_clock can lock to any
  // PTP master on the LAN and produce offsets that have nothing to do with
  // the sender's clock domain.
  if (receiver.session_timing_ptp && anchor->timeline_id != 0) {
    ptp_clock_set_master_clock_id(anchor->timeline_id);
  }
  audio_timing_set_anchor(&receiver.timing, &receiver.stream->format,
                          anchor->timeline_id, anchor->network_ns, anchor->rtp);

  const audio_engine_anchor_t engine_anchor = {
      .rtp = anchor->rtp,
      .network_ns = anchor->network_ns,
      .timeline_id = receiver.session_timing_ptp ? anchor->timeline_id : 0,
      .playout_offset_ns =
          ((int64_t)receiver.buffered_latency_samples * 1000000000LL) /
          sample_rate,
      .fas_valid = anchor->fas_valid,
      .fas_rtp = anchor->fas_rtp,
  };
  taskENTER_CRITICAL(&receiver.anchor_lock);
  receiver.rate_anchor = engine_anchor;
  receiver.rate_anchor_pending = true;
  taskEXIT_CRITICAL(&receiver.anchor_lock);

  audio_receiver_arm_rate_anchor();
  taskENTER_CRITICAL(&receiver.anchor_lock);
  const bool deferred = receiver.rate_anchor_pending;
  taskEXIT_CRITICAL(&receiver.anchor_lock);
  if (deferred) {
    receiver.rate_anchor_deferred++;
    ESP_LOGW(TAG,
             "Anchor rtp=%" PRIu32 " waits for the %s clock (timeline %016llx)",
             anchor->rtp, receiver.session_timing_ptp ? "PTP" : "NTP",
             (unsigned long long)anchor->timeline_id);
  }
  audio_receiver_set_playing(true);
}

void audio_receiver_get_diag(audio_receiver_diag_t *diag) {
  if (!diag) {
    return;
  }
  memset(diag, 0, sizeof(*diag));
  diag->state = "none";
  if (!receiver.engine_v2_ready) {
    return;
  }
  // Plain reads of counters other tasks own: a status line can stand a value
  // one update old.
  const audio_engine_v2_t *engine = &receiver.engine_v2;
  const audio_scheduler_t *scheduler = &engine->scheduler;
  diag->anchors_installed = engine->anchors_installed;
  diag->anchors_consistent = engine->anchors_consistent;
  diag->anchor_resets = scheduler->anchor_resets;
  diag->anchors_deferred = receiver.rate_anchor_deferred;
  diag->gm_rebases = receiver.playout_clock.rebases;
  diag->gm_rebase_failures = receiver.playout_clock.rebase_failures;
  diag->freewheels = receiver.playout_clock.freewheel_entries;
  diag->freewheeling = receiver.playout_clock.freewheeling;
  diag->steps = scheduler->drift_steps;
  diag->step_clock_deferrals = scheduler->drift_step_clock_deferrals;
  diag->trims = scheduler->drift_servo_trims;
  const int sample_rate = engine->format.sample_rate > 0
                              ? engine->format.sample_rate
                              : DEFAULT_SAMPLE_RATE;
  diag->error_us =
      (int32_t)(((int64_t)scheduler->playout_error_samples * 1000000LL) /
                sample_rate);
  diag->flushes_pending = (uint32_t)receiver.flush_queue.count;
  diag->flush_dropped = receiver.flush_queue.dropped;
  diag->buffered_blocks = (uint32_t)engine->timeline.count;
  diag->state = audio_scheduler_state_name(scheduler->state);
}

void audio_receiver_pause(void) {
  // Stop the consumer.  The receiver tasks keep running so the audio buffer
  // continues to fill with pre-buffered audio — TCP back-pressure naturally
  // throttles the sender.  On resume the phone sends a fresh
  // SETRATEANCHORTIME anchor that re-aligns the buffered frames to the
  // correct wall-clock position; no flush or offset compensation is needed.
  audio_timing_set_playing(&receiver.timing, false);
  if (receiver.engine_v2_ready) {
    if (receiver.stream == receiver.buffered_stream) {
      // Rate 0 on Apple flushes the output and invalidates the anchor but
      // keeps every buffered packet.  Forgetting the anchor matters: the
      // resume's anchor may still be waiting for the clock, and a scheduler
      // restarted on the old one would preroll at a stale position.
      taskENTER_CRITICAL(&receiver.anchor_lock);
      receiver.rate_anchor_pending = false;
      taskEXIT_CRITICAL(&receiver.anchor_lock);
      audio_engine_v2_stop(&receiver.engine_v2);
    } else {
      audio_engine_v2_set_playing(&receiver.engine_v2, false);
    }
  }
  receiver.blocks_read_in_sequence = 0;
}

uint16_t audio_receiver_get_buffered_port(void) {
  return receiver.buffered_port;
}

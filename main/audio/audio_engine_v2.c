// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "audio_engine_v2.h"

#include <inttypes.h>
#include <string.h>

#include <stdlib.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "audio_v2";

/* Longest single blocking slice used while waiting for timeline space.  A
 * shorter slice keeps the decode task responsive to epoch changes even if the
 * consumer never signals. */
#define AUDIO_V2_PUSH_WAIT_SLICE_MS 50U

/* A rate-1 anchor that puts the RTP being played within this of where the
 * current timeline has it is the same timeline restated, and is left alone.
 * Senders re-anchor with values computed from their own clock reading, so
 * the restatement is never bit-exact; a millisecond covers that and is still
 * far below anything audible as a sync error.  Apple's receiver ignores a
 * rate-1 re-anchor while running outright; this only does so when it is
 * consistent, and otherwise moves playout onto the new timeline. */
#define ANCHOR_CONSISTENT_US 1000

static void control_lock(audio_engine_v2_t *engine);
static void control_unlock(audio_engine_v2_t *engine);

/* Playout is about to stop: keep the next few milliseconds of the stream,
 * faded to silence, for the renders that follow.  Apple cuts hard here; on a
 * DAC without a mute line that is a click at every pause, seek and skip.
 * Taken now, while the audio at the cursor is still in the timeline, because
 * a flush sweeps it straight after.  A tail not yet played is kept: it
 * already ends in silence.  Caller holds control_mutex. */
static void take_tail_locked(audio_engine_v2_t *engine) {
  if (!engine->tail || engine->format.sample_rate <= 0 ||
      engine->tail_pos < engine->tail_frames) {
    return;
  }
  uint32_t frames =
      audio_scheduler_fade_out_frames((uint32_t)engine->format.sample_rate);
  if (frames > AUDIO_V2_TAIL_MAX_FRAMES) {
    frames = AUDIO_V2_TAIL_MAX_FRAMES;
  }
  const size_t taken = audio_scheduler_take_tail(
      &engine->scheduler, &engine->timeline, engine->tail, frames,
      (uint8_t)engine->format.channels);
  if (taken > 0U) {
    engine->tail_frames = (uint32_t)taken;
    engine->tail_pos = 0;
    engine->tails++;
  }
}

/* The fade-out, ahead of anything else; the rest of the block is silent.
 * Caller holds control_mutex.  Returns false when there is none. */
static bool emit_tail_locked(audio_engine_v2_t *engine, int16_t *out,
                             size_t samples) {
  if (engine->tail_pos >= engine->tail_frames) {
    return false;
  }
  const size_t channels = (size_t)engine->format.channels;
  const uint32_t left = engine->tail_frames - engine->tail_pos;
  const size_t n = samples < left ? samples : left;
  memcpy(out, &engine->tail[(size_t)engine->tail_pos * channels],
         n * channels * sizeof(int16_t));
  memset(&out[n * channels], 0, (samples - n) * channels * sizeof(int16_t));
  engine->tail_pos += (uint32_t)n;
  return true;
}

/* Startup watchdog: first report, then the interval between repeats. */
#define START_STALL_AFTER_US  3000000LL
#define START_STALL_REPEAT_US 10000000LL

esp_err_t audio_engine_v2_init(audio_engine_v2_t *engine,
                               const audio_format_t *format,
                               uint32_t frame_samples, uint16_t capacity) {
  if (!engine || !format || format->sample_rate <= 0 || format->channels <= 0 ||
      format->channels > AUDIO_V2_MAX_CHANNELS) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(engine, 0, sizeof(*engine));
  engine->publish_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
  audio_epoch_init(&engine->epoch);
  audio_clock_map_reset(&engine->clock_map);
  esp_err_t err =
      audio_timeline_init(&engine->timeline, capacity, frame_samples);
  if (err != ESP_OK) {
    return err;
  }
  engine->control_mutex = xSemaphoreCreateMutex();
  if (!engine->control_mutex) {
    audio_timeline_deinit(&engine->timeline);
    return ESP_ERR_NO_MEM;
  }
  /* Without it a stop cuts hard, as it always used to. */
  const size_t tail_bytes = (size_t)AUDIO_V2_TAIL_MAX_FRAMES *
                            AUDIO_V2_MAX_CHANNELS * sizeof(int16_t);
  engine->tail =
      heap_caps_malloc(tail_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!engine->tail) {
    engine->tail = malloc(tail_bytes);
  }

  engine->format = *format;
  uint32_t preroll_samples =
      (uint32_t)((uint64_t)format->sample_rate * 180U / 1000U);
  audio_scheduler_init(&engine->scheduler, preroll_samples, 1000000LL);
  engine->playing = false;
  engine->initialized = true;
  audio_scheduler_begin_epoch(&engine->scheduler,
                              audio_epoch_get(&engine->epoch), 0);
  ESP_LOGI(TAG,
           "initialized: %d Hz, %d ch, frame=%" PRIu32
           " blocks=%u preroll=%" PRIu32,
           format->sample_rate, format->channels, frame_samples,
           (unsigned)capacity, preroll_samples);
  return ESP_OK;
}

void audio_engine_v2_deinit(audio_engine_v2_t *engine) {
  if (!engine) {
    return;
  }
  audio_timeline_deinit(&engine->timeline);
  if (engine->control_mutex) {
    vSemaphoreDelete(engine->control_mutex);
  }
  free(engine->tail);
  memset(engine, 0, sizeof(*engine));
}

uint32_t audio_engine_v2_begin_epoch(audio_engine_v2_t *engine,
                                     int64_t now_us) {
  if (!engine || !engine->initialized) {
    return 0;
  }
  /* Before the slots go: a flush or teardown fades out of what was playing. */
  control_lock(engine);
  take_tail_locked(engine);
  control_unlock(engine);
  portENTER_CRITICAL(&engine->publish_lock);
  uint32_t next = audio_epoch_advance(&engine->epoch);
  audio_timeline_clear_slots(&engine->timeline);
  portEXIT_CRITICAL(&engine->publish_lock);
  /* Outside the critical section: waking a producer blocked on a full
   * timeline has to happen with interrupts enabled. */
  audio_timeline_signal_space(&engine->timeline);
  control_lock(engine);
  audio_clock_map_reset(&engine->clock_map);
  audio_scheduler_begin_epoch(&engine->scheduler, next, now_us);
  control_unlock(engine);
  engine->concealed_samples = 0;
  engine->blocks_inserted = 0;
  engine->blocks_rejected = 0;
  engine->last_status_log_us = 0;
  engine->conceal_events = 0;
  engine->conceal_events_logged = 0;
  // audio_scheduler_begin_epoch() zeroed the live trim count, so the snapshot
  // has to follow it: otherwise the next status line subtracts the old total
  // from zero and reports the per-second rate as a wrapped uint32.
  engine->drift_servo_trims_logged = 0;
  engine->last_conceal_gap_rtp = 0;
  __atomic_store_n(&engine->diag_rx_packets, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_gate_drops, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_enqueue_ok, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_enqueue_retries, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_queue_drops, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_dequeued, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_epoch_drops, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_decode_ok, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_decode_fail, 0, __ATOMIC_RELAXED);
  __atomic_store_n(&engine->diag_pcm_inserted, 0, __ATOMIC_RELAXED);
  engine->start_watchdog_armed = false;
  engine->start_watchdog_anchor_us = 0;
  engine->start_watchdog_report_us = 0;
  engine->start_watchdog_epoch = next;
  engine->start_watchdog_anchor_rtp = 0;
  ESP_LOGI(TAG, "new epoch=%" PRIu32, next);
  return next;
}

void audio_engine_v2_set_format(audio_engine_v2_t *engine,
                                const audio_format_t *format) {
  if (!engine || !format || format->sample_rate <= 0 || format->channels <= 0 ||
      format->channels > AUDIO_V2_MAX_CHANNELS) {
    return;
  }
  control_lock(engine);
  engine->format = *format;
  engine->scheduler.preroll_samples =
      (uint32_t)((uint64_t)format->sample_rate * 180U / 1000U);
  control_unlock(engine);
}

bool audio_engine_v2_set_frame_samples(audio_engine_v2_t *engine,
                                       uint32_t frame_samples) {
  if (!engine || !engine->initialized) {
    return false;
  }
  if (engine->timeline.frame_samples == frame_samples) {
    return true;
  }
  if (!audio_timeline_set_frame_samples(&engine->timeline, frame_samples)) {
    ESP_LOGE(TAG, "frame=%" PRIu32 " does not fit the slot stride",
             frame_samples);
    return false;
  }
  ESP_LOGI(TAG, "frame=%" PRIu32 " (%u blocks)", frame_samples,
           (unsigned)engine->timeline.capacity);
  return true;
}

/* Startup watchdog report.  Logs the engine's state once a start has waited
 * START_STALL_AFTER_US past its first anchor, then every
 * START_STALL_REPEAT_US for as long as it stays stuck.  Called from the
 * render path and from every anchor, so a stall still shows when nothing is
 * rendering. */
static void control_lock(audio_engine_v2_t *engine) {
  if (engine->control_mutex) {
    (void)xSemaphoreTake(engine->control_mutex, portMAX_DELAY);
  }
}

static void control_unlock(audio_engine_v2_t *engine) {
  if (engine->control_mutex) {
    (void)xSemaphoreGive(engine->control_mutex);
  }
}

static void start_watchdog_check(audio_engine_v2_t *engine, int64_t now_us) {
  if (!engine->start_watchdog_armed ||
      engine->start_watchdog_epoch != engine->scheduler.epoch ||
      engine->scheduler.state == AUDIO_SCHED_PLAYING ||
      engine->start_watchdog_anchor_us <= 0 ||
      now_us - engine->start_watchdog_anchor_us < START_STALL_AFTER_US ||
      now_us - engine->start_watchdog_report_us < START_STALL_REPEAT_US) {
    return;
  }
  engine->start_watchdog_report_us = now_us;
  audio_timeline_diag_t diag;
  memset(&diag, 0, sizeof(diag));
  (void)audio_timeline_get_diag(&engine->timeline, engine->scheduler.epoch,
                                engine->scheduler.wanted_rtp, &diag);
  ESP_LOGE(TAG,
           "START STALL: %" PRId64 " s since anchor, renders=%" PRIu64
           " epoch=%" PRIu32 " anchor=%" PRIu32
           " playing=%d state=%s reason=%s clock=%d wanted=%" PRIu32
           " blocks=%u oldest=%s%" PRIu32 " newest_end=%s%" PRIu32
           " covered=%d next=%s%" PRIu32 " contiguous=%" PRIu32 "/%" PRIu32
           " attempts=%" PRIu64 " fallback=%" PRIu64 " silent=%" PRIu64
           " inserted=%" PRIu64 " rejected=%" PRIu64,
           (now_us - engine->start_watchdog_anchor_us) / 1000000LL,
           __atomic_load_n(&engine->render_calls, __ATOMIC_RELAXED),
           engine->scheduler.epoch, engine->start_watchdog_anchor_rtp,
           engine->playing ? 1 : 0,
           audio_scheduler_state_name(engine->scheduler.state),
           audio_scheduler_wait_reason_name(engine->scheduler.wait_reason),
           engine->clock_map.valid ? 1 : 0, engine->scheduler.wanted_rtp,
           (unsigned)diag.count, diag.has_oldest ? "" : "n/a:", diag.oldest_rtp,
           diag.has_newest ? "" : "n/a:", diag.newest_end_rtp,
           diag.target_covered ? 1 : 0,
           diag.has_next ? "" : "n/a:", diag.next_rtp, diag.contiguous_samples,
           engine->scheduler.preroll_samples, engine->scheduler.start_attempts,
           engine->scheduler.fallback_attempts,
           engine->scheduler.silent_render_calls, engine->blocks_inserted,
           engine->blocks_rejected);
  ESP_LOGE(TAG,
           "PIPELINE: rx=%" PRIu64 " gate_drop=%" PRIu64 " enq=%" PRIu64
           " enq_retry=%" PRIu64 " queue_drop=%" PRIu64 " deq=%" PRIu64
           " epoch_drop=%" PRIu64 " decode_ok=%" PRIu64 " decode_fail=%" PRIu64
           " pcm_insert=%" PRIu64,
           __atomic_load_n(&engine->diag_rx_packets, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_gate_drops, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_enqueue_ok, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_enqueue_retries, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_queue_drops, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_dequeued, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_epoch_drops, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_decode_ok, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_decode_fail, __ATOMIC_RELAXED),
           __atomic_load_n(&engine->diag_pcm_inserted, __ATOMIC_RELAXED));
}

bool audio_engine_v2_set_anchor(audio_engine_v2_t *engine, uint32_t anchor_rtp,
                                uint64_t anchor_network_ns,
                                int64_t playout_offset_ns) {
  if (!engine || !engine->initialized) {
    return false;
  }
  control_lock(engine);
  bool ok = audio_clock_map_set(
      &engine->clock_map, (uint32_t)engine->format.sample_rate, anchor_rtp,
      anchor_network_ns, playout_offset_ns);
  if (ok) {
    if (engine->scheduler.state == AUDIO_SCHED_WAIT_ANCHOR) {
      engine->scheduler.state = AUDIO_SCHED_PREROLL;
      engine->scheduler.wait_reason = AUDIO_SCHED_WAIT_PREROLL;
    }
    /* A realtime sender re-anchors every second or so.  Timing the stall
     * from the latest anchor would restart the clock each time and the
     * watchdog would never fire, so it runs from the attempt's first. */
    const int64_t now_us = esp_timer_get_time();
    if (!engine->start_watchdog_armed ||
        engine->start_watchdog_epoch != engine->scheduler.epoch) {
      engine->start_watchdog_armed = true;
      engine->start_watchdog_anchor_us = now_us;
      engine->start_watchdog_report_us = 0;
      engine->start_watchdog_epoch = engine->scheduler.epoch;
      engine->start_watchdog_anchor_rtp = anchor_rtp;
    }
  }
  control_unlock(engine);
  if (ok) {
    /* Outside the lock: it may log, and the playback task must never wait
     * behind a log line. */
    start_watchdog_check(engine, esp_timer_get_time());
  }
  return ok;
}

audio_engine_anchor_result_t
audio_engine_v2_set_rate_anchor(audio_engine_v2_t *engine,
                                const audio_engine_anchor_t *anchor) {
  if (!engine || !engine->initialized || !anchor ||
      anchor->network_ns > (uint64_t)INT64_MAX ||
      engine->format.sample_rate <= 0) {
    return AUDIO_ENGINE_ANCHOR_REJECTED;
  }
  const uint32_t sample_rate = (uint32_t)engine->format.sample_rate;
  audio_engine_anchor_result_t result = AUDIO_ENGINE_ANCHOR_INSTALLED;

  control_lock(engine);
  audio_scheduler_t *scheduler = &engine->scheduler;
  audio_clock_map_t *map = &engine->clock_map;
  const bool playing = scheduler->state == AUDIO_SCHED_PLAYING;
  if (playing && map->valid && map->sample_rate == sample_rate &&
      (map->timeline_id == anchor->timeline_id || map->timeline_id == 0 ||
       anchor->timeline_id == 0)) {
    /* Where the current timeline plays the anchor's RTP, against where the
     * new anchor wants it. */
    int64_t current_ns = 0;
    (void)audio_clock_map_rtp_to_network(map, anchor->rtp, &current_ns);
    const int64_t wanted_ns =
        (int64_t)anchor->network_ns + anchor->playout_offset_ns;
    const int64_t diff_ns = wanted_ns - current_ns;
    if (diff_ns <= ANCHOR_CONSISTENT_US * 1000LL &&
        diff_ns >= -ANCHOR_CONSISTENT_US * 1000LL) {
      result = AUDIO_ENGINE_ANCHOR_CONSISTENT;
    }
  }

  if (result == AUDIO_ENGINE_ANCHOR_CONSISTENT) {
    /* The timeline being played already says this.  The first audible sample
     * is not re-applied either: what it gates has long been rendered or sits
     * inside audio the sender has already committed to. */
    engine->anchors_consistent++;
  } else {
    (void)audio_clock_map_set(map, sample_rate, anchor->rtp, anchor->network_ns,
                              anchor->playout_offset_ns);
    map->timeline_id = anchor->timeline_id;
    audio_scheduler_new_anchor(scheduler, playing);
    /* The first audible sample gates a start.  Under a stream that is already
     * playing, the move onto the new timeline is the reset step; muting up
     * to a first audible sample that lies ahead of the cursor as well would
     * cut a hole into audio that is meant to carry on.  Apple's receiver
     * gates there too on its older output path; its default path only trims
     * the one buffer still to be enqueued that holds it. */
    audio_scheduler_set_first_audible(scheduler, anchor->fas_valid && !playing,
                                      anchor->fas_rtp);
    if (scheduler->state == AUDIO_SCHED_WAIT_ANCHOR) {
      scheduler->state = AUDIO_SCHED_PREROLL;
      scheduler->wait_reason = AUDIO_SCHED_WAIT_PREROLL;
      scheduler->preroll_started_us = esp_timer_get_time();
    }
    engine->anchors_installed++;
    const int64_t now_us = esp_timer_get_time();
    if (!engine->start_watchdog_armed ||
        engine->start_watchdog_epoch != scheduler->epoch) {
      engine->start_watchdog_armed = true;
      engine->start_watchdog_anchor_us = now_us;
      engine->start_watchdog_report_us = 0;
      engine->start_watchdog_epoch = scheduler->epoch;
      engine->start_watchdog_anchor_rtp = anchor->rtp;
    }
  }
  control_unlock(engine);
  return result;
}

void audio_engine_v2_stop(audio_engine_v2_t *engine) {
  if (!engine || !engine->initialized) {
    return;
  }
  control_lock(engine);
  take_tail_locked(engine);
  engine->playing = false;
  engine->start_watchdog_armed = false;
  audio_clock_map_reset(&engine->clock_map);
  audio_scheduler_set_paused(&engine->scheduler, true);
  audio_scheduler_set_first_audible(&engine->scheduler, false, 0);
  audio_scheduler_new_anchor(&engine->scheduler, false);
  control_unlock(engine);
}

size_t audio_engine_v2_flush_seq_range(audio_engine_v2_t *engine, bool has_from,
                                       uint32_t from_seq, uint32_t until_seq) {
  if (!engine || !engine->initialized) {
    return 0U;
  }
  /* The generation moves first: a decode that checked the flush queue before
   * the caller updated it then fails its publish and checks again, and one
   * that published before this point is caught by the sweep below. */
  portENTER_CRITICAL(&engine->publish_lock);
  engine->flush_generation++;
  const uint32_t epoch = audio_epoch_get(&engine->epoch);
  portEXIT_CRITICAL(&engine->publish_lock);
  return audio_timeline_discard_seq_range(&engine->timeline, epoch, has_from,
                                          from_seq, until_seq);
}

uint32_t audio_engine_v2_flush_generation(const audio_engine_v2_t *engine) {
  return engine ? __atomic_load_n(&engine->flush_generation, __ATOMIC_ACQUIRE)
                : 0U;
}

bool audio_engine_v2_last_rendered_rtp(audio_engine_v2_t *engine,
                                       uint32_t *rtp) {
  if (!engine || !engine->initialized || !rtp) {
    return false;
  }
  control_lock(engine);
  const bool ok = engine->scheduler.rendered_samples > 0U;
  *rtp = engine->scheduler.cursor_rtp;
  control_unlock(engine);
  return ok;
}

void audio_engine_v2_wait_for_anchor(audio_engine_v2_t *engine,
                                     int64_t now_us) {
  if (!engine || !engine->initialized) {
    return;
  }

  control_lock(engine);
  take_tail_locked(engine);
  audio_clock_map_reset(&engine->clock_map);
  engine->scheduler.state =
      engine->playing ? AUDIO_SCHED_WAIT_ANCHOR : AUDIO_SCHED_PAUSED;
  engine->scheduler.cursor_rtp = 0;
  engine->scheduler.wanted_rtp = 0;
  engine->scheduler.raw_playout_error_samples = 0;
  engine->scheduler.playout_error_samples = 0;
  engine->scheduler.filtered_playout_error_q16 = 0;
  engine->scheduler.max_abs_playout_error_samples = 0;
  engine->scheduler.estimated_drift_ppm = 0;
  engine->scheduler.drift_reference_error_q16 = 0;
  engine->scheduler.drift_reference_network_ns = 0;
  engine->scheduler.error_filter_valid = false;
  engine->scheduler.drift_servo_accum = 0;
  engine->scheduler.drift_servo_phase = 0;
  engine->scheduler.drift_servo_warmup = 0;
  engine->scheduler.preroll_started_us = now_us;
  control_unlock(engine);
}

void audio_engine_v2_set_playing(audio_engine_v2_t *engine, bool playing) {
  if (!engine || !engine->initialized) {
    return;
  }

  /* RTSP may repeat RECORD/rate=1 while a new epoch is already waiting for
   * its anchor or preroll.  The public playing flag and the scheduler state
   * are related, but they are not the same state machine: returning only
   * because playing was already true can leave a PAUSED/IDLE scheduler
   * stranded, while blindly calling set_paused(false) can kick a running
   * scheduler back to PREROLL. */
  control_lock(engine);
  engine->playing = playing;

  if (!playing) {
    engine->start_watchdog_armed = false;
    take_tail_locked(engine);
    if (engine->scheduler.state != AUDIO_SCHED_PAUSED) {
      audio_scheduler_set_paused(&engine->scheduler, true);
    }
    control_unlock(engine);
    return;
  }

  switch (engine->scheduler.state) {
  case AUDIO_SCHED_PLAYING:
  case AUDIO_SCHED_WAIT_ANCHOR:
  case AUDIO_SCHED_PREROLL:
  case AUDIO_SCHED_RECOVERING:
    /* Already active for this epoch.  Do not disturb cursor/preroll. */
    break;

  case AUDIO_SCHED_PAUSED:
  case AUDIO_SCHED_IDLE:
  default:
    /* A repeated playing=true must be able to revive a scheduler which
     * was paused by the probe/teardown sequence.  Choose the next state
     * from the actual clock-map validity instead of forcing PREROLL. */
    engine->scheduler.state =
        engine->clock_map.valid ? AUDIO_SCHED_PREROLL : AUDIO_SCHED_WAIT_ANCHOR;
    engine->scheduler.wait_reason = engine->clock_map.valid
                                        ? AUDIO_SCHED_WAIT_PREROLL
                                        : AUDIO_SCHED_WAIT_CLOCK_MAP;
    engine->scheduler.preroll_started_us = esp_timer_get_time();
    break;
  }
  control_unlock(engine);
}

bool audio_engine_v2_is_nearly_full(audio_engine_v2_t *engine) {
  return engine && engine->initialized &&
         audio_timeline_is_nearly_full(&engine->timeline);
}

bool audio_engine_v2_play_cursor(const audio_engine_v2_t *engine,
                                 uint32_t *rtp) {
  if (!engine || !rtp || !engine->initialized ||
      __atomic_load_n(&engine->scheduler.state, __ATOMIC_RELAXED) !=
          AUDIO_SCHED_PLAYING) {
    return false;
  }
  *rtp = __atomic_load_n(&engine->scheduler.cursor_rtp, __ATOMIC_RELAXED);
  return true;
}

/* `flush_generation` NULL: the block is not subject to FLUSHBUFFERED. */
static audio_engine_push_result_t
push_pcm(audio_engine_v2_t *engine, uint32_t epoch, uint32_t first_rtp,
         uint32_t seq_tag, const uint32_t *flush_generation, const int16_t *pcm,
         size_t samples, uint8_t channels) {
  if (!engine || !engine->initialized || !pcm || samples == 0U ||
      samples > engine->timeline.frame_samples || channels == 0U ||
      channels > AUDIO_V2_MAX_CHANNELS) {
    return AUDIO_ENGINE_PUSH_FAILED;
  }

  audio_timeline_reservation_t reservation = {0};
  int16_t *timeline_pcm = NULL;

  /* Phase 1: validate the current stream identity and reserve a private
   * timeline slot.  Keep publish_lock only for the short control-state
   * transaction; the potentially slow PSRAM copy happens after unlock. */
  portENTER_CRITICAL(&engine->publish_lock);

  if (!audio_epoch_matches(&engine->epoch, epoch)) {
    engine->blocks_rejected++;
    portEXIT_CRITICAL(&engine->publish_lock);
    return AUDIO_ENGINE_PUSH_FAILED;
  }
  if (flush_generation && *flush_generation != engine->flush_generation) {
    portEXIT_CRITICAL(&engine->publish_lock);
    return AUDIO_ENGINE_PUSH_FLUSHED;
  }

  if (!audio_timeline_reserve(&engine->timeline, epoch, first_rtp, seq_tag,
                              &reservation, &timeline_pcm)) {
    engine->blocks_rejected++;
    portEXIT_CRITICAL(&engine->publish_lock);
    return AUDIO_ENGINE_PUSH_FAILED;
  }

  /* A retransmitted/duplicate block that is already READY needs no second
   * payload copy.  Treat it exactly like the previous insert() no-op path. */
  if (reservation.duplicate) {
    engine->blocks_inserted++;
    (void)__atomic_add_fetch(&engine->diag_pcm_inserted, 1U, __ATOMIC_RELAXED);
    portEXIT_CRITICAL(&engine->publish_lock);
    return AUDIO_ENGINE_PUSH_OK;
  }

  portEXIT_CRITICAL(&engine->publish_lock);

  /* Stage 2 optimization: copy decoded PCM with no engine publish lock and no
   * timeline metadata lock held.  The reservation is private/WRITING, so
   * playback cannot observe a partially copied block. */
  memcpy(timeline_pcm, pcm, samples * channels * sizeof(int16_t));
  /* A short trailing packet still starts on a frame boundary, so keep the slot
   * a whole block and silence the tail rather than replaying the last epoch. */
  if (samples < engine->timeline.frame_samples) {
    memset(&timeline_pcm[samples * channels], 0,
           (engine->timeline.frame_samples - samples) * channels *
               sizeof(int16_t));
  }

  /* Phase 2: before publishing, revalidate stream identity. A FLUSH/seek can
   * advance epoch while PCM is copied outside the locks. */
  portENTER_CRITICAL(&engine->publish_lock);

  if (!audio_epoch_matches(&engine->epoch, epoch)) {
    audio_timeline_cancel(&engine->timeline, &reservation);
    engine->blocks_rejected++;
    portEXIT_CRITICAL(&engine->publish_lock);
    return AUDIO_ENGINE_PUSH_FAILED;
  }
  /* A sweep since the check: it could not see this block, so it is not
   * published until the caller has checked it against the new records. */
  if (flush_generation && *flush_generation != engine->flush_generation) {
    audio_timeline_cancel(&engine->timeline, &reservation);
    portEXIT_CRITICAL(&engine->publish_lock);
    return AUDIO_ENGINE_PUSH_FLUSHED;
  }

  if (!audio_timeline_commit(&engine->timeline, &reservation)) {
    /* commit() consumes/cancels a valid reservation on every terminal path. */
    engine->blocks_rejected++;
    portEXIT_CRITICAL(&engine->publish_lock);
    return AUDIO_ENGINE_PUSH_FAILED;
  }

  engine->blocks_inserted++;
  (void)__atomic_add_fetch(&engine->diag_pcm_inserted, 1U, __ATOMIC_RELAXED);

  portEXIT_CRITICAL(&engine->publish_lock);
  return AUDIO_ENGINE_PUSH_OK;
}

bool audio_engine_v2_push_pcm(audio_engine_v2_t *engine, uint32_t epoch,
                              uint32_t first_rtp, const int16_t *pcm,
                              size_t samples, uint8_t channels) {
  return push_pcm(engine, epoch, first_rtp, 0U, NULL, pcm, samples, channels) ==
         AUDIO_ENGINE_PUSH_OK;
}

static audio_engine_push_result_t
push_pcm_wait(audio_engine_v2_t *engine, uint32_t epoch, uint32_t first_rtp,
              uint32_t seq_tag, const uint32_t *flush_generation,
              const int16_t *pcm, size_t samples, uint8_t channels,
              uint32_t timeout_ms) {
  if (!engine || !engine->initialized) {
    return AUDIO_ENGINE_PUSH_FAILED;
  }

  const int64_t deadline_us =
      esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;

  while (audio_epoch_matches(&engine->epoch, epoch)) {
    /* A block of another RTP phase while nothing is scheduled cannot be a
     * gapless transition waiting for the outgoing audio to drain: there is
     * no outgoing audio.  A block or two of the wrong phase got in ahead of a
     * new stream (a seek's backlog frame, say) and fixed the phase, so they
     * go, or the new stream never gets in and the sender stalls behind it.
     * A whole buffered stream of the other phase means this block is the odd
     * one out, and it is dropped instead. */
    if (audio_timeline_phase_blocked(&engine->timeline, epoch, first_rtp) &&
        (engine->scheduler.state == AUDIO_SCHED_WAIT_ANCHOR ||
         engine->scheduler.state == AUDIO_SCHED_IDLE)) {
      const size_t held = audio_timeline_count(&engine->timeline);
      if (held > 8U) {
        ESP_LOGW(TAG,
                 "Dropping rtp=%" PRIu32
                 ": another phase than the %u blocks held",
                 first_rtp, (unsigned)held);
        return AUDIO_ENGINE_PUSH_FAILED;
      }
      ESP_LOGW(TAG, "Dropping %u blocks of another phase ahead of rtp=%" PRIu32,
               (unsigned)held, first_rtp);
      audio_timeline_clear(&engine->timeline);
    }
    if (flush_generation &&
        *flush_generation != audio_engine_v2_flush_generation(engine)) {
      return AUDIO_ENGINE_PUSH_FLUSHED;
    }
    if (!audio_timeline_phase_blocked(&engine->timeline, epoch, first_rtp) &&
        audio_timeline_free_slots(&engine->timeline) > 0U) {
      const audio_engine_push_result_t pushed =
          push_pcm(engine, epoch, first_rtp, seq_tag, flush_generation, pcm,
                   samples, channels);
      if (pushed != AUDIO_ENGINE_PUSH_FAILED ||
          !audio_epoch_matches(&engine->epoch, epoch)) {
        return pushed;
      }
      /* A free slot is not necessarily this block's: the ring is addressed
       * by RTP, so after a lost packet the free slot is the hole's, and this
       * block maps onto one still holding unplayed audio.  Returning here
       * dropped every packet until the hole came round -- 4.5 s of silence
       * in tests/playout_sim lost_packet -- so it waits like a full
       * timeline, and the sender is held back instead. */
    }

    const int64_t remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us <= 0) {
      return AUDIO_ENGINE_PUSH_FAILED;
    }

    /* Block on the consumer instead of polling.  The timeline gives this
     * semaphore whenever it retires a slot or is flushed, so the wake-up is
     * immediate and the decode task stays off the run queue while the sender
     * is being back-pressured.  The cap bounds the wait so teardown is never
     * blocked behind a paused scheduler holding a full timeline. */
    uint32_t wait_ms = (uint32_t)(remaining_us / 1000LL);
    if (wait_ms > AUDIO_V2_PUSH_WAIT_SLICE_MS) {
      wait_ms = AUDIO_V2_PUSH_WAIT_SLICE_MS;
    }
    (void)audio_timeline_wait_for_space(&engine->timeline, wait_ms + 1U);
  }
  return AUDIO_ENGINE_PUSH_FAILED;
}

bool audio_engine_v2_push_pcm_wait(audio_engine_v2_t *engine, uint32_t epoch,
                                   uint32_t first_rtp, const int16_t *pcm,
                                   size_t samples, uint8_t channels,
                                   uint32_t timeout_ms) {
  return push_pcm_wait(engine, epoch, first_rtp, 0U, NULL, pcm, samples,
                       channels, timeout_ms) == AUDIO_ENGINE_PUSH_OK;
}

audio_engine_push_result_t audio_engine_v2_push_pcm_seq_wait(
    audio_engine_v2_t *engine, uint32_t epoch, uint32_t first_rtp, uint32_t seq,
    uint32_t flush_generation, const int16_t *pcm, size_t samples,
    uint8_t channels, uint32_t timeout_ms) {
  return push_pcm_wait(engine, epoch, first_rtp,
                       AUDIO_TIMELINE_SEQ_VALID | (seq & 0xFFFFFFU),
                       &flush_generation, pcm, samples, channels, timeout_ms);
}

/* Caller holds control_mutex.  have_clock false: no conversion of the output
 * instant exists yet, so nothing may be scheduled against output_network_ns. */
static size_t render_locked(audio_engine_v2_t *engine,
                            int64_t output_network_ns, bool have_clock,
                            int16_t *out, size_t samples) {
  __atomic_add_fetch(&engine->render_calls, 1, __ATOMIC_RELAXED);
  if (emit_tail_locked(engine, out, samples)) {
    return samples;
  }
  if (engine->playing && !have_clock && engine->clock_map.valid) {
    /* Hold position: silence now, and the cursor stays where it is for when
     * the clock comes. */
    engine->scheduler.wait_reason = AUDIO_SCHED_WAIT_PTP_TO_RTP;
    memset(out, 0, samples * engine->format.channels * sizeof(int16_t));
    return samples;
  }
  if (!engine->playing) {
    memset(out, 0, samples * engine->format.channels * sizeof(int16_t));
    start_watchdog_check(engine, esp_timer_get_time());
    return samples;
  }

  const audio_scheduler_state_t state_before = engine->scheduler.state;
  const uint32_t cursor_before = engine->scheduler.cursor_rtp;
  size_t concealed = 0;
  size_t produced = audio_scheduler_render(
      &engine->scheduler, &engine->timeline, &engine->clock_map,
      output_network_ns, out, samples, (uint8_t)engine->format.channels,
      &concealed);
  if (engine->scheduler.state == AUDIO_SCHED_PLAYING) {
    engine->start_watchdog_armed = false;
  }

  if (state_before != AUDIO_SCHED_PLAYING &&
      engine->scheduler.state == AUDIO_SCHED_PLAYING) {
    ESP_LOGI(
        TAG,
        "start decision: epoch=%" PRIu32 " wanted=%" PRIu32 " start=%" PRIu32
        " delta=%" PRId32 " blocks=%u attempts=%" PRIu64 " fallback=%" PRIu64,
        engine->scheduler.epoch, engine->scheduler.wanted_rtp,
        engine->scheduler.cursor_rtp,
        (int32_t)(engine->scheduler.cursor_rtp - engine->scheduler.wanted_rtp),
        (unsigned)audio_timeline_count(&engine->timeline),
        engine->scheduler.start_attempts, engine->scheduler.fallback_attempts);
  }

  start_watchdog_check(engine, esp_timer_get_time());

  if (concealed > 0U) {
    /* Concealment is recorded, never logged from here.  This runs on the I2S
     * playback task: a formatted ESP_LOGW plus a timeline dump can block for
     * milliseconds on the UART mutex, and a burst of holes would then starve
     * the very task that has to drain them.  The 1 Hz status line below
     * reports the accumulated counters instead. */
    engine->conceal_events++;
    engine->last_conceal_gap_rtp = cursor_before;
  }
  engine->concealed_samples += concealed;

  int64_t now_us = esp_timer_get_time();
  if (engine->scheduler.state == AUDIO_SCHED_PLAYING &&
      now_us - engine->last_status_log_us >= 1000000LL) {
    engine->last_status_log_us = now_us;
    int32_t filtered_samples = engine->scheduler.playout_error_samples;
    int32_t raw_samples = engine->scheduler.raw_playout_error_samples;
    int32_t span_samples = engine->scheduler.raw_error_span_valid
                               ? engine->scheduler.raw_error_max_samples -
                                     engine->scheduler.raw_error_min_samples
                               : 0;
    engine->scheduler.raw_error_span_valid = false;
    int32_t filtered_us = 0;
    int32_t raw_us = 0;
    int32_t span_us = 0;
    if (engine->format.sample_rate > 0) {
      filtered_us = (int32_t)(((int64_t)filtered_samples * 1000000LL) /
                              engine->format.sample_rate);
      raw_us = (int32_t)(((int64_t)raw_samples * 1000000LL) /
                         engine->format.sample_rate);
      span_us = (int32_t)(((int64_t)span_samples * 1000000LL) /
                          engine->format.sample_rate);
    }
    uint32_t raw_abs_us =
        (uint32_t)(raw_us < 0 ? -(int64_t)raw_us : (int64_t)raw_us);
    uint32_t filtered_abs_us =
        (uint32_t)(filtered_us < 0 ? -(int64_t)filtered_us
                                   : (int64_t)filtered_us);
    const uint64_t new_conceals =
        engine->conceal_events - engine->conceal_events_logged;
    engine->conceal_events_logged = engine->conceal_events;
    const uint32_t new_trims =
        engine->scheduler.drift_servo_trims - engine->drift_servo_trims_logged;
    engine->drift_servo_trims_logged = engine->scheduler.drift_servo_trims;
    ESP_LOGI(TAG,
             "playout: raw=%s%" PRIu32 ".%03" PRIu32 " ms span=%" PRId32
             " us filt=%s%" PRIu32 ".%03" PRIu32 " ms (%" PRId32
             " smp) drift=%" PRId32 " ppm trims=%" PRIu32 "/s (%" PRIu32
             ") steps=%" PRIu32 " (anchor %" PRIu32 ") clock-holds=%" PRIu32
             " buffered=%u concealed=%" PRIu64 " holes=%" PRIu64 " (+%" PRIu64
             ")",
             raw_us < 0 ? "-" : "", raw_abs_us / 1000U, raw_abs_us % 1000U,
             span_us, filtered_us < 0 ? "-" : "", filtered_abs_us / 1000U,
             filtered_abs_us % 1000U, filtered_samples,
             engine->scheduler.estimated_drift_ppm, new_trims,
             engine->scheduler.drift_servo_trims, engine->scheduler.drift_steps,
             engine->scheduler.anchor_resets, engine->scheduler.servo_holds,
             (unsigned)audio_timeline_count(&engine->timeline),
             engine->concealed_samples, engine->conceal_events, new_conceals);
  }
  return produced;
}

size_t audio_engine_v2_render(audio_engine_v2_t *engine,
                              int64_t output_network_ns, int16_t *out,
                              size_t samples) {
  if (!engine || !engine->initialized || !out || samples == 0U) {
    return 0;
  }
  control_lock(engine);
  audio_scheduler_set_clock_offset(&engine->scheduler, false, 0);
  const size_t produced =
      render_locked(engine, output_network_ns, true, out, samples);
  control_unlock(engine);
  return produced;
}

size_t audio_engine_v2_render_local(audio_engine_v2_t *engine,
                                    int64_t output_local_ns,
                                    audio_playout_clock_t *clock, int16_t *out,
                                    size_t samples) {
  if (!engine || !engine->initialized || !out || samples == 0U || !clock) {
    return 0;
  }
  control_lock(engine);
  int64_t network_ns = 0;
  const bool have_clock =
      engine->clock_map.valid &&
      audio_playout_clock_convert(clock, &engine->clock_map, output_local_ns,
                                  &network_ns);
  /* Measured against the anchor, so a grandmaster change -- which moves the
   * network time and the anchor alike -- is no jump. */
  audio_scheduler_set_clock_offset(
      &engine->scheduler, have_clock,
      network_ns - (int64_t)engine->clock_map.anchor_network_ns -
          output_local_ns);
  const size_t produced =
      render_locked(engine, network_ns, have_clock, out, samples);
  control_unlock(engine);
  return produced;
}

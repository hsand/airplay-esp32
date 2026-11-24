// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "audio_clock_map.h"
#include "audio_epoch.h"
#include "audio_playout_clock.h"
#include "audio_receiver.h"
#include "audio_scheduler.h"
#include "audio_timeline.h"

typedef struct {
  audio_epoch_t epoch;
  audio_timeline_t timeline;
  audio_clock_map_t clock_map;
  audio_scheduler_t scheduler;
  audio_format_t format;
  bool initialized;
  bool playing;
  uint64_t blocks_inserted;
  uint64_t blocks_rejected;
  uint64_t concealed_samples;
  int64_t last_status_log_us;
  uint64_t conceal_events;
  /* Conceal activity since the last status log, so the render path never has
   * to format a message itself. */
  uint64_t conceal_events_logged;
  uint32_t drift_servo_trims_logged;
  uint32_t last_conceal_gap_rtp;

  /* Per-epoch ingress/decode diagnostics. Atomic increments are used because
   * buffered RX, decoder worker and audio output run on different tasks. */
  uint64_t diag_rx_packets;
  uint64_t diag_gate_drops;
  uint64_t diag_enqueue_ok;
  uint64_t diag_enqueue_retries;
  uint64_t diag_queue_drops;
  uint64_t diag_dequeued;
  uint64_t diag_epoch_drops;
  uint64_t diag_decode_ok;
  uint64_t diag_decode_fail;
  uint64_t diag_pcm_inserted;
  /* Every render call, paused or not: shows whether output asks at all. */
  uint64_t render_calls;

  /* Startup watchdog. Armed by an epoch's first valid anchor and cleared as
   * soon as the scheduler reaches PLAYING; until then it reports every
   * START_STALL_REPEAT_US. It is diagnostic only and never changes
   * scheduler, timeline, RTP gate, decoder, or I2S behavior. */
  bool start_watchdog_armed;
  int64_t start_watchdog_anchor_us;
  int64_t start_watchdog_report_us;
  uint32_t start_watchdog_epoch;
  uint32_t start_watchdog_anchor_rtp;

  /* Short publication gate shared by epoch changes and PCM insertion. */
  portMUX_TYPE publish_lock;

  /* Serialises every change to the clock map and the scheduler against the
   * playback task's render.  The RTSP task used to overwrite the clock map
   * while a render was half-way through it.  A mutex rather than a spinlock:
   * the render holds it across PSRAM copies, and priority inheritance lifts
   * a briefly holding RTSP task above the playback task's priority. */
  SemaphoreHandle_t control_mutex;

  /* Bumped (under publish_lock) by every FLUSHBUFFERED sweep, so a block
   * decoded against the flush queue as it stood before cannot be published
   * after the sweep that would have removed it. */
  uint32_t flush_generation;

  /* Buffered anchors, for the status line. */
  uint32_t anchors_installed;
  uint32_t anchors_consistent;

  /* The stop's fade-out (audio_scheduler_take_tail()), taken under
   * control_mutex as playout stops and handed out by the next renders ahead
   * of anything else, so the stream fades where it was cut.  In PSRAM, for
   * AUDIO_V2_TAIL_MAX_FRAMES frames. */
  int16_t *tail;
  uint32_t tail_frames;
  uint32_t tail_pos;
  uint32_t tails;
} audio_engine_v2_t;

/* The longest fade-out: AUDIO_SCHED_FADE_OUT_MS at 96 kHz. */
#define AUDIO_V2_TAIL_MAX_FRAMES (96U * AUDIO_SCHED_FADE_OUT_MS)

/* A SETRATEANCHORTIME rate=1 anchor, already in nanoseconds. */
typedef struct {
  uint32_t rtp;
  uint64_t network_ns;
  uint64_t timeline_id;
  int64_t playout_offset_ns;
  bool fas_valid;
  uint32_t fas_rtp;
} audio_engine_anchor_t;

typedef enum {
  AUDIO_ENGINE_ANCHOR_REJECTED = 0,
  /* A new timeline: installed, skew tracking reset. */
  AUDIO_ENGINE_ANCHOR_INSTALLED,
  /* Running, and the anchor agrees with the timeline being played: kept as
   * it is (Apple's hose ignores a rate-1 re-anchor while running). */
  AUDIO_ENGINE_ANCHOR_CONSISTENT,
} audio_engine_anchor_result_t;

typedef enum {
  AUDIO_ENGINE_PUSH_FAILED = 0,
  AUDIO_ENGINE_PUSH_OK,
  /* A FLUSHBUFFERED swept the timeline since the caller checked the flush
   * queue: check the packet against it again before retrying. */
  AUDIO_ENGINE_PUSH_FLUSHED,
} audio_engine_push_result_t;

esp_err_t audio_engine_v2_init(audio_engine_v2_t *engine,
                               const audio_format_t *format,
                               uint32_t frame_samples, uint16_t capacity);
void audio_engine_v2_deinit(audio_engine_v2_t *engine);
uint32_t audio_engine_v2_begin_epoch(audio_engine_v2_t *engine, int64_t now_us);
void audio_engine_v2_set_format(audio_engine_v2_t *engine,
                                const audio_format_t *format);
/* Switch codecs on the shared slot pool.  Discards anything held, so call it
 * at stream start, before PCM for the new stream arrives. */
bool audio_engine_v2_set_frame_samples(audio_engine_v2_t *engine,
                                       uint32_t frame_samples);
bool audio_engine_v2_set_anchor(audio_engine_v2_t *engine, uint32_t anchor_rtp,
                                uint64_t anchor_network_ns,
                                int64_t playout_offset_ns);
/* AirPlay 2 buffered SETRATEANCHORTIME rate=1.  A new anchor resets the skew
 * tracking; one that lands on a playing stream moves playout onto its
 * timeline at the first measurement, unless it agrees with the timeline being
 * played to within ANCHOR_CONSISTENT_US, when nothing moves at all. */
audio_engine_anchor_result_t
audio_engine_v2_set_rate_anchor(audio_engine_v2_t *engine,
                                const audio_engine_anchor_t *anchor);

/* Immediate FLUSHBUFFERED: stop output and forget the anchor until the next
 * SETRATEANCHORTIME.  The buffered PCM is left to the sequence sweep. */
void audio_engine_v2_stop(audio_engine_v2_t *engine);

/* Drop buffered blocks by sender sequence number; see
 * audio_timeline_discard_seq_range(). */
size_t audio_engine_v2_flush_seq_range(audio_engine_v2_t *engine, bool has_from,
                                       uint32_t from_seq, uint32_t until_seq);
uint32_t audio_engine_v2_flush_generation(const audio_engine_v2_t *engine);

/* The RTP position the playback cursor is on, playing or not: the reply to a
 * FLUSHBUFFERED.  False before anything was played in this epoch. */
bool audio_engine_v2_last_rendered_rtp(audio_engine_v2_t *engine,
                                       uint32_t *rtp);

// Invalidate the current clock map without discarding buffered PCM. Used when
// an anchor is known but the matching network clock has not locked yet.
void audio_engine_v2_wait_for_anchor(audio_engine_v2_t *engine, int64_t now_us);
void audio_engine_v2_set_playing(audio_engine_v2_t *engine, bool playing);
bool audio_engine_v2_is_nearly_full(audio_engine_v2_t *engine);
/* The RTP of the next frame to play, while the scheduler is playing.  Read
 * without a lock, so it can be a block out of date: the realtime receive task
 * only uses it to stop asking for packets that are already due. */
bool audio_engine_v2_play_cursor(const audio_engine_v2_t *engine,
                                 uint32_t *rtp);

bool audio_engine_v2_push_pcm(audio_engine_v2_t *engine, uint32_t epoch,
                              uint32_t first_rtp, const int16_t *pcm,
                              size_t samples, uint8_t channels);

/* Buffered AirPlay 2: like audio_engine_v2_push_pcm_wait(), with the
 * packet's 24-bit sequence number kept on the block for FLUSHBUFFERED, and
 * the flush generation the caller read before it last checked the flush
 * queue. */
audio_engine_push_result_t audio_engine_v2_push_pcm_seq_wait(
    audio_engine_v2_t *engine, uint32_t epoch, uint32_t first_rtp, uint32_t seq,
    uint32_t flush_generation, const int16_t *pcm, size_t samples,
    uint8_t channels, uint32_t timeout_ms);

/* Buffered streams must never discard a decoded packet merely because the PCM
 * timeline is temporarily full.  This variant blocks until the consumer frees
 * a slot, aborting immediately if the stream epoch changes.  timeout_ms is a
 * hard bound: an unbounded wait would leave the decode task unresponsive to
 * teardown while a paused scheduler holds the timeline full. */
bool audio_engine_v2_push_pcm_wait(audio_engine_v2_t *engine, uint32_t epoch,
                                   uint32_t first_rtp, const int16_t *pcm,
                                   size_t samples, uint8_t channels,
                                   uint32_t timeout_ms);

size_t audio_engine_v2_render(audio_engine_v2_t *engine,
                              int64_t output_network_ns, int16_t *out,
                              size_t samples);

/* Render for an output instant in LOCAL time: `clock` converts it into the
 * anchor's domain (re-basing the anchor across a grandmaster change, or
 * freewheeling without a lock), under the same lock as the render. */
size_t audio_engine_v2_render_local(audio_engine_v2_t *engine,
                                    int64_t output_local_ns,
                                    audio_playout_clock_t *clock, int16_t *out,
                                    size_t samples);

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "audio_clock_map.h"
#include "audio_timeline.h"

typedef enum {
  AUDIO_SCHED_IDLE = 0,
  AUDIO_SCHED_WAIT_ANCHOR,
  AUDIO_SCHED_PREROLL,
  AUDIO_SCHED_PLAYING,
  AUDIO_SCHED_PAUSED,
  AUDIO_SCHED_RECOVERING,
} audio_scheduler_state_t;

typedef enum {
  AUDIO_SCHED_WAIT_NONE = 0,
  AUDIO_SCHED_WAIT_PAUSED,
  AUDIO_SCHED_WAIT_CLOCK_MAP,
  AUDIO_SCHED_WAIT_PTP_TO_RTP,
  AUDIO_SCHED_WAIT_PREROLL,
  AUDIO_SCHED_WAIT_FALLBACK_DATA,
} audio_scheduler_wait_reason_t;

typedef struct {
  audio_scheduler_state_t state;
  uint32_t epoch;
  uint32_t cursor_rtp;
  uint32_t wanted_rtp;
  uint32_t preroll_samples;
  int64_t preroll_started_us;
  int64_t fallback_after_us;
  /* Raw block-boundary error is useful for debugging but contains the
   * expected callback/DMA phase sawtooth. filtered_playout_error_q16 tracks
   * the same error at the rendered block midpoint with a 1/8 IIR filter. */
  int32_t raw_playout_error_samples;
  /* Peak-to-peak span of the raw error since the last status log, i.e. the
   * measurement noise floor the servo thresholds have to clear. */
  int32_t raw_error_min_samples;
  int32_t raw_error_max_samples;
  bool raw_error_span_valid;
  int32_t playout_error_samples;
  int64_t filtered_playout_error_q16;
  int32_t max_abs_playout_error_samples;
  int32_t estimated_drift_ppm;
  int64_t drift_reference_error_q16;
  int64_t drift_reference_network_ns;
  uint64_t rendered_samples;
  bool error_filter_valid;
  /* Drift servo: closes the loop between the output crystal and the sender's
   * clock by trimming one sample at a time.  See audio_scheduler.c. */
  int64_t drift_servo_accum;
  uint32_t drift_servo_phase;
  uint32_t drift_servo_trims;
  uint32_t drift_servo_warmup;
  /* Steps: an error far beyond what the servo trims away in good time, as
   * from a stall, made up at once.  See audio_scheduler.c. */
  uint32_t drift_step_confirm;
  uint32_t drift_step_skip;
  uint32_t drift_step_hold;
  bool drift_step_fade_out;
  uint32_t drift_step_fade_in;  /* frames of the fade back in, 0 if none */
  uint32_t drift_step_faded_in; /* ...and how many of them have played */
  uint32_t drift_steps;
  /* How long the step condition has held, in rendered frames, and the span
   * of the clock offset over that window.  A step is only taken while the
   * clock held still: an error the clock made comes back out by itself. */
  uint32_t drift_step_confirm_frames;
  int64_t drift_step_offset_min_ns;
  int64_t drift_step_offset_max_ns;
  uint32_t drift_step_clock_deferrals;
  /* Network minus local time at the output instant of this render, as the
   * caller converted it, or invalid when it could not say. */
  bool clock_offset_valid;
  int64_t clock_offset_ns;
  /* The previous render's offset, and how many more frames the servo holds
   * still after the offset jumped. */
  bool last_clock_offset_valid;
  int64_t last_clock_offset_ns;
  uint32_t servo_hold_frames;
  uint32_t servo_holds;
  /* Set by a new anchor that moved the timeline under a playing stream: the
   * first measurement after it puts playout where the anchor says at once
   * (Apple re-sets its offset on the first measurement after an anchor). */
  bool anchor_reset_pending;
  uint32_t anchor_resets;
  /* First audible sample: nothing before it is heard, and output starts
   * exactly on it. */
  bool fas_valid;
  uint32_t fas_rtp;
  audio_scheduler_wait_reason_t wait_reason;
  uint64_t render_calls;
  uint64_t silent_render_calls;
  uint64_t start_attempts;
  uint64_t fallback_attempts;
} audio_scheduler_t;

void audio_scheduler_init(audio_scheduler_t *scheduler,
                          uint32_t preroll_samples, int64_t fallback_after_us);
void audio_scheduler_begin_epoch(audio_scheduler_t *scheduler, uint32_t epoch,
                                 int64_t now_us);
void audio_scheduler_set_paused(audio_scheduler_t *scheduler, bool paused);

/* A new anchor moved the timeline: forget the skew measured against the old
 * one.  `reposition` additionally makes the first measurement afterwards
 * step playout onto the new timeline (only meaningful while playing). */
void audio_scheduler_new_anchor(audio_scheduler_t *scheduler, bool reposition);

/* First audible sample, from SETRATEANCHORTIME; valid = false clears it. */
void audio_scheduler_set_first_audible(audio_scheduler_t *scheduler, bool valid,
                                       uint32_t rtp);

/* The clock offset used for the coming render; see clock_offset_ns. */
void audio_scheduler_set_clock_offset(audio_scheduler_t *scheduler, bool valid,
                                      int64_t offset_ns);

size_t audio_scheduler_render(audio_scheduler_t *scheduler,
                              audio_timeline_t *timeline,
                              const audio_clock_map_t *clock_map,
                              int64_t output_network_ns, int16_t *out,
                              size_t samples, uint8_t channels,
                              size_t *concealed_samples);

const char *audio_scheduler_state_name(audio_scheduler_state_t state);
const char *
audio_scheduler_wait_reason_name(audio_scheduler_wait_reason_t reason);

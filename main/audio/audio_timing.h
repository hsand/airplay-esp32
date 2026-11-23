// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "audio_receiver.h"

/* Stream timing state.
 *
 * This used to be a playout engine as well: audio_timing_read() pulled frames
 * off the sorted buffer, gated them on an early/late window, drained stale
 * ones and ran a position servo.  Playback now lives in audio_scheduler.c,
 * which computes position from the RTP-addressed timeline instead of
 * discovering it, so what is left here is the state the RTSP layer sets and
 * the engine reads: the sender's anchor, the latency figures advertised in
 * SETUP.  FLUSHBUFFERED lives in audio_flush_queue.c, keyed on sequence
 * numbers. */
typedef struct {
  uint32_t output_latency_us;
  bool playing;
  bool anchor_valid;
  uint64_t anchor_network_time_ns;
  uint32_t anchor_rtp_time;
  int64_t anchor_local_time_ns;
  bool ptp_locked;
  // Stream playout latency in samples, added to every frame's scheduled
  // play time: the sender's latency from the stream SETUP, audioLatencyMs if
  // present, else latencyMin.  Realtime streams default to 11025 (250 ms)
  // without one; buffered streams to 0, as Apple's receiver does.  The
  // platform half of Apple's (sender - platform) latency is already in the
  // output instant the playback task converts.  Set at stream SETUP;
  // survives audio_timing_reset() because it is stream configuration, not
  // playback state.
  uint32_t playout_latency_samples;
} audio_timing_t;

void audio_timing_init(audio_timing_t *timing);
void audio_timing_reset(audio_timing_t *timing);
void audio_timing_set_output_latency(audio_timing_t *timing,
                                     uint32_t latency_us);
uint32_t audio_timing_get_output_latency(const audio_timing_t *timing);
uint32_t audio_timing_get_hardware_latency(void);
// Total end-to-end latency (output target + HW DMA + fixed pipeline delay).
//
// DIAGNOSTIC ONLY — do NOT wire this into outputLatencyMicros.
// rtsp_handlers.c deliberately advertises 0 for both inputLatencyMicros and
// outputLatencyMicros because compute_early_us() already compensates for the
// hardware and pipeline delay internally; advertising a non-zero value makes
// the sender adjust its anchor as well and the delay is applied twice.
// shairport-sync likewise advertises no audioLatencies.
//
// Note also that this figure does not include the sender-driven pre-buffer
// actually sitting in the jitter buffer during playback (frequently 1.5 s+),
// so it is not the true end-to-end delay either. Use it for logging and
// introspection, not for protocol negotiation.
uint32_t audio_timing_get_advertised_latency(const audio_timing_t *timing);
// Set the stream playout latency (samples).  See playout_latency_samples.
void audio_timing_set_playout_latency(audio_timing_t *timing,
                                      uint32_t latency_samples);
void audio_timing_set_anchor(audio_timing_t *timing,
                             const audio_format_t *format, uint64_t clock_id,
                             uint64_t network_time_ns, uint32_t rtp_time);
void audio_timing_set_playing(audio_timing_t *timing, bool playing);

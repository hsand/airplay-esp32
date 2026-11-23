// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "audio_clock_map.h"

/* Local time -> the sender's network time, for the playback task.
 *
 * Every render converts the instant its first frame reaches the DAC into the
 * clock domain of the anchor, through the clock's offset+rate model
 * (ptp_clock_local_to_network()) rather than an offset sampled now and
 * applied ~50 ms into the future.  Two things this adds over a bare offset:
 *
 *  - A grandmaster change.  The clock reports which timeline each value is
 *    on.  When it differs from the anchor's, the anchor is re-expressed on
 *    the new timeline through local time: the last instant converted on the
 *    old timeline is converted again on the new one, and the anchor moves by
 *    the difference.  Playback carries on where it was (Apple re-anchors from
 *    its last skew-check point the same way) instead of losing lock and
 *    scheduling against an offset of 0.
 *  - No lock.  Rather than an offset of 0 -- which put wanted_rtp hours away
 *    and held the stream behind silence -- the last good conversion is
 *    carried forward on the local crystal (freewheel).  Before the first one
 *    there is nothing to carry, and the caller renders silence.
 *
 * Owned by the playback task: only it converts and reads this state. */

typedef enum {
  AUDIO_PLAYOUT_CLOCK_PTP = 0,
  AUDIO_PLAYOUT_CLOCK_NTP,
} audio_playout_clock_kind_t;

typedef struct {
  audio_playout_clock_kind_t kind;
  /* The latest conversion made with the clock locked, on `last_timeline`. */
  bool have_last;
  int64_t last_local_ns;
  int64_t last_network_ns;
  uint64_t last_timeline;
  bool freewheeling;
  /* Diagnostics. */
  uint32_t rebases;
  uint32_t rebase_failures;
  uint32_t freewheel_entries;
  int64_t last_rebase_delta_ns;
} audio_playout_clock_t;

void audio_playout_clock_reset(audio_playout_clock_t *clock,
                               audio_playout_clock_kind_t kind);

/* Convert `local_ns` onto the anchor timeline of `map`, re-expressing the
 * anchor on a new grandmaster's timeline when the clock has moved to one.
 * False when there is no usable conversion at all. */
bool audio_playout_clock_convert(audio_playout_clock_t *clock,
                                 audio_clock_map_t *map, int64_t local_ns,
                                 int64_t *network_ns);

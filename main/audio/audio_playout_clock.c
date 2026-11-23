// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "audio_playout_clock.h"

#include <string.h>

#include "ntp_clock.h"
#include "ptp_clock.h"

void audio_playout_clock_reset(audio_playout_clock_t *clock,
                               audio_playout_clock_kind_t kind) {
  if (!clock) {
    return;
  }
  memset(clock, 0, sizeof(*clock));
  clock->kind = kind;
}

/* The clock's own conversion, with the timeline it is on (0 for NTP). */
static bool clock_now(const audio_playout_clock_t *clock, int64_t local_ns,
                      int64_t *network_ns, uint64_t *timeline) {
  if (clock->kind == AUDIO_PLAYOUT_CLOCK_NTP) {
    if (!ntp_clock_is_locked()) {
      return false;
    }
    *network_ns = local_ns + ntp_clock_get_offset_ns();
    *timeline = 0;
    return true;
  }
  uint64_t network = 0;
  uint64_t gm = 0;
  if (!ptp_clock_local_to_network(local_ns, &network, &gm) ||
      network > (uint64_t)INT64_MAX) {
    return false;
  }
  *network_ns = (int64_t)network;
  *timeline = gm;
  return true;
}

bool audio_playout_clock_convert(audio_playout_clock_t *clock,
                                 audio_clock_map_t *map, int64_t local_ns,
                                 int64_t *network_ns) {
  if (!clock || !map || !network_ns) {
    return false;
  }

  int64_t network = 0;
  uint64_t timeline = 0;
  if (clock_now(clock, local_ns, &network, &timeline)) {
    /* 0 on either side means "not known", never "a different one". */
    if (map->valid && map->timeline_id != 0 && timeline != 0 &&
        timeline != map->timeline_id) {
      int64_t again = 0;
      uint64_t again_timeline = 0;
      if (clock->have_last && clock->last_timeline == map->timeline_id &&
          clock_now(clock, clock->last_local_ns, &again, &again_timeline) &&
          again_timeline == timeline) {
        /* The same local instant on both timelines: the anchor moves by the
         * distance between them, and the RTP it names stays where it was in
         * real time. */
        const int64_t delta = again - clock->last_network_ns;
        map->anchor_network_ns =
            (uint64_t)((int64_t)map->anchor_network_ns + delta);
        map->timeline_id = timeline;
        clock->last_rebase_delta_ns = delta;
        clock->rebases++;
      } else {
        /* Nothing was played on the old timeline yet, so there is no instant
         * to carry across.  The anchor waits for its own timeline or a new
         * anchor, and so does the output. */
        clock->rebase_failures++;
        if (!clock->have_last || clock->last_timeline != map->timeline_id) {
          return false;
        }
        *network_ns =
            clock->last_network_ns + (local_ns - clock->last_local_ns);
        return true;
      }
    }
    clock->have_last = true;
    clock->last_local_ns = local_ns;
    clock->last_network_ns = network;
    clock->last_timeline = timeline;
    clock->freewheeling = false;
    *network_ns = network;
    return true;
  }

  /* No lock.  Carry the last conversion forward on the local crystal, which
   * also drives the DAC: what drift there is accrues at the crystal's ppm,
   * a millisecond in tens of seconds, and the servo follows the clock back
   * when it returns. */
  if (clock->have_last && (map->timeline_id == 0 || clock->last_timeline == 0 ||
                           clock->last_timeline == map->timeline_id)) {
    if (!clock->freewheeling) {
      clock->freewheeling = true;
      clock->freewheel_entries++;
    }
    *network_ns = clock->last_network_ns + (local_ns - clock->last_local_ns);
    return true;
  }
  return false;
}

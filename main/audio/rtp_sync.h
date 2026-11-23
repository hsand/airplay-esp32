// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* The realtime stream's sync packets (0x54 NTP, 0x57 PTP), turned into one
 * RTP <-> network-time map per play segment, the way Apple's receiver does.
 *
 * A sync packet says which RTP frame plays at which network time.  v0.3.0
 * made each one the new map, so every packet's timestamp jitter, about once
 * a second, went straight into the scheduler's error.  Here the first packet
 * of a segment fixes the anchor, and later ones are only measurements of
 * where that anchor has drifted to, in samples:
 *
 *  - an error under 16 samples is ignored;
 *  - a larger one moves the map 1/16 of the way (a running average);
 *  - one over 4410 samples, or a packet with the "first sync after flush"
 *    bit, starts a new anchor, which takes effect only once playback reaches
 *    the packet's "apply" RTP, so audio from before the change still plays
 *    on the old map.
 *
 * Skew correction stays with the scheduler's drift servo; this only hands it
 * a map that does not move without cause.
 *
 * Bookkeeping only, no clock or task of its own; the control task owns it. */

// Errors below this many samples do not move the map.
#define RTP_SYNC_DEAD_BAND 16
// The running average moves 1/2^SHIFT of the error per packet.
#define RTP_SYNC_SHIFT 4
// An error above this many samples (100 ms at 44.1 kHz) re-anchors.
#define RTP_SYNC_RESET_SAMPLES 4410
// A deferred anchor is applied at once when its apply point is more than
// this many samples (10 s at 44.1 kHz) ahead of the playback position.
#define RTP_SYNC_FORCE_APPLY_SAMPLES 441000
// Byte 0 of a sync packet: first sync after a flush.
#define RTP_SYNC_FLAG_FIRST_AFTER_FLUSH 0x10

typedef struct {
  uint32_t rtp;
  uint64_t network_ns;
} rtp_sync_anchor_t;

typedef enum {
  RTP_SYNC_KEEP,    // the map stands
  RTP_SYNC_PUBLISH, // publish the anchor written to *out now
  RTP_SYNC_DEFER,   // a new map waits for rtp_sync_poll()
} rtp_sync_action_t;

typedef struct {
  bool valid;
  uint32_t sample_rate;
  uint64_t timeline; // clock ID the map is on (0 for NTP)
  uint32_t anchor_rtp;
  uint64_t anchor_network_ns;
  // The smoothed offset of the map from the anchor, in samples.
  int32_t average;
  // The newest packet's RTP: anchors are re-expressed there when published.
  uint32_t last_rtp;
  // A new map waiting for playback to reach apply_rtp.
  bool pending;
  uint32_t apply_rtp;

  // For the status line.
  uint32_t announces;
  uint32_t nudges;    // the average moved
  uint32_t resets;    // re-anchored on a flush bit
  uint32_t jumps;     // re-anchored on an error over RTP_SYNC_RESET_SAMPLES
  uint32_t applied;   // deferred maps applied
  uint32_t forced;    // of those, applied early
  int32_t last_error; // the newest packet against the map, samples
  int32_t max_error;  // largest |error| that did not re-anchor
} rtp_sync_t;

void rtp_sync_reset(rtp_sync_t *s);

/* One sync packet: `rtp` plays at `network_ns` on timeline `timeline`.
 * flags is byte 0 of the packet, apply_rtp its bytes 16-19.  On
 * RTP_SYNC_PUBLISH, *out is the map to hand to the engine. */
rtp_sync_action_t rtp_sync_announce(rtp_sync_t *s, uint32_t sample_rate,
                                    uint64_t timeline, uint8_t flags,
                                    uint32_t rtp, uint64_t network_ns,
                                    uint32_t apply_rtp, rtp_sync_anchor_t *out);

/* While a new map is pending: true, with the map in *out, once it is due --
 * playback is past apply_rtp (or not running at all, or so far behind that
 * waiting makes no sense). */
bool rtp_sync_poll(rtp_sync_t *s, bool cursor_valid, uint32_t cursor_rtp,
                   rtp_sync_anchor_t *out);

/* The current map expressed at `rtp`: the network time `rtp` plays at.
 * False before the first packet. */
bool rtp_sync_current(const rtp_sync_t *s, uint32_t rtp,
                      rtp_sync_anchor_t *out);

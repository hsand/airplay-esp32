// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * gPTP (IEEE 1588 / 802.1AS) slave for AirPlay 2 time synchronization,
 * modelled on Apple's own receiver:
 *
 *  - Two-way end-to-end measurement: a unicast Delay_Req answers every Sync
 *    from the chosen master port, and the (t1+t4)/2 vs (t2+t3)/2 midpoints
 *    cancel the symmetric part of the path delay.
 *  - A lag-compensated two-stage IIR per timeline gives offset *and* rate,
 *    so local <-> network conversions are anchor + rate, not offset only.
 *  - Masters that never answer Delay_Req, or stop answering for 10 s, are
 *    followed passively, from the least delayed one-way samples, with a
 *    retry of two-way every 10 s.
 *  - The grandmaster is tracked from Announce and the Follow_Up TLVs; every
 *    network time carries the GM it is on, and a GM change resets the clock.
 *
 * The board never becomes master and never sends Announce.
 */

/**
 * Initialize and start PTP clock synchronization.
 * Creates a task that listens for PTP messages on UDP 319/320.
 */
esp_err_t ptp_clock_init(void);

/**
 * Stop PTP clock and free resources.
 */
void ptp_clock_stop(void);

/**
 * Clear PTP clock synchronization state.
 * Resets offset, rate, lock, master filter and peer list without stopping
 * the clock.  Used when a new session cannot safely reuse the previous
 * clock state.
 */
void ptp_clock_clear(void);

/**
 * Called at TEARDOWN.  If the master is still sending (an accepted sample
 * within the last 2 s), keep the model, lock and master filter, so the next
 * session from the same master starts without re-locking;
 * otherwise behave like ptp_clock_clear().
 */
void ptp_clock_end_session(void);

/**
 * Check if PTP is locked to a master clock.
 * @return true if synchronized with acceptable accuracy
 */
bool ptp_clock_is_locked(void);

/**
 * Get current PTP time in nanoseconds: local time through the offset + rate
 * model.
 */
uint64_t ptp_clock_get_time_ns(void);

/**
 * Get current offset from local clock to PTP time in nanoseconds, evaluated
 * at "now" from the offset + rate model.  PTP_time = local_time + offset.
 */
int64_t ptp_clock_get_offset_ns(void);

/**
 * Notify the PTP clock that playback is resuming after a pause.
 *
 * The clock keeps measuring through a pause, so this only matters when the
 * master went quiet meanwhile (a phone that slept): if the pause lasted at
 * least 30 s and the samples are not current, the next sample re-seeds the
 * filter instead of being gated against a stale model.
 *
 * @param pause_duration_ms  Wall-clock length of the pause in milliseconds.
 */
void ptp_clock_notify_resume(uint32_t pause_duration_ms);

/**
 * Get synchronization statistics.
 */
typedef struct {
  uint32_t sync_count;        // SYNCs received from the selected port
  uint32_t followup_count;    // FOLLOW_UPs matched to them
  int64_t last_offset_ns;     // Last RAW measured offset (pre-filter)
  int64_t filtered_offset_ns; // Model offset at "now" (what timing uses)
  uint32_t lock_time_ms;      // Time since lock achieved (0 if not locked)
  uint32_t outlier_count;     // Samples dropped by any gate since start
  uint32_t delay_req_sent;
  uint32_t delay_resp_received;
  bool two_way; // the clock is on two-way samples (else passive)
  int32_t rate_ppb;
  uint64_t grandmaster_id;
} ptp_stats_t;

void ptp_clock_get_stats(ptp_stats_t *stats);

/**
 * Name the timeline the AirPlay anchor is on: the grandmaster clockIdentity
 * carried in the 0xD7 anchor packet.  Pass 0 to clear the filter (choose the
 * grandmaster by BMCA from Announce -- the default at startup).
 *
 * A port is on GM `id` when its Announce or Apple Follow_Up TLV names it, or
 * when its own clockIdentity is `id` (the GM's direct port, preferred).
 *
 * When the GM changes, the model and lock are reset so a stale offset to a
 * previous master is not carried over -- unless the clock is already on
 * that GM and its latest sample, accepted, is under 2 s old.
 */
void ptp_clock_set_master_clock_id(uint64_t clock_id);

/**
 * Read the current expected master clock_id (0 if none / filter cleared).
 */
uint64_t ptp_clock_get_master_clock_id(void);

/**
 * Peers from SETUP timingPeerInfo/timingPeerList and SETPEERS/SETPEERSX.
 * IPv4 only; replaces the whole list.  While the list is non-empty, Sync
 * from any other address is ignored.  count == 0 accepts every source.
 */
void ptp_clock_set_peers(const uint32_t *ipv4_addrs_net_order, size_t count);

/**
 * Local (esp_timer ns) -> network (PTP ns) using the offset + rate model.
 * Returns false when not locked.  *gm_id (may be NULL) = the grandmaster /
 * timeline the value is on.
 */
bool ptp_clock_local_to_network(int64_t local_ns, uint64_t *network_ns,
                                uint64_t *gm_id);

/**
 * Network -> local.  Returns false when not locked or when gm_id is not the
 * current grandmaster (the timeline changed): that is how the audio side
 * detects a GM change and re-derives its anchor.
 */
bool ptp_clock_network_to_local(uint64_t network_ns, uint64_t gm_id,
                                int64_t *local_ns);

/**
 * The grandmaster the clock is on; 0 if none.
 */
uint64_t ptp_clock_get_grandmaster_id(void);

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Missing-packet bookkeeping for the realtime (UDP) stream, timed the way
 * Apple's receiver times its retransmit requests.
 *
 * Every sequence number skipped among the last RTP_RESEND_WINDOW packets is
 * remembered until it turns up, retransmitted or late, until its audio is
 * due to play, or until the sender says it no longer has it.  The window is
 * longer than any realtime buffer: 512 packets of 352 frames is 4.1 s at
 * 44.1 kHz, where a sender runs about 2 s ahead.
 *
 * Requests go out only when a packet arrives, at most
 * RTP_RESEND_REQUESTS_PER_PACKET of them, so asking can never crowd out
 * reading.  A hole is asked for again every retransmit timeout, which follows
 * the measured round trip of retransmissions (TCP's 2 x srtt + 4 x rttvar,
 * Karn's rule, capped at 100 ms), for as long as its audio could still play.
 * A gap of more than RTP_RESEND_BURST packets is not asked for at all, and
 * drops everything still outstanding: a sender that lost that much is not
 * going to fill it in time.
 *
 * Bookkeeping only, with no clock or socket of its own.  The receive task
 * owns the tracker and sends the requests. */

#define RTP_RESEND_WINDOW 512U
// Requests sent for one arriving packet, at most.
#define RTP_RESEND_REQUESTS_PER_PACKET 3U
// A gap longer than this is a burst: nothing asked, everything dropped.
#define RTP_RESEND_BURST 128U
// The retransmit timeout before any round trip is known, and its ceiling.
#define RTP_RESEND_RTO_INITIAL_US 100000U
#define RTP_RESEND_RTO_MAX_US     100000U

typedef struct {
  uint16_t first;
  uint16_t count;
} rtp_resend_range_t;

typedef enum {
  RTP_RESEND_NEWEST,   // Newer than any packet before it; may open holes
  RTP_RESEND_FILLED,   // Filled a hole, retransmitted or out of order
  RTP_RESEND_UNWANTED, // A duplicate, or older than any hole still open
} rtp_resend_verdict_t;

// Running totals for the receive task's status line, which may clear them.
typedef struct {
  uint32_t received; // packets newer than any before them
  uint32_t skipped;  // sequence numbers that went missing
  uint32_t resent;   // holes filled by a retransmission
  uint32_t late;     // holes filled by the original packet, out of order
  uint32_t expired;  // holes given up on: due to play, or out of the window
  uint32_t unwanted; // duplicates, and packets nobody was waiting for
  uint32_t asked;    // packets asked for, each repeat counted again
  uint32_t requests; // requests made, one per run of packets
  uint32_t bursts;   // gaps over RTP_RESEND_BURST, not asked for
  uint32_t aborted;  // holes dropped by a burst
  uint32_t refused;  // holes the sender said it no longer has
} rtp_resend_counts_t;

typedef struct {
  // Indexed by sequence number modulo the window.
  uint32_t missing[RTP_RESEND_WINDOW / 32U];
  // When each hole was last asked for, in RTP_RESEND_TICK_US units, and how
  // many times.  0 asks: never asked, due at once.
  uint16_t asked_tick[RTP_RESEND_WINDOW];
  uint8_t asks[RTP_RESEND_WINDOW];
  bool valid;
  uint16_t newest_seq;
  uint32_t newest_rtp;
  // RTP frames per packet, taken from consecutive packets.
  uint32_t rtp_step;
  // Round trip of retransmissions, microseconds.
  int32_t srtt_us;
  int32_t rttvar_us;
  int32_t rto_us;
  int32_t rtt_min_us;
  int32_t rtt_max_us;
  int32_t rtt_last_us;
  rtp_resend_counts_t counts;
} rtp_resend_t;

// The resolution of the request times: a uint16_t of these covers 8.4 s.
#define RTP_RESEND_TICK_US 128U

// Forgets everything.  rtp_step is the packet length to assume until two
// consecutive packets show the real one.
void rtp_resend_reset(rtp_resend_t *r, uint32_t rtp_step);

/* Records a data packet or a retransmission, arriving at now_us, and says
 * whether to use it.  A retransmission of a hole asked for once gives a
 * round-trip sample. */
rtp_resend_verdict_t rtp_resend_on_packet(rtp_resend_t *r, uint16_t seq,
                                          uint32_t rtp, bool retransmit,
                                          int64_t now_us);

/* Gives up on the holes whose audio starts before cursor_rtp, when
 * cursor_valid, then writes up to max_ranges runs of holes that are due to be
 * asked for, oldest first: never asked for, or last asked at least one
 * retransmit timeout before now_us.  Holes written count as asked at now_us.
 * Call it when a packet arrives, with max_ranges at most
 * RTP_RESEND_REQUESTS_PER_PACKET; 0 only expires. */
size_t rtp_resend_collect(rtp_resend_t *r, int64_t now_us, bool cursor_valid,
                          uint32_t cursor_rtp, rtp_resend_range_t *ranges,
                          size_t max_ranges);

/* The sender's 8-byte answer that it cannot resend: every hole up to and
 * including `seq` is dropped.  Returns how many. */
uint32_t rtp_resend_refused(rtp_resend_t *r, uint16_t seq);

// Holes still open.
uint32_t rtp_resend_outstanding(const rtp_resend_t *r);

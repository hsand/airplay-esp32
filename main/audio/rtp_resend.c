// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtp_resend.h"

#include <limits.h>
#include <string.h>

#define WINDOW_MASK (RTP_RESEND_WINDOW - 1U)
#define WORDS       (RTP_RESEND_WINDOW / 32U)
// The longest packet believed when learning rtp_step.
#define MAX_RTP_STEP 8192U

_Static_assert((RTP_RESEND_WINDOW & WINDOW_MASK) == 0U,
               "RTP_RESEND_WINDOW must be a power of two");
_Static_assert(RTP_RESEND_WINDOW <= 32768U,
               "the window must be shorter than half the sequence space");
_Static_assert(RTP_RESEND_BURST < RTP_RESEND_WINDOW,
               "a burst must fit the window");

static inline uint32_t slot(uint16_t seq) {
  return seq & WINDOW_MASK;
}

static inline bool bit_get(const uint32_t *bits, uint16_t seq) {
  const uint32_t i = slot(seq);
  return ((bits[i >> 5] >> (i & 31U)) & 1U) != 0U;
}

static inline void bit_set(uint32_t *bits, uint16_t seq) {
  const uint32_t i = slot(seq);
  bits[i >> 5] |= 1U << (i & 31U);
}

static inline void bit_clear(uint32_t *bits, uint16_t seq) {
  const uint32_t i = slot(seq);
  bits[i >> 5] &= ~(1U << (i & 31U));
}

static inline uint16_t tick_of(int64_t now_us) {
  return (uint16_t)((uint64_t)now_us / RTP_RESEND_TICK_US);
}

static bool any_missing(const rtp_resend_t *r) {
  for (size_t i = 0; i < WORDS; i++) {
    if (r->missing[i] != 0U) {
      return true;
    }
  }
  return false;
}

void rtp_resend_reset(rtp_resend_t *r, uint32_t rtp_step) {
  memset(r, 0, sizeof(*r));
  r->rtp_step = rtp_step;
  r->srtt_us = (int32_t)RTP_RESEND_RTO_INITIAL_US;
  r->rttvar_us = 0;
  r->rto_us = (int32_t)RTP_RESEND_RTO_INITIAL_US;
  r->rtt_min_us = INT32_MAX;
  r->rtt_max_us = INT32_MIN;
}

uint32_t rtp_resend_outstanding(const rtp_resend_t *r) {
  uint32_t open = 0;
  for (size_t i = 0; i < WORDS; i++) {
    open += (uint32_t)__builtin_popcount(r->missing[i]);
  }
  return open;
}

/* Apple's estimator: a sample that sets a new minimum or maximum only moves
 * that bound; the others feed TCP's smoothed round trip and variance (gains
 * 1/8 and 1/4), and the timeout is 2 x srtt + 4 x rttvar, at most 100 ms. */
static void rtt_sample(rtp_resend_t *r, int32_t rtt) {
  r->rtt_last_us = rtt;
  if (rtt < r->rtt_min_us || rtt > r->rtt_max_us) {
    if (rtt < r->rtt_min_us) {
      r->rtt_min_us = rtt;
    }
    if (rtt > r->rtt_max_us) {
      r->rtt_max_us = rtt;
    }
    return;
  }
  const int32_t err = rtt - r->srtt_us;
  r->srtt_us += err / 8;
  r->rttvar_us += ((err < 0 ? -err : err) - r->rttvar_us) / 4;
  int32_t rto = 2 * r->srtt_us + 4 * r->rttvar_us;
  if (rto > (int32_t)RTP_RESEND_RTO_MAX_US) {
    rto = (int32_t)RTP_RESEND_RTO_MAX_US;
  }
  r->rto_us = rto;
}

// Drops every hole: the sender is not going to fill them in time.
static void abort_all(rtp_resend_t *r) {
  r->counts.aborted += rtp_resend_outstanding(r);
  memset(r->missing, 0, sizeof(r->missing));
}

rtp_resend_verdict_t rtp_resend_on_packet(rtp_resend_t *r, uint16_t seq,
                                          uint32_t rtp, bool retransmit,
                                          int64_t now_us) {
  if (!r->valid) {
    // Nothing has been asked for yet, so no retransmission can be wanted.
    if (retransmit) {
      r->counts.unwanted++;
      return RTP_RESEND_UNWANTED;
    }
    r->valid = true;
    r->newest_seq = seq;
    r->newest_rtp = rtp;
    r->counts.received++;
    return RTP_RESEND_NEWEST;
  }

  const int32_t delta = (int16_t)(uint16_t)(seq - r->newest_seq);
  if (delta <= 0 || retransmit) {
    // Anything but the newest packet is wanted only while its hole is open.
    if (delta < 0 && (uint32_t)-delta < RTP_RESEND_WINDOW &&
        bit_get(r->missing, seq)) {
      bit_clear(r->missing, seq);
      const uint32_t i = slot(seq);
      if (retransmit) {
        r->counts.resent++;
        // Karn: only an answer to a single request times the round trip.
        if (r->asks[i] == 1U) {
          const uint16_t elapsed =
              (uint16_t)(tick_of(now_us) - r->asked_tick[i]);
          rtt_sample(r, (int32_t)elapsed * (int32_t)RTP_RESEND_TICK_US);
        }
      } else {
        r->counts.late++;
      }
      return RTP_RESEND_FILLED;
    }
    r->counts.unwanted++;
    return RTP_RESEND_UNWANTED;
  }

  const uint32_t step = rtp - r->newest_rtp;
  if (delta == 1 && step > 0U && step <= MAX_RTP_STEP) {
    r->rtp_step = step;
  }

  const uint32_t skipped = (uint32_t)delta - 1U;
  r->counts.skipped += skipped;
  if (skipped > RTP_RESEND_BURST) {
    // Burst loss: ask for none of it, and drop what was still awaited.
    r->counts.bursts++;
    r->counts.aborted += skipped;
    abort_all(r);
  } else {
    // Each slot the window moves onto last held the packet one window
    // earlier, and a hole still open there has had its chance.
    for (uint32_t i = 1; i <= (uint32_t)delta; i++) {
      const uint16_t s = (uint16_t)(r->newest_seq + i);
      if (bit_get(r->missing, s)) {
        r->counts.expired++;
      }
      if (i < (uint32_t)delta) {
        bit_set(r->missing, s);
        r->asks[slot(s)] = 0;
      } else {
        bit_clear(r->missing, s);
      }
    }
  }

  r->newest_seq = seq;
  r->newest_rtp = rtp;
  r->counts.received++;
  return RTP_RESEND_NEWEST;
}

size_t rtp_resend_collect(rtp_resend_t *r, int64_t now_us, bool cursor_valid,
                          uint32_t cursor_rtp, rtp_resend_range_t *ranges,
                          size_t max_ranges) {
  if (!r->valid || !any_missing(r)) {
    return 0;
  }

  const uint16_t tick = tick_of(now_us);
  const uint32_t rto_ticks =
      ((uint32_t)r->rto_us + RTP_RESEND_TICK_US - 1U) / RTP_RESEND_TICK_US;
  size_t n = 0;
  bool extending = false;

  // Oldest first, from a window behind the newest packet up to it.
  for (uint32_t back = RTP_RESEND_WINDOW - 1U; back > 0U; back--) {
    const uint16_t seq = (uint16_t)(r->newest_seq - back);
    const uint32_t i = slot(seq);
    if (r->missing[i >> 5] == 0U) {
      // A whole empty word: skip to its last bit.
      const uint32_t skip = 31U - (i & 31U);
      back = back > skip ? back - skip : 1U;
      extending = false;
      continue;
    }
    if (!bit_get(r->missing, seq)) {
      extending = false;
      continue;
    }

    // A packet that would arrive after its audio has played is no use.
    if (cursor_valid) {
      const uint32_t rtp = r->newest_rtp - back * r->rtp_step;
      if ((int32_t)(rtp - cursor_rtp) < 0) {
        bit_clear(r->missing, seq);
        r->counts.expired++;
        extending = false;
        continue;
      }
    }

    if (r->asks[i] != 0U && (uint16_t)(tick - r->asked_tick[i]) < rto_ticks) {
      extending = false;
      continue;
    }

    if (extending) {
      ranges[n - 1].count++;
    } else if (n < max_ranges) {
      ranges[n].first = seq;
      ranges[n].count = 1;
      n++;
      extending = true;
    } else {
      continue;
    }
    r->asked_tick[i] = tick;
    if (r->asks[i] < UINT8_MAX) {
      r->asks[i]++;
    }
    r->counts.asked++;
  }

  r->counts.requests += (uint32_t)n;
  return n;
}

uint32_t rtp_resend_refused(rtp_resend_t *r, uint16_t seq) {
  if (!r->valid) {
    return 0;
  }
  uint32_t dropped = 0;
  for (uint32_t back = RTP_RESEND_WINDOW - 1U; back > 0U; back--) {
    const uint16_t s = (uint16_t)(r->newest_seq - back);
    if ((int16_t)(uint16_t)(s - seq) > 0) {
      break; // past seq; the rest are newer still
    }
    if (bit_get(r->missing, s)) {
      bit_clear(r->missing, s);
      dropped++;
    }
  }
  r->counts.refused += dropped;
  return dropped;
}

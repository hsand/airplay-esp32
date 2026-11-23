// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for main/audio/rtp_resend.c, the realtime stream's record of
 * missing packets: which holes are asked for and when, how the retry
 * interval follows the round trip, when holes are given up on, and how much
 * of a lossy stream comes back compared with the v0.3.0 tracker (baseline/,
 * built beside it by v030.c). */

#include "rtp_resend.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v030.h"

#define STEP     352U        /* frames in an ALAC packet */
#define BASE_RTP 0xfff00000U /* so the RTP wraps during the longer runs */
#define MAX_REQ  RTP_RESEND_REQUESTS_PER_PACKET
#define MS       1000LL

static int failures;

#define CHECK(cond, ...)                            \
  do {                                              \
    if (!(cond)) {                                  \
      failures++;                                   \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                          \
      printf("\n");                                 \
    }                                               \
  } while (0)

static rtp_resend_t r;
static rtp_resend_range_t ranges[MAX_REQ];
static int64_t now_us = 1000 * MS;

static uint32_t rtp_at(uint32_t i) {
  return BASE_RTP + i * STEP;
}

static rtp_resend_verdict_t feed(uint32_t i) {
  return rtp_resend_on_packet(&r, (uint16_t)i, rtp_at(i), false, now_us);
}

static rtp_resend_verdict_t resend(uint32_t i) {
  return rtp_resend_on_packet(&r, (uint16_t)i, rtp_at(i), true, now_us);
}

// Packets first..last in order, without those in [hole, hole + holes).
static void feed_around(uint32_t first, uint32_t last, uint32_t hole,
                        uint32_t holes) {
  for (uint32_t i = first; i <= last; i++) {
    if (i < hole || i >= hole + holes) {
      (void)feed(i);
    }
  }
}

// What the receive task does when a packet has arrived.
static size_t ask(void) {
  return rtp_resend_collect(&r, now_us, false, 0, ranges, MAX_REQ);
}

// As ask(), with playback at packet `playing`.
static size_t ask_playing(uint32_t playing) {
  return rtp_resend_collect(&r, now_us, true, rtp_at(playing), ranges, MAX_REQ);
}

// Every hole is filled, given up on, or still open.
static bool balanced(void) {
  const rtp_resend_counts_t *c = &r.counts;
  return c->skipped == c->resent + c->late + c->expired + c->aborted +
                           c->refused + rtp_resend_outstanding(&r);
}

static void test_in_order(void) {
  printf("in order: nothing asked for\n");
  rtp_resend_reset(&r, STEP);
  for (uint32_t i = 0; i < 1000; i++) {
    CHECK(feed(i) == RTP_RESEND_NEWEST, "packet %u not the newest", i);
    CHECK(ask() == 0, "asked for something at packet %u", i);
  }
  CHECK(r.counts.received == 1000 && r.counts.skipped == 0,
        "received %u, skipped %u", r.counts.received, r.counts.skipped);
}

static void test_one_hole(void) {
  printf("one hole: asked at once, again every 100 ms, filled once\n");
  rtp_resend_reset(&r, STEP);
  feed_around(0, 51, 50, 1);
  size_t n = ask();
  CHECK(n == 1 && ranges[0].first == 50 && ranges[0].count == 1,
        "first request: %zu ranges, from %u", n, n ? ranges[0].first : 0);
  now_us += 99 * MS;
  CHECK(ask() == 0, "asked again within the retransmit timeout");
  // 100 ms, rounded up to the 128 us the request times are kept in.
  now_us += 1 * MS + RTP_RESEND_TICK_US;
  n = ask();
  CHECK(n == 1 && ranges[0].first == 50, "not asked again after 100 ms");
  CHECK(r.asks[50] == 2, "asks %u", r.asks[50]);
  CHECK(resend(50) == RTP_RESEND_FILLED, "retransmission not taken");
  CHECK(resend(50) == RTP_RESEND_UNWANTED, "second copy taken");
  CHECK(feed(51) == RTP_RESEND_UNWANTED, "duplicate taken");
  CHECK(ask() == 0, "asked for a filled hole");
  // Karn: an answer to the second request says nothing about the round trip.
  CHECK(r.rtt_last_us == 0 && r.rto_us == 100000, "sample taken: rto %d",
        r.rto_us);
  CHECK(r.counts.resent == 1 && r.counts.unwanted == 2 && r.counts.asked == 2 &&
            r.counts.requests == 2 && balanced(),
        "resent %u unwanted %u asked %u requests %u", r.counts.resent,
        r.counts.unwanted, r.counts.asked, r.counts.requests);
}

static void test_no_limit_while_useful(void) {
  printf("an unanswered hole: asked every timeout until it is due to play\n");
  rtp_resend_reset(&r, STEP);
  feed_around(0, 10, 5, 1);
  uint32_t asks = 0;
  // 1 s of packets, one every 8 ms; playback reaches packet 5 at 800 ms.
  for (uint32_t i = 11; i < 136; i++) {
    now_us += 8 * MS;
    (void)feed(i);
    asks += (uint32_t)ask_playing(i * 8 < 800 ? 0 : 5 + (i * 8 - 800) / 8);
  }
  // Every 100 ms, on the first packet after: 104 ms apart.
  CHECK(asks >= 7 && asks <= 9, "asked %u times in 800 ms", asks);
  CHECK(rtp_resend_outstanding(&r) == 0 && r.counts.expired == 1,
        "open %u expired %u", rtp_resend_outstanding(&r), r.counts.expired);
  CHECK(balanced(), "counts do not add up");
}

static void test_three_per_packet(void) {
  printf("10 separate holes: 3 requests per arriving packet, oldest first\n");
  rtp_resend_reset(&r, STEP);
  for (uint32_t i = 0; i <= 20; i += 2) {
    (void)feed(i);
  }
  size_t n = ask();
  CHECK(n == 3 && ranges[0].first == 1 && ranges[1].first == 3 &&
            ranges[2].first == 5,
        "first packet: %zu, from %u", n, n ? ranges[0].first : 0);
  n = ask();
  CHECK(n == 3 && ranges[0].first == 7, "second packet: %zu, from %u", n,
        n ? ranges[0].first : 0);
  n = ask();
  CHECK(n == 3 && ranges[0].first == 13, "third: %zu, from %u", n,
        n ? ranges[0].first : 0);
  n = ask();
  CHECK(n == 1 && ranges[0].first == 19, "fourth: %zu, from %u", n,
        n ? ranges[0].first : 0);
  CHECK(ask() == 0, "asked again before the timeout");
  // Without an arriving packet, nothing at all is asked.
  now_us += 500 * MS;
  CHECK(rtp_resend_collect(&r, now_us, false, 0, ranges, 0) == 0,
        "asked on a timeout");
  // A run of consecutive holes is one request.
  rtp_resend_reset(&r, STEP);
  feed_around(0, 60, 20, 30);
  n = ask();
  CHECK(n == 1 && ranges[0].first == 20 && ranges[0].count == 30,
        "a run of 30: %zu ranges, %u+%u", n, n ? ranges[0].first : 0,
        n ? ranges[0].count : 0);
}

static void test_rtt_estimate(void) {
  printf("retransmit timeout follows the round trip (Karn, new min/max "
         "only bound it)\n");
  rtp_resend_reset(&r, STEP);
  CHECK(r.rto_us == 100000 && r.srtt_us == 100000, "initial rto %d", r.rto_us);
  uint32_t seq = 0;
  (void)feed(seq++);
  // Answers after 4 ms, then 6 ms, then a run alternating 4 and 6 ms.
  const int32_t rtts[] = {4000, 6000, 5000, 4000, 6000, 5000, 5000, 4000,
                          6000, 5000, 4000, 6000, 5000, 5000, 4000, 6000,
                          5000, 4000, 6000, 5000, 5000, 4000, 6000, 5000,
                          4000, 6000, 5000, 5000, 4000, 6000};
  const size_t n_rtts = sizeof(rtts) / sizeof(rtts[0]);
  for (size_t k = 0; k < n_rtts; k++) {
    const uint32_t hole = seq++;
    (void)feed(seq++);
    CHECK(ask() == 1 && ranges[0].first == (uint16_t)hole, "hole %u not asked",
          hole);
    now_us += rtts[k];
    CHECK(resend(hole) == RTP_RESEND_FILLED, "hole %u not filled", hole);
    if (k == 0) {
      // Measured in 128 us steps.
      CHECK(r.rtt_min_us >= 3968 && r.rtt_min_us <= 4096, "min %d",
            r.rtt_min_us);
      CHECK(r.srtt_us == 100000, "first sample moved srtt to %d", r.srtt_us);
    }
    now_us += 10 * MS;
  }
  // 28 samples inside [4, 6] ms from 100 ms: srtt ~ 100 x (7/8)^28 + 5.
  CHECK(r.srtt_us > 5000 && r.srtt_us < 10000, "srtt %d", r.srtt_us);
  CHECK(r.rto_us == 2 * r.srtt_us + 4 * r.rttvar_us && r.rto_us < 40000,
        "rto %d = 2 x %d + 4 x %d", r.rto_us, r.srtt_us, r.rttvar_us);
  // So a hole is now asked again after rto, not 100 ms.
  const uint32_t hole = seq++;
  (void)feed(seq++);
  CHECK(ask() == 1, "hole not asked");
  now_us += r.rto_us - 1000;
  CHECK(ask() == 0, "asked before the timeout");
  now_us += 1000 + RTP_RESEND_TICK_US;
  CHECK(ask() == 1 && ranges[0].first == (uint16_t)hole,
        "not asked after the timeout");
  // A 300 ms answer is a new maximum: bounds only, srtt unmoved.
  const int32_t srtt = r.srtt_us;
  now_us += 300 * MS;
  // Asked twice by now: Karn says no sample at all.
  CHECK(resend(hole) == RTP_RESEND_FILLED, "late answer not taken");
  CHECK(r.srtt_us == srtt, "a twice-asked answer moved srtt");
  const uint32_t hole2 = seq++;
  (void)feed(seq++);
  CHECK(ask() == 1, "hole2 not asked");
  now_us += 300 * MS;
  CHECK(resend(hole2) == RTP_RESEND_FILLED, "hole2 not filled");
  CHECK(r.srtt_us == srtt && r.rtt_max_us >= 299000, "max %d srtt %d",
        r.rtt_max_us, r.srtt_us);
  // The timeout never exceeds 100 ms however slow the answers.
  for (int k = 0; k < 40; k++) {
    const uint32_t h = seq++;
    (void)feed(seq++);
    (void)ask();
    now_us += 250 * MS;
    (void)resend(h);
  }
  CHECK(r.rto_us <= 100000, "rto %d", r.rto_us);
}

static void test_burst(void) {
  printf("a gap over 128 packets: nothing asked, everything dropped\n");
  rtp_resend_reset(&r, STEP);
  feed_around(0, 20, 10, 3);
  CHECK(rtp_resend_outstanding(&r) == 3, "open %u", rtp_resend_outstanding(&r));
  // 128 lost is still asked for.
  (void)feed(20 + 129);
  CHECK(rtp_resend_outstanding(&r) == 131 && r.counts.bursts == 0,
        "128 lost: open %u bursts %u", rtp_resend_outstanding(&r),
        r.counts.bursts);
  // 129 lost is a burst.
  (void)feed(149 + 130);
  CHECK(rtp_resend_outstanding(&r) == 0 && r.counts.bursts == 1 &&
            r.counts.aborted == 131 + 129,
        "129 lost: open %u bursts %u aborted %u", rtp_resend_outstanding(&r),
        r.counts.bursts, r.counts.aborted);
  CHECK(ask() == 0, "asked after a burst");
  CHECK(resend(15) == RTP_RESEND_UNWANTED, "an aborted hole was filled");
  CHECK(feed(280) == RTP_RESEND_NEWEST && balanced(), "stream did not go on");
}

static void test_refused(void) {
  printf("the sender's 'cannot resend' drops holes up to its sequence\n");
  rtp_resend_reset(&r, STEP);
  for (uint32_t i = 65500; i <= 65560; i += 2) {
    (void)feed(i); // holes at every odd number, across the wrap
  }
  const uint32_t open = rtp_resend_outstanding(&r);
  const uint32_t dropped = rtp_resend_refused(&r, (uint16_t)65531);
  CHECK(dropped == 16 && rtp_resend_outstanding(&r) == open - 16,
        "dropped %u of %u", dropped, open);
  CHECK(resend(65531) == RTP_RESEND_UNWANTED, "refused hole filled");
  CHECK(resend(65533) == RTP_RESEND_FILLED, "later hole dropped");
  const size_t n = ask();
  CHECK(n == 3 && ranges[0].first == (uint16_t)65535, "asked from %u",
        n ? ranges[0].first : 0);
  // A sequence number from before the window drops nothing.
  CHECK(rtp_resend_refused(&r, (uint16_t)(65560 - 2000)) == 0,
        "an old seq dropped some");
  CHECK(balanced(), "counts do not add up");
}

static void test_due_holes_dropped(void) {
  printf("holes already due to play are given up on\n");
  rtp_resend_reset(&r, STEP);
  feed_around(0, 199, 100, 10);
  // Playback has reached packet 105, so 100-104 can no longer be heard.
  const size_t n = ask_playing(105);
  CHECK(n == 1 && ranges[0].first == 105 && ranges[0].count == 5,
        "%zu ranges, the first %u+%u", n, n ? ranges[0].first : 0,
        n ? ranges[0].count : 0);
  CHECK(r.counts.expired == 5, "expired %u", r.counts.expired);
  CHECK(resend(102) == RTP_RESEND_UNWANTED, "a hole given up on was filled");
  CHECK(resend(107) == RTP_RESEND_FILLED, "an open hole was not filled");
  // A receive timeout gives up on due holes too, without asking.
  CHECK(rtp_resend_collect(&r, now_us, true, rtp_at(109), ranges, 0) == 0,
        "asked on a timeout");
  CHECK(rtp_resend_outstanding(&r) == 1, "open %u", rtp_resend_outstanding(&r));
  CHECK(balanced(), "counts do not add up");
}

static void test_window_end(void) {
  printf("a hole lasts one window\n");
  rtp_resend_reset(&r, STEP);
  feed_around(0, 10 + RTP_RESEND_WINDOW - 1U, 10, 1);
  CHECK(rtp_resend_outstanding(&r) == 1, "hole gone before the window passed");
  CHECK(ask() == 1 && ranges[0].first == 10, "last chance not taken");
  (void)feed(10 + RTP_RESEND_WINDOW);
  CHECK(rtp_resend_outstanding(&r) == 0 && r.counts.expired == 1,
        "open %u, expired %u", rtp_resend_outstanding(&r), r.counts.expired);
  CHECK(resend(10) == RTP_RESEND_UNWANTED, "taken after its window");
  CHECK(balanced(), "counts do not add up");
}

static void test_wraparound(void) {
  printf("holes across the sequence number wrap\n");
  rtp_resend_reset(&r, STEP);
  feed_around(65500, 65600, 65530, 11);
  const size_t n = ask();
  CHECK(n == 1 && ranges[0].first == 65530 && ranges[0].count == 11,
        "%zu ranges, the first %u+%u", n, n ? ranges[0].first : 0,
        n ? ranges[0].count : 0);
  for (uint32_t i = 65530; i <= 65540; i++) {
    CHECK(resend(i) == RTP_RESEND_FILLED, "packet %u not taken", i);
  }
  CHECK(balanced(), "counts do not add up");
}

static void test_packet_length_learnt(void) {
  printf("1024-frame packets: due holes found from the real length\n");
  rtp_resend_reset(&r, STEP);
  for (uint32_t i = 0; i < 200; i++) {
    if (i < 50 || i > 59) {
      (void)rtp_resend_on_packet(&r, (uint16_t)i, 5000U + i * 1024U, false,
                                 now_us);
    }
  }
  const size_t n = rtp_resend_collect(&r, now_us, true, 5000U + 55U * 1024U,
                                      ranges, MAX_REQ);
  CHECK(n == 1 && ranges[0].first == 55 && ranges[0].count == 5 &&
            r.counts.expired == 5,
        "%zu ranges, the first %u+%u, expired %u", n, n ? ranges[0].first : 0,
        n ? ranges[0].count : 0, r.counts.expired);
}

static void test_reset(void) {
  printf("before the first packet and after a reset\n");
  rtp_resend_reset(&r, STEP);
  CHECK(resend(7) == RTP_RESEND_UNWANTED, "retransmission before any packet");
  feed_around(0, 9, 3, 1);
  rtp_resend_reset(&r, STEP);
  CHECK(rtp_resend_outstanding(&r) == 0, "hole kept through a reset");
  CHECK(feed(500) == RTP_RESEND_NEWEST && ask() == 0,
        "stream after a reset not taken as new");
}

/* ---------- a lossy stream, v0.3.0 against the current tracker ---------- */

/* A realtime stream at 44.1 kHz: a 352-frame packet every 7982 us.  Loss is
 * a two-state (Gilbert-Elliott) chain, so it comes in bursts; requests and
 * retransmissions cross the same network.  The sender answers a request
 * after its round trip, packet by packet.  A hole is recovered if its
 * retransmission arrives before its audio is due, `latency` after it was
 * sent. */
#define PACKET_US 7982LL
#define SIM_S     120

typedef struct {
  const char *name;
  double good_to_bad, bad_to_good, loss_good, loss_bad;
  int64_t latency_us;
  double rtt_ms; // mean round trip of a request and its answer
  bool answers;  // does the sender answer at all
} loss_case_t;

static const loss_case_t cases[] = {
    {"bursty, 250 ms", 0.004, 0.05, 0.005, 0.5, 250 * MS, 6, true},
    {"bursty, 2 s", 0.004, 0.05, 0.005, 0.5, 2000 * MS, 6, true},
    {"5% random, 250 ms", 0, 1, 0.05, 0.05, 250 * MS, 6, true},
    {"5% random, slow Wi-Fi", 0, 1, 0.05, 0.05, 250 * MS, 40, true},
    {"no answers, bursty", 0.004, 0.05, 0.005, 0.5, 2000 * MS, 6, false},
};

static uint64_t rng;
static double urand(void) {
  rng ^= rng << 13;
  rng ^= rng >> 7;
  rng ^= rng << 17;
  return (double)(rng >> 11) / 9007199254740992.0;
}

typedef struct {
  int64_t at;
  uint32_t index;
  bool retransmit;
} arrival_t;

#define MAX_ARRIVALS 200000
static arrival_t arrivals[MAX_ARRIVALS];
static size_t n_arrivals;

static void push(int64_t at, uint32_t index, bool retransmit) {
  if (n_arrivals == MAX_ARRIVALS) {
    return;
  }
  size_t i = n_arrivals++;
  while (i > 0 && arrivals[(i - 1) / 2].at > at) {
    arrivals[i] = arrivals[(i - 1) / 2];
    i = (i - 1) / 2;
  }
  arrivals[i] = (arrival_t){at, index, retransmit};
}

static arrival_t pop(void) {
  const arrival_t top = arrivals[0];
  const arrival_t last = arrivals[--n_arrivals];
  size_t i = 0;
  for (;;) {
    size_t c = 2 * i + 1;
    if (c >= n_arrivals) {
      break;
    }
    if (c + 1 < n_arrivals && arrivals[c + 1].at < arrivals[c].at) {
      c++;
    }
    if (arrivals[c].at >= last.at) {
      break;
    }
    arrivals[i] = arrivals[c];
    i = c;
  }
  arrivals[i] = last;
  return top;
}

#define N_PACKETS (SIM_S * 1000000LL / PACKET_US)
static bool bad_at[N_PACKETS + 4096];
static bool original_lost[N_PACKETS];
static bool recovered[N_PACKETS];

static bool lost_at(int64_t t, const loss_case_t *lc) {
  const int64_t tick = t / PACKET_US;
  const bool bad =
      tick >= 0 && tick < (int64_t)(sizeof(bad_at)) && bad_at[tick];
  return urand() < (bad ? lc->loss_bad : lc->loss_good);
}

static double one_way_us(const loss_case_t *lc) {
  return lc->rtt_ms * 1000.0 / 2.0 * (0.5 - 0.5 * log(urand() + 1e-12));
}

typedef struct {
  uint32_t lost, recovered, requests, worst_second;
} sim_result_t;

static void sender_request(const loss_case_t *lc, int64_t t, uint16_t first,
                           uint16_t count, uint32_t newest) {
  if (!lc->answers || lost_at(t, lc)) {
    return;
  }
  const int64_t at_sender = t + (int64_t)one_way_us(lc);
  for (uint16_t k = 0; k < count; k++) {
    // Unwrap the sequence number against the newest packet.
    const uint16_t seq = (uint16_t)(first + k);
    const uint32_t index = newest - (uint16_t)(newest - seq);
    if (lost_at(at_sender, lc)) {
      continue;
    }
    push(at_sender + 300 + (int64_t)one_way_us(lc), index, true);
  }
}

static sim_result_t simulate(const loss_case_t *lc, bool old) {
  rng = 0x243F6A8885A308D3ULL;
  bool bad = false;
  for (size_t i = 0; i < sizeof(bad_at); i++) {
    const double u = urand();
    bad = bad ? u >= lc->bad_to_good : u < lc->good_to_bad;
    bad_at[i] = bad;
  }
  n_arrivals = 0;
  memset(original_lost, 0, sizeof(original_lost));
  memset(recovered, 0, sizeof(recovered));
  for (uint32_t i = 0; i < N_PACKETS; i++) {
    const int64_t sent = i * PACKET_US;
    if (lost_at(sent, lc)) {
      original_lost[i] = true;
      continue;
    }
    push(sent + 2 * MS, i, false);
  }
  if (old) {
    v030_reset(STEP);
  } else {
    rtp_resend_reset(&r, STEP);
  }

  sim_result_t res = {0};
  uint32_t newest = 0;
  int64_t last_event = 0;
  int64_t second = 0;
  uint32_t this_second = 0;
  while (n_arrivals > 0) {
    const arrival_t a = pop();
    // v0.3.0 also asked on every 100 ms receive timeout.
    while (old && a.at - last_event >= 100 * MS) {
      last_event += 100 * MS;
      const int64_t play = last_event - lc->latency_us;
      uint16_t first[16], count[16];
      const size_t n =
          v030_collect((uint32_t)(last_event / 1000), play >= 0,
                       (uint32_t)(play / PACKET_US) * STEP, first, count, 16);
      for (size_t k = 0; k < n; k++) {
        sender_request(lc, last_event, first[k], count[k], newest);
      }
      res.requests += (uint32_t)n;
    }
    last_event = a.at;
    if (a.at / 1000000 != second) {
      second = a.at / 1000000;
      this_second = 0;
    }
    const uint16_t seq = (uint16_t)a.index;
    const uint32_t rtp = a.index * STEP;
    const int64_t play = a.at - lc->latency_us;
    const bool playing = play >= 0;
    const uint32_t cursor = (uint32_t)(play / PACKET_US) * STEP;
    bool taken;
    size_t n;
    uint16_t first[16], count[16];
    if (old) {
      taken = v030_on_packet(seq, rtp, a.retransmit);
      n = v030_collect((uint32_t)(a.at / 1000), playing, cursor, first, count,
                       16);
    } else {
      taken = rtp_resend_on_packet(&r, seq, rtp, a.retransmit, a.at) !=
              RTP_RESEND_UNWANTED;
      rtp_resend_range_t rr[MAX_REQ];
      n = rtp_resend_collect(&r, a.at, playing, cursor, rr, MAX_REQ);
      for (size_t k = 0; k < n; k++) {
        first[k] = rr[k].first;
        count[k] = rr[k].count;
      }
    }
    if (!a.retransmit && a.index > newest) {
      newest = a.index;
    }
    if (taken && a.retransmit && original_lost[a.index] &&
        a.at < (int64_t)a.index * PACKET_US + lc->latency_us) {
      recovered[a.index] = true;
    }
    for (size_t k = 0; k < n; k++) {
      sender_request(lc, a.at, first[k], count[k], newest);
    }
    res.requests += (uint32_t)n;
    this_second += (uint32_t)n;
    if (this_second > res.worst_second) {
      res.worst_second = this_second;
    }
  }
  for (uint32_t i = 0; i < N_PACKETS; i++) {
    res.lost += original_lost[i];
    res.recovered += recovered[i];
  }
  return res;
}

static void test_lossy_stream(void) {
  printf("lossy streams, %d s each: v0.3.0 against the current tracker\n",
         SIM_S);
  printf("  %-24s %-7s %6s %10s %9s %8s\n", "case", "code", "lost", "recovered",
         "requests", "worst/s");
  for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
    const sim_result_t o = simulate(&cases[c], true);
    const sim_result_t n = simulate(&cases[c], false);
    printf("  %-24s %-7s %6u %9.1f%% %9u %8u\n", cases[c].name, "v0.3.0",
           o.lost, o.lost ? 100.0 * o.recovered / o.lost : 0, o.requests,
           o.worst_second);
    printf("  %-24s %-7s %6u %9.1f%% %9u %8u\n", "", "new", n.lost,
           n.lost ? 100.0 * n.recovered / n.lost : 0, n.requests,
           n.worst_second);
    if (cases[c].answers) {
      CHECK(n.recovered + n.lost / 100 >= o.recovered,
            "%s: recovered %u, v0.3.0 %u", cases[c].name, n.recovered,
            o.recovered);
    }
    // Never more than three requests per arriving packet.
    CHECK(n.worst_second <= RTP_RESEND_REQUESTS_PER_PACKET * 126U,
          "%s: %u requests in a second", cases[c].name, n.worst_second);
  }
}

int main(void) {
  test_in_order();
  test_one_hole();
  test_no_limit_while_useful();
  test_three_per_packet();
  test_rtt_estimate();
  test_burst();
  test_refused();
  test_due_holes_dropped();
  test_window_end();
  test_wraparound();
  test_packet_length_learnt();
  test_reset();
  test_lossy_stream();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}

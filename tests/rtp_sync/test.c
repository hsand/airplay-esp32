// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for main/audio/rtp_sync.c, the realtime stream's sync packets
 * turned into one RTP <-> network-time map per segment: the dead band, the
 * running average, re-anchoring on a flush bit or a jump, the deferred apply,
 * and a jittery sender against v0.3.0, which took every packet as the map. */

#include "rtp_sync.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define RATE  44100U
#define NS    1000000000ULL
#define NET0  (3976214400ULL * NS) // 2026 on the NTP scale
#define RTP0  0xFFFF0000U          // wraps during the runs
#define APPLY 77175U               // sender latency, 1.75 s

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

static rtp_sync_t s;
static rtp_sync_anchor_t out;

// A packet saying `rtp` plays at net, `offset` samples off the true line.
static rtp_sync_action_t announce(uint32_t second, int32_t offset,
                                  uint8_t flags) {
  const uint32_t rtp = RTP0 + second * RATE + (uint32_t)offset;
  return rtp_sync_announce(&s, RATE, 0, flags, rtp, NET0 + second * NS,
                           rtp + APPLY, &out);
}

// Where the current map has `rtp` playing, against the true line, samples.
static double map_error(uint32_t second) {
  rtp_sync_anchor_t a;
  const uint32_t rtp = RTP0 + second * RATE;
  if (!rtp_sync_current(&s, rtp, &a)) {
    return NAN;
  }
  // True: rtp plays at NET0 + second; the map says a.network_ns.  A map
  // that plays it later is behind: that is a positive error in samples.
  return ((double)(int64_t)(NET0 + second * NS - a.network_ns)) * RATE / 1e9;
}

static void test_first_anchors(void) {
  printf("the first packet anchors at once\n");
  rtp_sync_reset(&s);
  CHECK(announce(0, 0, RTP_SYNC_FLAG_FIRST_AFTER_FLUSH) == RTP_SYNC_PUBLISH,
        "first packet not published");
  CHECK(out.rtp == RTP0 && out.network_ns == NET0, "anchor %u", out.rtp);
}

static void test_dead_band(void) {
  printf("errors under 16 samples leave the map alone\n");
  rtp_sync_reset(&s);
  (void)announce(0, 0, 0);
  for (uint32_t i = 1; i < 100; i++) {
    const int32_t jitter = (int32_t)(i * 7 % 31) - 15; // -15..15
    CHECK(announce(i, jitter, 0) == RTP_SYNC_KEEP, "packet %u moved it", i);
  }
  CHECK(s.average == 0 && s.nudges == 0, "average %d", s.average);
  CHECK(fabs(map_error(100)) < 1e-3, "map moved %f", map_error(100));
}

static void test_average(void) {
  printf("a steady 40-sample error is followed 1/16 at a time\n");
  rtp_sync_reset(&s);
  (void)announce(0, 0, 0);
  CHECK(announce(1, 40, 0) == RTP_SYNC_PUBLISH && s.average == 2, "average %d",
        s.average);
  // The published anchor is re-expressed at the packet, on the moved map.
  CHECK(out.rtp == RTP0 + RATE + 40U, "published at %u", out.rtp);
  uint32_t i = 2;
  while (i < 100 && announce(i, 40, 0) != RTP_SYNC_KEEP) {
    i++;
  }
  CHECK(s.average == 25, "settled at %d after %u packets", s.average, i);
  CHECK(40 - s.average < RTP_SYNC_DEAD_BAND, "left %d", 40 - s.average);
  // Negative too.
  rtp_sync_reset(&s);
  (void)announce(0, 0, 0);
  CHECK(announce(1, -40, 0) == RTP_SYNC_PUBLISH && s.average == -2,
        "average %d", s.average);
}

static void test_jump_defers(void) {
  printf("a jump over 4410 samples re-anchors, from its apply point\n");
  rtp_sync_reset(&s);
  (void)announce(0, 0, 0);
  CHECK(announce(1, 4410, 0) == RTP_SYNC_PUBLISH, "4410 is a jump");
  rtp_sync_reset(&s);
  (void)announce(0, 0, 0);
  CHECK(announce(1, 5000, 0) == RTP_SYNC_DEFER && s.jumps == 1,
        "5000 not deferred");
  const uint32_t apply = RTP0 + RATE + 5000U + APPLY;
  // Still playing the old audio: not yet.
  CHECK(!rtp_sync_poll(&s, true, apply - 1000U, &out), "applied early");
  // Later packets measure against the new anchor meanwhile.
  CHECK(announce(2, 5000, 0) == RTP_SYNC_DEFER && s.average == 0, "average %d",
        s.average);
  CHECK(rtp_sync_poll(&s, true, apply, &out), "not applied at apply");
  CHECK(out.rtp == RTP0 + 2U * RATE + 5000U && out.network_ns == NET0 + 2 * NS,
        "map %u", out.rtp);
  CHECK(!rtp_sync_poll(&s, true, apply + 1U, &out), "applied twice");
  CHECK(s.applied == 1 && s.forced == 0, "applied %u forced %u", s.applied,
        s.forced);
}

static void test_flush_bit(void) {
  printf("the first-sync-after-flush bit re-anchors, deferred\n");
  rtp_sync_reset(&s);
  (void)announce(0, 0, 0);
  CHECK(announce(1, 3, RTP_SYNC_FLAG_FIRST_AFTER_FLUSH) == RTP_SYNC_DEFER &&
            s.resets == 1,
        "flush bit not deferred");
  // Nothing playing: due at once.
  CHECK(rtp_sync_poll(&s, false, 0, &out), "not applied while idle");
  // Far behind the apply point: forced.
  (void)announce(2, 0, RTP_SYNC_FLAG_FIRST_AFTER_FLUSH);
  CHECK(rtp_sync_poll(&s, true, RTP0 + 2U * RATE - 500000U, &out) &&
            s.forced == 1,
        "not forced");
}

static void test_timeline_change(void) {
  printf("another PTP timeline anchors at once\n");
  rtp_sync_reset(&s);
  rtp_sync_anchor_t a;
  CHECK(rtp_sync_announce(&s, RATE, 0x1111, 0, RTP0, NET0, RTP0, &a) ==
            RTP_SYNC_PUBLISH,
        "first not published");
  CHECK(rtp_sync_announce(&s, RATE, 0x1111, 0, RTP0 + RATE, NET0 + NS, RTP0,
                          &a) == RTP_SYNC_KEEP,
        "same timeline moved the map");
  CHECK(rtp_sync_announce(&s, RATE, 0x2222, 0, RTP0 + 2 * RATE, 12345, RTP0,
                          &a) == RTP_SYNC_PUBLISH &&
            a.network_ns == 12345 && s.timeline == 0x2222,
        "new timeline not published");
}

static void test_long_segment(void) {
  printf("a 30 h segment: the anchor is re-based, the map stays put\n");
  rtp_sync_reset(&s);
  (void)announce(0, 0, 0);
  for (uint32_t i = 60; i < 30U * 3600U; i += 60) {
    CHECK(announce(i, 0, 0) == RTP_SYNC_KEEP, "packet at %u s moved it", i);
    if (failures) {
      break;
    }
  }
  CHECK(fabs(map_error(30U * 3600U)) < 0.01, "map off by %f samples",
        map_error(30U * 3600U));
}

/* A sender whose sync packets carry its timestamping jitter, on a clock 2 ppm
 * off its audio, for 10 minutes; the error of the map the engine is given,
 * sampled each 10 ms.  v0.3.0 handed every packet over as the map. */
static void test_jittery_sender(void) {
  printf("jittery sender, 10 min: error of the map handed to the scheduler\n");
  uint64_t rng = 0x9E3779B97F4A7C15ULL;
  const double jitter_samples[] = {5, 20, 60};
  for (size_t j = 0; j < 3; j++) {
    rtp_sync_reset(&s);
    double old_last = 0, anchor_off = 0, sum_old = 0, sum_new = 0, max_old = 0,
           max_new = 0;
    uint32_t publishes = 0;
    int n = 0;
    for (uint32_t ms = 0; ms < 600000; ms += 10) {
      const double drift = 2e-6 * RATE * (ms / 1000.0); // samples
      if (ms % 1000 == 0) {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        const double u = (double)(rng >> 11) / 9007199254740992.0;
        const double jit = (2 * u - 1) * jitter_samples[j];
        const int32_t off = (int32_t)lround(drift + jit);
        if (announce(ms / 1000, off, 0) == RTP_SYNC_PUBLISH) {
          publishes++;
        }
        if (ms == 0) {
          anchor_off = off; // the anchor carries the first packet's error
        }
        old_last = off;
      }
      if (ms < 1000) {
        continue;
      }
      // Errors against the sender's true (drifting) audio position.
      const double e_old = old_last - drift;
      const double e_new = anchor_off + (double)s.average - drift;
      sum_old += e_old * e_old;
      sum_new += e_new * e_new;
      max_old = fmax(max_old, fabs(e_old));
      max_new = fmax(max_new, fabs(e_new));
      n++;
    }
    printf("  jitter +-%2.0f samples: rms %5.1f / %5.1f samples, peak %5.1f / "
           "%5.1f (v0.3.0 / new), %u map changes in 600 (v0.3.0: 600)\n",
           jitter_samples[j], sqrt(sum_old / n), sqrt(sum_new / n), max_old,
           max_new, publishes);
    CHECK(max_new <= RTP_SYNC_DEAD_BAND + jitter_samples[j] / 4 + 2,
          "peak %.1f", max_new);
  }
}

int main(void) {
  test_first_anchors();
  test_dead_band();
  test_average();
  test_jump_defers();
  test_flush_bit();
  test_timeline_change();
  test_long_segment();
  test_jittery_sender();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}

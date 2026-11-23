// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for the AirPlay 2 control plane: the FLUSHBUFFERED queue and
 * timeline sweep (audio_flush_queue.c, audio_timeline.c), request validation
 * (rtsp_anchor.c), timing-peer extraction and the SETUP/status replies
 * (bplist_parser.c, bplist_builder.c).  run.py writes the request plists and
 * checks the replies with plistlib. */

#include "sim_platform.h"

#include <string.h>

#include "audio_flush_queue.h"
#include "audio_timeline.h"
#include "plist.h"
#include "rtsp_anchor.h"

int64_t sim_now_us;
void sim_log(char level, const char *tag, const char *fmt, ...) {
  (void)level;
  (void)tag;
  (void)fmt;
}

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

static const char *plist_dir;

static size_t load(const char *name, uint8_t *buf, size_t cap) {
  char path[512];
  snprintf(path, sizeof(path), "%s/%s", plist_dir, name);
  FILE *f = fopen(path, "rb");
  if (!f) {
    printf("  cannot open %s\n", path);
    exit(2);
  }
  const size_t n = fread(buf, 1, cap, f);
  fclose(f);
  return n;
}

/* ---------- the flush queue ---------- */

/* Feed `count` packets from `first` in stream order and record which drop. */
static void feed(audio_flush_queue_t *q, uint32_t first, uint32_t count,
                 bool *dropped) {
  for (uint32_t i = 0; i < count; i++) {
    dropped[i] =
        audio_flush_queue_should_drop(q, (first + i) & AUDIO_SEQ24_MASK);
  }
}

static void test_flush_immediate(void) {
  printf("flush queue: immediate drops everything before untilSeq\n");
  audio_flush_queue_t q;
  audio_flush_queue_reset(&q);
  const audio_flush_record_t r = {.has_from = false, .until_seq = 50};
  CHECK(audio_flush_queue_push(&q, &r), "an immediate flush is processed now");
  bool d[100];
  feed(&q, 0, 100, d);
  for (int i = 0; i < 100; i++) {
    CHECK(d[i] == (i < 50), "seq %d %s", i, d[i] ? "dropped" : "kept");
  }
  CHECK(audio_flush_queue_pending(&q) == 0, "retired by seq 50");
  /* Once retired, a late straggler from before the flush is not dropped:
   * the record is gone, as on Apple. */
  CHECK(!audio_flush_queue_should_drop(&q, 10), "no record left");
}

static void test_flush_deferred(void) {
  printf("flush queue: deferred drops [from, until) and keeps both sides\n");
  audio_flush_queue_t q;
  audio_flush_queue_reset(&q);
  const audio_flush_record_t r = {
      .has_from = true, .from_seq = 20, .until_seq = 30};
  CHECK(audio_flush_queue_push(&q, &r), "processed at once on an empty queue");
  bool d[60];
  feed(&q, 0, 60, d);
  for (int i = 0; i < 60; i++) {
    CHECK(d[i] == (i >= 20 && i < 30), "seq %d %s", i,
          d[i] ? "dropped" : "kept");
  }
  CHECK(audio_flush_queue_pending(&q) == 0, "retired");
}

static void test_flush_queued(void) {
  printf("flush queue: a second deferred flush queues behind the first\n");
  audio_flush_queue_t q;
  audio_flush_queue_reset(&q);
  const audio_flush_record_t a = {
      .has_from = true, .from_seq = 10, .until_seq = 20};
  const audio_flush_record_t b = {
      .has_from = true, .from_seq = 25, .until_seq = 40};
  CHECK(audio_flush_queue_push(&q, &a), "first at once");
  CHECK(!audio_flush_queue_push(&q, &b), "second appended (no RTP-Info)");
  CHECK(audio_flush_queue_pending(&q) == 2, "two pending");
  bool d[50];
  feed(&q, 0, 50, d);
  for (int i = 0; i < 50; i++) {
    const bool want = (i >= 10 && i < 20) || (i >= 25 && i < 40);
    CHECK(d[i] == want, "seq %d %s", i, d[i] ? "dropped" : "kept");
  }
  CHECK(audio_flush_queue_pending(&q) == 0, "both retired");
}

static void test_flush_immediate_cancels(void) {
  printf("flush queue: an immediate flush cancels pending deferred ones\n");
  audio_flush_queue_t q;
  audio_flush_queue_reset(&q);
  const audio_flush_record_t a = {
      .has_from = true, .from_seq = 10, .until_seq = 20};
  const audio_flush_record_t b = {
      .has_from = true, .from_seq = 30, .until_seq = 40};
  const audio_flush_record_t now = {.has_from = false, .until_seq = 15};
  (void)audio_flush_queue_push(&q, &a);
  (void)audio_flush_queue_push(&q, &b);
  CHECK(audio_flush_queue_push(&q, &now), "processed at once");
  CHECK(audio_flush_queue_pending(&q) == 1, "only the immediate one");
  bool d[50];
  feed(&q, 0, 50, d);
  for (int i = 0; i < 50; i++) {
    CHECK(d[i] == (i < 15), "seq %d %s", i, d[i] ? "dropped" : "kept");
  }
}

static void test_flush_wrap(void) {
  printf("flush queue: 24-bit sequence wrap\n");
  audio_flush_queue_t q;
  audio_flush_queue_reset(&q);
  const audio_flush_record_t r = {
      .has_from = true, .from_seq = 0xFFFFF8, .until_seq = 0x000008};
  (void)audio_flush_queue_push(&q, &r);
  bool d[32];
  feed(&q, 0xFFFFF0, 32, d);
  for (int i = 0; i < 32; i++) {
    const bool want = i >= 8 && i < 24;
    CHECK(d[i] == want, "seq 0x%06x %s", (0xFFFFF0 + i) & 0xFFFFFF,
          d[i] ? "dropped" : "kept");
  }
  CHECK(audio_seq24_diff(0x000002, 0xFFFFFE) == 4, "diff across the wrap");
  CHECK(audio_seq24_diff(0xFFFFFE, 0x000002) == -4, "and back");
}

static void test_flush_overflow(void) {
  printf("flush queue: overflow widens the newest record\n");
  audio_flush_queue_t q;
  audio_flush_queue_reset(&q);
  for (uint32_t i = 0; i < AUDIO_FLUSH_QUEUE_DEPTH + 2; i++) {
    const audio_flush_record_t r = {
        .has_from = true, .from_seq = 100 * i + 10, .until_seq = 100 * i + 20};
    (void)audio_flush_queue_push(&q, &r);
  }
  CHECK(audio_flush_queue_pending(&q) == AUDIO_FLUSH_QUEUE_DEPTH, "%u pending",
        (unsigned)audio_flush_queue_pending(&q));
  /* The last range must still be dropped. */
  const uint32_t last = 100 * (AUDIO_FLUSH_QUEUE_DEPTH + 1) + 15;
  for (uint32_t s = 0; s < last; s++) {
    (void)audio_flush_queue_should_drop(&q, s);
  }
  CHECK(audio_flush_queue_should_drop(&q, last), "seq %u kept", last);
}

/* ---------- the timeline sweep ---------- */

static void test_timeline_sweep(void) {
  printf("timeline: FLUSHBUFFERED sweep by sequence number\n");
  audio_timeline_t t;
  CHECK(audio_timeline_init(&t, 16, 1024) == ESP_OK, "init");
  int16_t *pcm = NULL;
  audio_timeline_reservation_t r;
  /* Blocks seq 100..109 at RTP 5000 + 1024 n, and one without a seq. */
  for (uint32_t n = 0; n < 10; n++) {
    CHECK(audio_timeline_reserve(&t, 1, 5000 + 1024 * n,
                                 AUDIO_TIMELINE_SEQ_VALID | (100 + n), &r,
                                 &pcm),
          "reserve %u", n);
    memset(pcm, 1, 1024 * 2 * sizeof(int16_t));
    CHECK(audio_timeline_commit(&t, &r), "commit %u", n);
  }
  CHECK(audio_timeline_reserve(&t, 1, 5000 + 1024 * 10, 0, &r, &pcm),
        "untagged");
  CHECK(audio_timeline_commit(&t, &r), "commit untagged");
  CHECK(audio_timeline_count(&t) == 11, "11 blocks");

  CHECK(audio_timeline_discard_seq_range(&t, 1, true, 103, 106) == 3,
        "deferred [103, 106) drops 3");
  CHECK(audio_timeline_count(&t) == 8, "8 left");
  uint32_t start = 0;
  CHECK(audio_timeline_find_contiguous_from(&t, 1, 5000, 3 * 1024, 0, &start),
        "102 and before intact");
  CHECK(!audio_timeline_find_contiguous_from(&t, 1, 5000 + 3 * 1024, 1, 0,
                                             &start),
        "103 gone");
  CHECK(audio_timeline_find_contiguous_from(&t, 1, 5000 + 6 * 1024, 4 * 1024, 0,
                                            &start),
        "106 on intact");

  CHECK(audio_timeline_discard_seq_range(&t, 1, false, 0, 108) == 5,
        "immediate below 108 drops 100-102 and 106-107");
  CHECK(audio_timeline_count(&t) == 3, "108, 109 and the untagged block left");
  CHECK(audio_timeline_discard_seq_range(&t, 2, false, 0, 0xFFFFFF) == 0,
        "another epoch untouched");
  audio_timeline_deinit(&t);
}

/* ---------- SETRATEANCHORTIME ---------- */

static bool parse_anchor(const char *name, audio_rate_anchor_t *a,
                         int64_t *status) {
  uint8_t buf[1024];
  const size_t n = load(name, buf, sizeof(buf));
  return rtsp_parse_rate_anchor(buf, n, 44100, a, status);
}

static void test_anchor_validation(void) {
  printf("SETRATEANCHORTIME validation\n");
  audio_rate_anchor_t a;
  int64_t status = 0;

  CHECK(parse_anchor("anchor_full", &a, &status), "full anchor refused");
  CHECK(a.rate == 1 && a.rtp == 123456, "rate %u rtp %u", (unsigned)a.rate,
        (unsigned)a.rtp);
  CHECK(a.network_ns == 1700000000ULL * 1000000000ULL + 500000000ULL,
        "network %llu", (unsigned long long)a.network_ns);
  CHECK(a.timeline_id == 0x1122334455667788ULL, "timeline");
  CHECK(a.fas_valid && a.fas_rtp == 123999, "first audible sample");

  CHECK(parse_anchor("anchor_rtp0", &a, &status) && a.rtp == 0 && !a.fas_valid,
        "an rtpTime of 0 is an anchor; no FAS is no FAS");
  CHECK(!parse_anchor("anchor_no_flags", &a, &status) &&
            status == AIRPLAY_STATUS_NOT_FOUND_ERR,
        "networkTimeFlags is required at rate 1 (status %lld)",
        (long long)status);
  CHECK(!parse_anchor("anchor_no_rtp", &a, &status) &&
            status == AIRPLAY_STATUS_NOT_FOUND_ERR,
        "the anchor time is required at rate 1");
  CHECK(parse_anchor("anchor_media_time", &a, &status) && a.rtp == 88200,
        "media time at the stream rate");
  CHECK(parse_anchor("anchor_big_frac", &a, &status) &&
            a.network_ns % 1000000000ULL == 999999999ULL,
        "a fraction past INT64_MAX: %llu",
        (unsigned long long)(a.network_ns % 1000000000ULL));
  CHECK(parse_anchor("rate0_bare", &a, &status) && a.rate == 0,
        "rate 0 needs no time keys");
  CHECK(!parse_anchor("rate2", &a, &status) &&
            status == AIRPLAY_STATUS_PARAM_ERR,
        "rate may only be 0 or 1");
  CHECK(parse_anchor("rate_real1", &a, &status) && a.rate == 1 && a.rtp == 7,
        "a real 1.0");
  CHECK(!parse_anchor("rate_real_half", &a, &status) &&
            status == AIRPLAY_STATUS_PARAM_ERR,
        "a real 0.5");
  CHECK(!parse_anchor("rate_missing", &a, &status), "no rate");
}

/* ---------- FLUSHBUFFERED parsing ---------- */

static bool parse_flush(const char *name, audio_flush_record_t *r) {
  uint8_t buf[512];
  const size_t n = load(name, buf, sizeof(buf));
  return rtsp_parse_flush_buffered(buf, n, 44100, r);
}

static void test_flush_parse(void) {
  printf("FLUSHBUFFERED parsing\n");
  audio_flush_record_t r;
  CHECK(parse_flush("flush_deferred", &r) && r.has_from && r.from_seq == 100 &&
            r.until_seq == 200 && r.from_ts == 5000 && r.until_ts == 5000,
        "deferred");
  CHECK(parse_flush("flush_immediate", &r) && !r.has_from &&
            r.until_seq == 300 && r.until_ts == 9000,
        "immediate");
  CHECK(parse_flush("flush_from_without_ts", &r) && !r.has_from,
        "fromSeq without a valid fromTS is a flush-all, as on Apple");
  CHECK(!parse_flush("flush_no_until", &r), "untilSeq is required");
}

/* ---------- timing peers ---------- */

static bool has(const uint32_t *addrs, size_t n, uint8_t a, uint8_t b,
                uint8_t c, uint8_t d) {
  const uint8_t want[4] = {a, b, c, d};
  uint32_t w;
  memcpy(&w, want, 4);
  for (size_t i = 0; i < n; i++) {
    if (addrs[i] == w) {
      return true;
    }
  }
  return false;
}

static void test_peers(void) {
  printf("timing peers\n");
  uint8_t buf[1024];
  uint32_t addrs[16];
  size_t n = 0;
  size_t len = load("setup_peers", buf, sizeof(buf));
  CHECK(bplist_collect_ipv4(buf, len, "timingPeerInfo", addrs, 16, &n) &&
            n == 1 && has(addrs, n, 192, 168, 1, 20),
        "timingPeerInfo: %u", (unsigned)n);
  CHECK(bplist_collect_ipv4(buf, len, "timingPeerList", addrs, 16, &n) &&
            n == 3 && has(addrs, n, 192, 168, 1, 30) &&
            has(addrs, n, 192, 168, 1, 20) && has(addrs, n, 10, 0, 0, 7),
        "timingPeerList: %u", (unsigned)n);
  CHECK(!bplist_collect_ipv4(buf, len, "absent", addrs, 16, &n) && n == 0,
        "absent key");
  len = load("setpeers", buf, sizeof(buf));
  CHECK(bplist_collect_ipv4(buf, len, NULL, addrs, 16, &n) && n == 2 &&
            has(addrs, n, 192, 168, 1, 40) && has(addrs, n, 192, 168, 1, 41),
        "SETPEERS: %u (IPv6 skipped, duplicates dropped)", (unsigned)n);
  len = load("setpeersx", buf, sizeof(buf));
  CHECK(bplist_collect_ipv4(buf, len, NULL, addrs, 16, &n) && n == 2 &&
            has(addrs, n, 192, 168, 1, 50) && has(addrs, n, 192, 168, 1, 51),
        "SETPEERSX: %u", (unsigned)n);
  CHECK(bplist_collect_ipv4(buf, len, NULL, addrs, 1, &n) && n == 1,
        "capacity respected");
  /* Truncated input must not be read past. */
  for (size_t cut = 0; cut < len; cut++) {
    (void)bplist_collect_ipv4(buf, cut, NULL, addrs, 16, &n);
  }
}

/* ---------- replies ---------- */

static void write_file(const char *dir, const char *name, const uint8_t *buf,
                       size_t len) {
  char path[512];
  snprintf(path, sizeof(path), "%s/%s", dir, name);
  FILE *f = fopen(path, "wb");
  fwrite(buf, 1, len, f);
  fclose(f);
}

static void test_replies(const char *out_dir) {
  printf("replies\n");
  uint8_t buf[256];
  const size_t setup =
      bplist_build_session_setup(buf, sizeof(buf), 51234, "192.168.1.99",
                                 "a1b2c3d4-0000-4000-8000-0011223344AA");
  CHECK(setup > 0, "session SETUP reply built");
  write_file(out_dir, "setup_reply.bplist", buf, setup);
  for (size_t cap = 0; cap < setup; cap++) {
    CHECK(bplist_build_session_setup(buf, cap, 51234, "192.168.1.99",
                                     "a1b2c3d4-0000-4000-8000-0011223344AA") ==
              0,
          "fits in %u", (unsigned)cap);
  }
  const size_t status = bplist_build_status(buf, sizeof(buf), -6705);
  CHECK(status > 0, "status reply built");
  write_file(out_dir, "status_reply.bplist", buf, status);
}

int main(int argc, char **argv) {
  if (argc < 3) {
    return 2;
  }
  plist_dir = argv[1];
  test_flush_immediate();
  test_flush_deferred();
  test_flush_queued();
  test_flush_immediate_cancels();
  test_flush_wrap();
  test_flush_overflow();
  test_timeline_sweep();
  test_anchor_validation();
  test_flush_parse();
  test_peers();
  test_replies(argv[2]);
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for main/network/ptp_clock.c.  The file is included whole and
 * driven with real PTP packets: a scripted master answers the clock's
 * Delay_Reqs (captured through a sendto() mock), so the two-way path, the
 * passive fallback, the grandmaster/port choice, the session rules and the
 * lwIP timestamp hook are exercised end to end, without sockets.
 *
 * Rewritten for the two-way clock.  Tests whose premise changed say so:
 *  - congestion is now told from a clock step by the round trip (delay
 *    gate), not by the size and length of a fall;
 *  - every step, forward or back, large or small, is followed after Apple's
 *    24 SYNCs (~3 s): v0.3.0 took a forward step at once and a fall under
 *    500 ms only after 160 SYNCs (~20 s);
 *  - samples come from one chosen port, so "mixed masters" no longer
 *    exists; ports and GMs are tested instead. */

#include "ptp_clock.c"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#define MASTER_A      0x5c3e1b15cc4d0008ULL
#define MASTER_B      0x0011223344556677ULL
#define TRUE_OFFSET   146894076359526LL /* ns, as seen from an iPhone */
#define US            1000LL
#define MS            1000000LL
#define SYNC_INTERVAL 125         /* ms, 8 Hz */
#define IP_A          0x0a01a8c0u /* 192.168.1.10 */
#define IP_B          0x1401a8c0u /* 192.168.1.20 */
#define IP_R          0x1e01a8c0u /* 192.168.1.30 */

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

/* ---------- mocks ---------- */

int64_t mock_now_us = 1000000;
static _Thread_local unsigned critical_depth;
static void (*log_hook)(void);

void mock_enter_critical(portMUX_TYPE *mux) {
  if (pthread_mutex_lock(mux) != 0) {
    abort();
  }
  critical_depth++;
}

void mock_exit_critical(portMUX_TYPE *mux) {
  critical_depth--;
  if (pthread_mutex_unlock(mux) != 0) {
    abort();
  }
}

void mock_log(const char *tag, const char *format, ...) {
  if (critical_depth != 0) {
    fputs("FAIL: logging while the clock-state mux is held\n", stderr);
    abort();
  }
  if (log_hook) {
    void (*hook)(void) = log_hook;
    log_hook = NULL;
    hook();
  }
  if (getenv("PTP_TEST_VERBOSE") == NULL) {
    return;
  }
  va_list args;
  va_start(args, format);
  printf("    [%s] ", tag);
  vprintf(format, args);
  printf("\n");
  va_end(args);
}

BaseType_t task_create_spiram(TaskFunction_t fn, const char *name,
                              uint32_t depth, void *param, int prio,
                              TaskHandle_t *handle, spiram_task_mem_t *mem) {
  (void)fn, (void)name, (void)depth, (void)param, (void)prio, (void)handle;
  (void)mem;
  return 0;
}

void task_free_spiram(spiram_task_mem_t *mem) {
  (void)mem;
}

static const uint8_t board_mac[6] = {0x24, 0x0a, 0xc4, 0x12, 0x34, 0x56};

esp_err_t esp_read_mac(uint8_t *mac, int type) {
  (void)type;
  memcpy(mac, board_mac, 6);
  return ESP_OK;
}

/* Captured Delay_Reqs (thread-local: the concurrency test sends from a
 * second thread). */
static _Thread_local uint8_t dreq[64];
static _Thread_local int dreq_count;
static _Thread_local uint32_t dreq_ip;
static _Thread_local uint16_t dreq_port;
static _Thread_local int64_t dreq_local_ns;

ssize_t mock_sendto(int fd, const void *buf, size_t len, int flags,
                    const struct sockaddr *dst, socklen_t dst_len) {
  (void)fd, (void)flags, (void)dst_len;
  if (critical_depth != 0) {
    fputs("FAIL: sendto while the clock-state mux is held\n", stderr);
    abort();
  }
  const struct sockaddr_in *in = (const struct sockaddr_in *)dst;
  memcpy(dreq, buf, len < sizeof(dreq) ? len : sizeof(dreq));
  dreq_count++;
  dreq_ip = in->sin_addr.s_addr;
  dreq_port = ntohs(in->sin_port);
  dreq_local_ns = (int64_t)mock_now_us * 1000;
  return (ssize_t)len;
}

/* ---------- a scripted master ---------- */

typedef struct {
  uint64_t clock;
  uint32_t ip;
  uint16_t port;
  uint16_t seq;
  bool answers;
  int64_t offset;     /* master = local + offset + drift */
  double ppm;         /* master rate against the board */
  int64_t base_local; /* where drift starts */
  int8_t log_interval;
  uint64_t apple_gm; /* Apple TLV in FOLLOW_UP, 0 = none */
  bool tlv8021as;
  uint16_t time_base;
} master_t;

static int64_t now_ns(void) {
  return (int64_t)mock_now_us * 1000;
}

static int64_t master_time(const master_t *m, int64_t local_ns) {
  return local_ns + m->offset +
         (int64_t)((double)(local_ns - m->base_local) * m->ppm * 1e-6);
}

static void put_be(uint8_t *p, uint64_t v, int n) {
  for (int i = n - 1; i >= 0; i--) {
    p[i] = (uint8_t)v;
    v >>= 8;
  }
}

static void put_ts(uint8_t *p, int64_t ns) {
  put_be(p, (uint64_t)ns / 1000000000ULL, 6);
  put_be(p + 6, (uint64_t)ns % 1000000000ULL, 4);
}

static void header(uint8_t *p, size_t len, uint8_t type, uint64_t clock,
                   uint16_t port, uint16_t seq, int8_t log_interval) {
  memset(p, 0, len);
  p[0] = type;
  p[1] = 2;
  put_be(p + 2, len, 2);
  if (type == PTP_MSG_SYNC) {
    p[6] = 2; /* twoStepFlag */
  }
  put_be(p + 20, clock, 8);
  put_be(p + 28, port, 2);
  put_be(p + 30, seq, 2);
  p[33] = (uint8_t)log_interval;
}

static void send_sync(const master_t *m, uint16_t seq) {
  uint8_t p[44];
  header(p, sizeof(p), PTP_MSG_SYNC, m->clock, m->port, seq, m->log_interval);
  handle_packet(p, sizeof(p), true, m->ip, now_ns());
}

static void send_followup(const master_t *m, uint16_t seq, int64_t t1) {
  uint8_t p[128];
  size_t len = 44;
  memset(p, 0, sizeof(p));
  if (m->tlv8021as) {
    uint8_t *t = p + len;
    put_be(t, 3, 2);
    put_be(t + 2, 28, 2);
    put_be(t + 4, 0x0080C2, 3);
    put_be(t + 7, 1, 3);
    put_be(t + 14, m->time_base, 2);
    len += 32;
  }
  if (m->apple_gm) {
    uint8_t *t = p + len;
    put_be(t, 3, 2);
    put_be(t + 2, 14, 2);
    put_be(t + 4, 0x000D93, 3);
    put_be(t + 7, 4, 3);
    put_be(t + 10, m->apple_gm, 8);
    len += 18;
  }
  uint8_t h[44];
  header(h, sizeof(h), PTP_MSG_FOLLOW_UP, m->clock, m->port, seq,
         m->log_interval);
  memcpy(p, h, 34);
  put_be(p + 2, len, 2);
  put_ts(p + 34, t1);
  handle_packet(p, len, false, m->ip, 0);
}

static void send_announce(const master_t *m, uint64_t gm, uint8_t p1,
                          uint16_t steps) {
  uint8_t p[64];
  header(p, sizeof(p), PTP_MSG_ANNOUNCE, m->clock, m->port, 0, 0);
  p[47] = p1;
  p[48] = 248;
  p[49] = 0x21;
  put_be(p + 50, 0x436A, 2);
  p[52] = 248;
  put_be(p + 53, gm, 8);
  put_be(p + 61, steps, 2);
  handle_packet(p, sizeof(p), false, m->ip, 0);
}

/* Answer the Delay_Req the clock just sent, if it was for this master. */
static void answer(const master_t *m, int64_t rev_us) {
  if (!m->answers || dreq_ip != m->ip || dreq_count == 0) {
    return;
  }
  const int64_t t4 = master_time(m, dreq_local_ns + rev_us * US);
  uint8_t p[54];
  header(p, sizeof(p), PTP_MSG_DELAY_RESP, m->clock, m->port,
         (uint16_t)((dreq[30] << 8) | dreq[31]), m->log_interval);
  put_ts(p + 34, t4);
  memcpy(p + 44, dreq + 20, 10);
  handle_packet(p, sizeof(p), false, m->ip, 0);
}

/* One SYNC interval: SYNC whose trip took fwd_ms, FOLLOW_UP, and the answer
 * to the Delay_Req, whose trip took rev_ms. */
static void exchange_us(master_t *m, int64_t fwd_us, int64_t rev_us) {
  mock_now_us += SYNC_INTERVAL * 1000;
  const int64_t t1 = master_time(m, now_ns());
  const uint16_t seq = m->seq++;
  mock_now_us += fwd_us;
  const int before = dreq_count;
  send_sync(m, seq);
  mock_now_us += 100;
  send_followup(m, seq, t1);
  mock_now_us += rev_us + fwd_us;
  if (dreq_count != before) {
    answer(m, rev_us);
  }
}

static void exchange(master_t *m, int fwd_ms, int rev_ms) {
  exchange_us(m, (int64_t)fwd_ms * 1000, (int64_t)rev_ms * 1000);
}

/* Typical Wi-Fi: a few ms of queuing, never less than zero. */
static int jitter_ms(int i) {
  static const int pattern[] = {3, 11, 1, 7, 18, 0, 5, 26, 2, 9};
  return pattern[i % 10];
}

/* Independent queuing in us, 0-2 ms, the same distribution both ways (the
 * argument is ignored; each call draws again). */
static int jitter_us(int i) {
  (void)i;
  static uint32_t lcg = 12345;
  lcg = lcg * 1664525u + 1013904223u;
  const int u = (int)((lcg >> 8) % 1000);
  return u * u / 500; /* skewed towards small, like queuing */
}

static void settle(master_t *m, int count) {
  for (int i = 0; i < count; i++) {
    exchange(m, 1 + jitter_ms(i) / 4, 1 + jitter_ms(i + 3) / 4);
  }
}

/* A congested network: every SYNC 70-400 ms late, both ways. */
static int congested_ms(int i) {
  static const int pattern[] = {70, 180, 95, 260, 120, 310, 85, 220, 150, 400};
  return pattern[i % 10];
}

static master_t master_a(void) {
  master_t m = {0};
  m.clock = MASTER_A;
  m.ip = IP_A;
  m.port = 1;
  m.answers = true;
  m.offset = TRUE_OFFSET;
  m.base_local = now_ns();
  m.log_interval = -3;
  return m;
}

static master_t master_b(void) {
  master_t m = master_a();
  m.clock = MASTER_B;
  m.ip = IP_B;
  m.offset = 912345LL * 1000000000LL;
  return m;
}

/* Clock error against the master at "now", in ns. */
static int64_t error_ns(const master_t *m) {
  return (int64_t)ptp_clock_get_time_ns() - master_time(m, now_ns());
}

static int64_t error_ms(const master_t *m) {
  return error_ns(m) / MS;
}

static void reset(void) {
  portENTER_CRITICAL(&ptp_state_mux);
  reset_all_state();
  portEXIT_CRITICAL(&ptp_state_mux);
  init_identity();
  ptp.event_socket = 3;
  mock_now_us += 60000000;
}

static bool two_way(void) {
  ptp_stats_t st;
  ptp_clock_get_stats(&st);
  return st.two_way;
}

/* ---------- arithmetic ---------- */

static void test_mul_q40(void) {
  printf("mul_q40 against 128-bit\n");
  uint64_t x = 88172645463325252ULL;
  for (int i = 0; i < 200000; i++) {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    const int64_t a = (int64_t)(x >> (i % 30)) * (i & 1 ? -1 : 1);
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    const int64_t q = (int64_t)(x >> 24) * (i & 2 ? -1 : 1); /* |q| < 2^40 */
    const __int128 p = (__int128)a * q;
    const __int128 mag = p < 0 ? -p : p;
    const __int128 r = (mag + ((__int128)1 << 39)) >> 40;
    const int64_t want = (int64_t)(p < 0 ? -r : r);
    if (mul_q40(a, q) != want) {
      CHECK(false, "mul_q40(%lld, %lld) = %lld, want %lld", (long long)a,
            (long long)q, (long long)mul_q40(a, q), (long long)want);
      return;
    }
  }
}

/* ---------- two-way basics ---------- */

static void test_delay_req_format(void) {
  printf("Delay_Req format\n");
  reset();
  master_t a = master_a();
  dreq_count = 0;
  exchange(&a, 1, 1);
  CHECK(dreq_count == 1, "%d Delay_Reqs for one SYNC", dreq_count);
  CHECK(dreq_ip == IP_A && dreq_port == 319, "sent to %08x:%u",
        (unsigned)dreq_ip, dreq_port);
  CHECK((dreq[0] & 0x0F) == PTP_MSG_DELAY_REQ && dreq[1] == 2 &&
            read_be(dreq + 2, 2) == 44 && dreq[4] == 0,
        "type/version/length/domain wrong");
  CHECK(read_be(dreq + 6, 2) == 0x0408, "flags %04x",
        (unsigned)read_be(dreq + 6, 2));
  const uint8_t eui[8] = {0x24, 0x0a, 0xc4, 0xff, 0xfe, 0x12, 0x34, 0x56};
  CHECK(memcmp(dreq + 20, eui, 8) == 0 && read_be(dreq + 28, 2) == 1,
        "clockIdentity is not the EUI-64 of the MAC, port 1");
  CHECK(dreq[32] == 1 && (int8_t)dreq[33] == -3, "control/logInterval wrong");
  const uint16_t first = (uint16_t)read_be(dreq + 30, 2);
  exchange(&a, 1, 1);
  CHECK((uint16_t)read_be(dreq + 30, 2) == (uint16_t)(first + 1),
        "sequenceId does not advance");
}

/* A symmetric path cancels: the clock lands on the master's time, not one
 * path delay below it as the passive v0.3.0 clock did. */
static void test_two_way_accuracy(void) {
  printf("two-way accuracy\n");
  reset();
  master_t a = master_a();
  for (int i = 0; i < 16; i++) {
    exchange(&a, 1, 1);
  }
  CHECK(ptp_clock_is_locked(), "not locked after 2 s");
  CHECK(two_way(), "not on two-way samples");
  for (int i = 0; i < 400; i++) {
    exchange_us(&a, 800 + jitter_us(i), 800 + jitter_us(i + 4));
  }
  CHECK(llabs(error_ns(&a)) < 100 * US, "error %lld us with a symmetric path",
        (long long)(error_ns(&a) / US));
  /* An asymmetric path leaves half the asymmetry. */
  reset();
  a = master_a();
  for (int i = 0; i < 400; i++) {
    exchange_us(&a, 1400, 600);
  }
  CHECK(llabs(error_ns(&a) + 400 * US) < 50 * US,
        "error %lld us, want -400 with 1.4/0.6 ms paths",
        (long long)(error_ns(&a) / US));
}

/* A drifting crystal (+-50 ppm) is followed with no lag: v0.3.0 lagged by
 * 32 us per ppm when the board's clock ran fast. */
static void test_drift_ramp(void) {
  printf("drift ramp\n");
  static const double ppms[] = {50, -50, 7.5};
  for (size_t k = 0; k < sizeof(ppms) / sizeof(ppms[0]); k++) {
    reset();
    master_t a = master_a();
    a.ppm = ppms[k];
    int64_t worst = 0, sum = 0;
    for (int i = 0; i < 1200; i++) {
      exchange_us(&a, 900 + jitter_us(i), 900 + jitter_us(i + 2));
      if (i >= 600) {
        sum += error_ns(&a);
        if (llabs(error_ns(&a)) > worst) {
          worst = llabs(error_ns(&a));
        }
      }
    }
    ptp_stats_t st;
    ptp_clock_get_stats(&st);
    CHECK(llabs(sum / 600) < 50 * US && worst < 200 * US,
          "%+.1f ppm: mean error %lld us, worst %lld us", ppms[k],
          (long long)(sum / 600 / US), (long long)(worst / US));
    CHECK(fabs(st.rate_ppb - ppms[k] * 1000) < 1500, "%+.1f ppm: rate %lld ppb",
          ppms[k], (long long)st.rate_ppb);
  }
}

/* Hours of timestamps near 1.7e18 ns keep full precision: no __int128, no
 * float on the conversion path. */
static void test_long_run_precision(void) {
  printf("three hours of exact samples\n");
  reset();
  master_t a = master_a();
  a.offset = 1763900000LL * 1000000000LL;
  a.ppm = 31.25;
  for (int i = 0; i < 3 * 3600 * 8; i++) {
    exchange_us(&a, 700, 700);
  }
  CHECK(llabs(error_ns(&a)) < 200, "error %lld ns after 3 h",
        (long long)error_ns(&a));
  uint64_t net, gm;
  int64_t back;
  const int64_t local = now_ns() + 5 * MS;
  CHECK(ptp_clock_local_to_network(local, &net, &gm) && gm == MASTER_A,
        "local_to_network failed");
  CHECK(ptp_clock_network_to_local(net, gm, &back) && llabs(back - local) <= 1,
        "round trip off by %lld ns", (long long)(back - local));
  CHECK(!ptp_clock_network_to_local(net, MASTER_B, &back),
        "converted a time on another GM's timeline");
}

static void test_delay_resp_matching(void) {
  printf("Delay_Resp matching\n");
  reset();
  master_t a = master_a();
  settle(&a, 8);
  const uint32_t got = ptp.dresp_received;
  /* Someone else's response (requestingPortIdentity) is not ours. */
  mock_now_us += SYNC_INTERVAL * 1000;
  send_sync(&a, a.seq);
  send_followup(&a, a.seq, master_time(&a, now_ns()));
  a.seq++;
  uint8_t p[54];
  header(p, sizeof(p), PTP_MSG_DELAY_RESP, a.clock, 1,
         (uint16_t)read_be(dreq + 30, 2), -3);
  put_ts(p + 34, master_time(&a, now_ns()));
  memcpy(p + 44, dreq + 20, 10);
  p[44] ^= 0xFF;
  handle_packet(p, sizeof(p), false, a.ip, 0);
  CHECK(ptp.dresp_received == got, "foreign Delay_Resp accepted");
  /* A response to an older request is superseded, but proves the master
   * answers. */
  p[44] ^= 0xFF;
  put_be(p + 30, read_be(dreq + 30, 2) - 1, 2);
  const uint32_t sup = ptp.superseded;
  handle_packet(p, sizeof(p), false, a.ip, 0);
  CHECK(ptp.superseded == sup + 1, "stale Delay_Resp not superseded");
  CHECK(ptp.x_active && !ptp.x_have_t4, "stale Delay_Resp completed the pair");
}

/* ---------- gates and steps ---------- */

/* The first exchanges after a reset may be badly delayed; the seed is the
 * least delayed of several, so one slow SYNC cannot gate out the good ones
 * after it. */
static void test_delayed_first_sample(void) {
  printf("delayed first sample\n");
  reset();
  master_t a = master_a();
  exchange(&a, 178, 2);
  for (int i = 0; i < 12; i++) {
    exchange(&a, 1, 1);
  }
  CHECK(llabs(error_ns(&a)) < MS, "offset %lld us off",
        (long long)(error_ns(&a) / US));
  CHECK(ptp_clock_is_locked(), "not locked after 1.5 s of good samples");
}

/* A burst of delayed SYNCs fails the delay gate; the lock and the offset
 * hold. */
static void test_delay_burst_ignored(void) {
  printf("delay burst\n");
  reset();
  master_t a = master_a();
  settle(&a, 40);
  const int64_t before = error_ns(&a);
  for (int i = 0; i < 15; i++) {
    exchange(&a, 80 + jitter_ms(i), 80);
    CHECK(ptp_clock_is_locked(), "lock lost on delayed sample %d", i);
  }
  CHECK(llabs(error_ns(&a) - before) < MS, "offset moved %lld us",
        (long long)((error_ns(&a) - before) / US));
  CHECK(ptp.delay_drops >= 15, "delayed samples not dropped by delay gate");
}

/* An iPhone's clock stops while it sleeps, so its time steps back.  The
 * round trip is normal and the midpoint is far off: after 24 SYNCs (Apple's
 * limit, ~3 s) the filter resets onto the new line, without ever reporting
 * itself unlocked.  v0.3.0 needed 16 SYNCs for this size of fall. */
static void test_backward_step_followed(void) {
  printf("backward step\n");
  reset();
  master_t a = master_a();
  settle(&a, 40);
  a.offset -= 4445 * MS;
  for (int i = 0; i < PTP_PPM_DROP_LIMIT + 8; i++) {
    exchange(&a, 1 + jitter_ms(i) / 4, 1);
    CHECK(ptp_clock_is_locked(), "unlocked on sample %d after the step", i);
  }
  CHECK(llabs(error_ms(&a)) <= 1, "offset %lld ms off after the step",
        (long long)error_ms(&a));
}

/* The same step measured through congestion: the congested SYNCs fail the
 * delay gate and count for nothing; once the round trip is normal again the
 * step is followed after 24 SYNCs. */
static void test_step_through_congestion(void) {
  printf("step through congestion\n");
  reset();
  master_t a = master_a();
  settle(&a, 40);
  a.offset -= 4445 * MS;
  for (int i = 0; i < 24; i++) {
    exchange(&a, congested_ms(i), congested_ms(i + 3));
    CHECK(ptp_clock_is_locked(), "unlocked on congested sample %d", i);
  }
  settle(&a, PTP_PPM_DROP_LIMIT + 8);
  CHECK(ptp_clock_is_locked(), "unlocked after the step");
  CHECK(llabs(error_ms(&a)) <= 1, "offset %lld ms off after the step",
        (long long)error_ms(&a));
}

/* Congestion is not the master's clock moving.  Ten seconds of it leave the
 * offset where it was and the lock held (the master is still sending). */
static void test_congestion_not_followed(void) {
  printf("congestion\n");
  reset();
  master_t a = master_a();
  settle(&a, 40);
  const int64_t before = error_ns(&a);
  for (int i = 0; i < 70; i++) {
    exchange(&a, congested_ms(i), congested_ms(i + 5));
    CHECK(ptp_clock_is_locked(), "unlocked on congested sample %d", i);
  }
  CHECK(llabs(error_ns(&a) - before) < MS,
        "offset moved %lld us in 9 s of congestion",
        (long long)((error_ns(&a) - before) / US));
  settle(&a, 8);
  CHECK(llabs(error_ms(&a)) <= 1, "offset %lld ms off after it",
        (long long)error_ms(&a));
}

/* A fall under 500 ms used to take 160 SYNCs (20 s) to follow, because
 * without a round trip it looked like congestion.  It is now followed like
 * any step, after 24. */
static void test_small_step_followed(void) {
  printf("small step\n");
  reset();
  master_t a = master_a();
  settle(&a, 40);
  a.offset -= 300 * MS;
  for (int i = 0; i < PTP_PPM_DROP_LIMIT + 8; i++) {
    exchange(&a, 1, 1);
    CHECK(ptp_clock_is_locked(), "unlocked on sample %d after the step", i);
  }
  CHECK(llabs(error_ms(&a)) <= 1, "offset %lld ms off after the step",
        (long long)error_ms(&a));
}

/* A forward step is gated like any other (v0.3.0 took it at once): a single
 * early sample does not move the clock, a persistent one does after 24. */
static void test_forward_step_followed(void) {
  printf("forward step\n");
  reset();
  master_t a = master_a();
  settle(&a, 40);
  a.offset += 2000 * MS;
  exchange(&a, 1, 1);
  CHECK(llabs(error_ms(&a) + 2000) <= 1, "one sample moved the clock");
  for (int i = 0; i < PTP_PPM_DROP_LIMIT + 8; i++) {
    exchange(&a, 1, 1);
  }
  CHECK(llabs(error_ms(&a)) <= 1, "offset %lld ms off after the step",
        (long long)error_ms(&a));
  CHECK(ptp_clock_is_locked(), "unlocked by a forward step");
}

/* A long pause with stale samples re-seeds on the next sample, without
 * dropping the lock; one later delayed SYNC cannot move the resumed clock. */
static void test_resume_after_pause(void) {
  printf("resume after pause\n");
  reset();
  master_t a = master_a();
  settle(&a, 40);
  for (int i = 0; i < 15; i++) {
    exchange(&a, 600, 600);
  }
  ptp_clock_notify_resume(30000);
  settle(&a, 8);
  const int64_t before = error_ns(&a);
  exchange(&a, 700, 1);
  CHECK(llabs(error_ns(&a) - before) < MS,
        "one delayed sample after resume moved offset by %lld us",
        (long long)((error_ns(&a) - before) / US));
  CHECK(ptp_clock_is_locked(), "resume or delayed sample dropped the lock");
}

/* ---------- passive fallback ---------- */

static void test_passive_fallback(void) {
  printf("passive fallback\n");
  reset();
  master_t a = master_a();
  a.answers = false;
  for (int i = 0; i < 24; i++) {
    exchange(&a, 1, 1);
  }
  CHECK(ptp.mode == PTP_MODE_PASSIVE, "not passive after 24 silent SYNCs");
  CHECK(ptp_clock_is_locked(), "passive clock not locked");
  CHECK(!two_way(), "reports two-way");
  const int before = dreq_count;
  for (int i = 0; i < 160; i++) {
    exchange_us(&a, 500 + jitter_us(i), 500);
  }
  CHECK(dreq_count - before >= 1 && dreq_count - before <= 3,
        "%d Delay_Reqs in 20 s of passive mode", dreq_count - before);
  /* With nothing learnt, the prior covers a typical one-way delay. */
  CHECK(llabs(error_ns(&a)) < 300 * US, "passive error %lld us",
        (long long)(error_ns(&a) / US));
  /* The master starts answering: the next retry brings two-way back. */
  a.answers = true;
  for (int i = 0; i < 160; i++) {
    exchange_us(&a, 500 + jitter_us(i), 500 + jitter_us(i + 3));
  }
  CHECK(ptp.mode == PTP_MODE_TWO_WAY && two_way(), "two-way not restored");
  CHECK(llabs(error_ns(&a)) < 300 * US, "error %lld us after returning",
        (long long)(error_ns(&a) / US));
}

/* A passive backward step larger than congestion is followed after ~16
 * SYNCs, as in v0.3.0. */
static void test_passive_step(void) {
  printf("passive step\n");
  reset();
  master_t a = master_a();
  a.answers = false;
  for (int i = 0; i < 120; i++) {
    exchange(&a, 1, 1);
  }
  a.offset -= 4445 * MS;
  for (int i = 0; i < 48; i++) {
    exchange(&a, 1 + jitter_ms(i) / 8, 1);
    CHECK(ptp_clock_is_locked(), "unlocked on sample %d after the step", i);
  }
  CHECK(llabs(error_ms(&a)) <= 2, "offset %lld ms off after the step",
        (long long)error_ms(&a));
}

/* ---------- grandmaster and ports ---------- */

static void test_bmca_and_gm_change(void) {
  printf("BMCA and GM change\n");
  reset();
  master_t a = master_a(), b = master_b();
  send_announce(&a, MASTER_A, 248, 0);
  send_announce(&b, MASTER_B, 16, 0); /* prefers itself */
  for (int i = 0; i < 16; i++) {
    exchange(&a, 1, 1);
    exchange(&b, 1, 1);
  }
  CHECK(ptp_clock_get_grandmaster_id() == MASTER_B,
        "BMCA chose %016llx, not the priority-16 master",
        (unsigned long long)ptp_clock_get_grandmaster_id());
  /* The anchor names A: the clock moves to A's timeline, and times on B's
   * no longer convert. */
  uint64_t net = 0, gm = 0;
  CHECK(ptp_clock_local_to_network(now_ns(), &net, &gm) && gm == MASTER_B,
        "not locked on B");
  ptp_clock_set_master_clock_id(MASTER_A);
  int64_t local;
  CHECK(!ptp_clock_network_to_local(net, MASTER_B, &local),
        "old timeline still converts after the GM change");
  for (int i = 0; i < 16; i++) {
    exchange(&a, 1, 1);
    exchange(&b, 1, 1);
  }
  CHECK(ptp_clock_get_grandmaster_id() == MASTER_A && ptp_clock_is_locked(),
        "not locked on the anchor's GM");
  CHECK(llabs(error_ms(&a)) <= 1, "offset %lld ms off A",
        (long long)error_ms(&a));
  CHECK(ptp.rejected_master_count > 0, "B's SYNCs not counted as rejected");
}

/* The GM's own port is preferred; a boundary port relaying it is the
 * fallback when the direct one goes quiet, without a reset of the
 * timeline. */
static void test_relayed_port(void) {
  printf("relayed port\n");
  reset();
  master_t a = master_a();
  master_t r = master_a(); /* same timeline, relayed */
  r.clock = MASTER_B;
  r.ip = IP_R;
  r.apple_gm = MASTER_A;
  ptp_clock_set_master_clock_id(MASTER_A);
  send_announce(&r, MASTER_A, 248, 1);
  for (int i = 0; i < 40; i++) {
    exchange(&a, 1, 1);
    exchange(&r, 1, 1);
  }
  CHECK(ptp.selected >= 0 && ptp.ports[ptp.selected].ip == IP_A,
        "direct port not preferred");
  for (int i = 0; i < 80; i++) {
    exchange(&r, 1, 1);
    if (i % 8 == 0) {
      send_announce(&r, MASTER_A, 248, 1);
    }
    CHECK(ptp_clock_is_locked(), "unlocked when the direct port went quiet");
  }
  CHECK(ptp.ports[ptp.selected].ip == IP_R, "relay not used");
  CHECK(llabs(error_ms(&a)) <= 1, "offset %lld ms off through the relay",
        (long long)error_ms(&a));
  /* Without an Announce, the Apple Follow_Up TLV alone names the GM. */
  reset();
  r.seq = 0;
  ptp_clock_set_master_clock_id(MASTER_A);
  for (int i = 0; i < 24; i++) {
    exchange(&r, 1, 1);
  }
  CHECK(ptp_clock_is_locked() && ptp_clock_get_grandmaster_id() == MASTER_A,
        "relay named by the Apple TLV not accepted");
}

/* A gmTimeBaseIndicator change is a discontinuity: the filter re-seeds.  A
 * sender without the 802.1AS TLV is still accepted (Apple would discard
 * it). */
static void test_time_base_and_tlv(void) {
  printf("gmTimeBaseIndicator / no 802.1AS TLV\n");
  reset();
  master_t a = master_a();
  a.tlv8021as = true;
  settle(&a, 24);
  CHECK(ptp_clock_is_locked(), "not locked with the 802.1AS TLV");
  a.time_base++;
  mock_now_us += SYNC_INTERVAL * 1000;
  send_sync(&a, a.seq);
  send_followup(&a, a.seq, master_time(&a, now_ns()));
  a.seq++;
  CHECK(!ptp.tw.seeded, "time base change did not reset the filter");
  CHECK(ptp_clock_is_locked(), "time base change dropped the lock");
  settle(&a, 16);
  CHECK(ptp.tw.seeded && llabs(error_ms(&a)) <= 1, "not re-seeded");
  reset();
  master_t b = master_a();
  settle(&b, 24);
  CHECK(ptp_clock_is_locked(), "not locked without the 802.1AS TLV");
}

static void test_peers(void) {
  printf("timing peers\n");
  reset();
  master_t a = master_a();
  const uint32_t others[] = {IP_B, IP_R};
  ptp_clock_set_peers(others, 2);
  settle(&a, 24);
  CHECK(!ptp_clock_is_locked(), "locked to a master outside the peer list");
  const uint32_t mine[] = {IP_B, IP_A};
  ptp_clock_set_peers(mine, 2);
  settle(&a, 24);
  CHECK(ptp_clock_is_locked(), "peer in the list not accepted");
  ptp_clock_set_peers(NULL, 0);
  CHECK(ptp.peer_count == 0, "peer list not cleared");
}

/* ---------- FOLLOW_UP pairing and sessions ---------- */

/* Sequence numbers are only unique within one source port.  A same-sequence
 * FOLLOW_UP from another port must not pair with a SYNC. */
static void test_followup_source_identity(void) {
  printf("FOLLOW_UP source identity\n");
  reset();
  master_t a = master_a(), b = master_b();
  b.ip = IP_A; /* same host, other clock */
  settle(&a, 24);
  const uint32_t fu = ptp.followup_count;
  const uint16_t seq = a.seq++;
  mock_now_us += SYNC_INTERVAL * 1000;
  send_sync(&a, seq);
  send_followup(&b, seq, master_time(&a, now_ns()) + 100 * MS);
  CHECK(ptp.followup_count == fu, "another clock's FOLLOW_UP paired");
  master_t a2 = a;
  a2.port = 2;
  send_followup(&a2, seq, master_time(&a, now_ns()) + 100 * MS);
  CHECK(ptp.followup_count == fu, "another port's FOLLOW_UP paired");
  send_followup(&a, (uint16_t)(seq + 1), master_time(&a, now_ns()));
  CHECK(ptp.followup_count == fu, "wrong sequence paired");
  send_followup(&a, seq, master_time(&a, now_ns()));
  CHECK(ptp.followup_count == fu + 1, "matching FOLLOW_UP rejected");
  send_followup(&a, seq, master_time(&a, now_ns()));
  CHECK(ptp.followup_count == fu + 1, "duplicate FOLLOW_UP counted");
  CHECK(llabs(error_ms(&a)) <= 1, "offset moved");
  /* Sequence wrap is valid. */
  a.seq = UINT16_MAX - 4;
  settle(&a, 10);
  CHECK(ptp_clock_is_locked() && llabs(error_ms(&a)) <= 1,
        "sequence wrap broke the clock");
}

/* A reset or a switch to another master invalidates any pending pair. */
static void test_followup_invalidation(void) {
  printf("FOLLOW_UP invalidation\n");
  reset();
  master_t a = master_a();
  ptp_clock_set_master_clock_id(MASTER_A);
  settle(&a, 24);
  mock_now_us += SYNC_INTERVAL * 1000;
  send_sync(&a, a.seq);
  ptp_clock_clear();
  const uint32_t fu = ptp.followup_count;
  send_followup(&a, a.seq, master_time(&a, now_ns()));
  CHECK(ptp.followup_count == fu, "clear retained a pending SYNC");
  a.seq++;

  ptp_clock_set_master_clock_id(MASTER_A);
  settle(&a, 24);
  mock_now_us += SYNC_INTERVAL * 1000;
  send_sync(&a, a.seq);
  ptp_clock_set_master_clock_id(MASTER_B);
  const uint32_t fu2 = ptp.followup_count;
  send_followup(&a, a.seq, master_time(&a, now_ns()));
  CHECK(ptp.followup_count == fu2, "master change retained a pending SYNC");
  CHECK(!ptp_clock_is_locked(), "locked on a master the anchor did not name");
}

/* Freshness belongs to the SYNC receive time, not a delayed FOLLOW_UP or
 * Delay_Resp.  Test both retention and lock timeout boundaries while the
 * millisecond tick counter wraps between the packets. */
static void test_followup_does_not_refresh_old_sample(void) {
  printf("FOLLOW_UP sample freshness\n");
  static const uint32_t delays[] = {SESSION_KEEP_MS - 10, SESSION_KEEP_MS,
                                    LOCK_TIMEOUT_MS + 1};
  for (size_t i = 0; i < sizeof(delays) / sizeof(delays[0]); i++) {
    reset();
    master_t a = master_a();
    mock_now_us = ((int64_t)UINT32_MAX - 1000 - 40 * SYNC_INTERVAL) * 1000;
    a.base_local = now_ns();
    ptp_clock_set_master_clock_id(MASTER_A);
    settle(&a, 40);
    CHECK(ptp_clock_is_locked(), "failed to lock before freshness test");
    ptp_clock_end_session();
    const uint16_t seq = a.seq++;
    mock_now_us += SYNC_INTERVAL * 1000;
    const int64_t t1 = master_time(&a, now_ns());
    const int before = dreq_count;
    send_sync(&a, seq);
    mock_now_us += (int64_t)delays[i] * 1000;
    send_followup(&a, seq, t1);
    if (dreq_count != before) {
      answer(&a, 1000);
    }
    if (delays[i] > LOCK_TIMEOUT_MS) {
      CHECK(!ptp_clock_is_locked(), "late FOLLOW_UP revived an expired lock");
    }
    ptp_clock_set_master_clock_id(MASTER_A);
    CHECK(ptp_clock_is_locked() == (delays[i] < SESSION_KEEP_MS),
          "same-master session with a %u ms old SYNC retained the wrong state",
          (unsigned)delays[i]);
  }
}

/* ESP_LOGI can yield to the PTP task. Once master selection reaches its log,
 * a foreign packet must already be excluded from the lock it decided to keep.
 */
static void foreign_packet_during_master_log(void) {
  master_t b = master_b();
  mock_now_us += 1000;
  send_sync(&b, 50);
}

static void test_master_selection_log_yield(void) {
  printf("master selection log yield\n");
  reset();
  master_t a = master_a();
  settle(&a, 40);
  const int64_t before = error_ns(&a);
  const uint32_t rejected = ptp.rejected_master_count;
  log_hook = foreign_packet_during_master_log;
  ptp_clock_set_master_clock_id(MASTER_A);
  CHECK(log_hook == NULL, "master selection did not reach its log");
  CHECK(ptp.rejected_master_count == rejected + 1,
        "foreign packet was accepted during the master-selection log");
  CHECK(ptp_clock_get_grandmaster_id() == MASTER_A && ptp_clock_is_locked() &&
            llabs(error_ns(&a) - before) < 10 * US,
        "master selection lost the lock or moved the clock");
}

static void *packet_worker(void *unused) {
  (void)unused;
  master_t a = master_a(), b = master_b();
  for (int i = 0; i < 3000; i++) {
    master_t *m = i % 2 ? &a : &b;
    const uint16_t seq = m->seq++;
    const int64_t t1 = master_time(m, now_ns());
    send_sync(m, seq);
    send_followup(m, seq, t1);
    answer(m, 0);
    if (i % 50 == 0) {
      send_announce(m, m->clock, (uint8_t)(i % 3 ? 248 : 16), 0);
    }
  }
  return NULL;
}

/* Exercise real host mutual exclusion while packets, session transitions and
 * diagnostic/audio getters run concurrently. Mock time stays fixed so this
 * isolates shared clock state from the test clock itself. */
static void test_concurrent_packets_and_sessions(void) {
  printf("concurrent packets and sessions\n");
  reset();
  pthread_t sender;
  int err = pthread_create(&sender, NULL, packet_worker, NULL);
  CHECK(err == 0, "failed to start packet worker: %d", err);
  if (err != 0) {
    return;
  }
  for (unsigned i = 0; i < 6000; i++) {
    if (i % 37 == 0) {
      ptp_clock_clear();
    }
    ptp_clock_set_master_clock_id(i % 2 ? MASTER_A : MASTER_B);
    if (i % 3 == 0) {
      ptp_clock_end_session();
    }
    if (i % 7 == 0) {
      ptp_clock_notify_resume(30000);
    }
    if (i % 11 == 0) {
      const uint32_t peers[] = {IP_A};
      ptp_clock_set_peers(peers, i % 22 ? 1 : 0);
    }
    ptp_stats_t stats;
    ptp_clock_get_stats(&stats);
    uint64_t net, gm;
    int64_t local;
    (void)ptp_clock_get_offset_ns();
    (void)ptp_clock_get_time_ns();
    (void)ptp_clock_is_locked();
    (void)ptp_clock_get_master_clock_id();
    (void)ptp_clock_local_to_network(now_ns(), &net, &gm);
    (void)ptp_clock_network_to_local(net, gm, &local);
    portENTER_CRITICAL(&ptp_state_mux);
    CHECK(ptp.expected_clock_id == 0 || ptp.gm_id == 0 ||
              ptp.gm_id == ptp.expected_clock_id,
          "clock on a GM the anchor did not name");
    portEXIT_CRITICAL(&ptp_state_mux);
  }
  CHECK(pthread_join(sender, NULL) == 0, "failed to join packet worker");
}

/* The next track from the same phone keeps the lock; anything else starts
 * over. */
static void test_session_handover(void) {
  printf("session handover\n");

  reset();
  master_t a = master_a();
  ptp_clock_set_master_clock_id(MASTER_A);
  settle(&a, 40);
  ptp_clock_end_session();
  mock_now_us += 400000;
  ptp_clock_set_master_clock_id(MASTER_A);
  CHECK(ptp_clock_is_locked(), "same master, current samples: lock dropped");

  reset();
  a = master_a();
  settle(&a, 40);
  ptp_clock_end_session();
  ptp_clock_set_master_clock_id(MASTER_B);
  CHECK(!ptp_clock_is_locked(), "different master: lock kept");

  reset();
  a = master_a();
  settle(&a, 40);
  ptp_clock_end_session();
  mock_now_us += (SESSION_KEEP_MS + 100) * 1000;
  ptp_clock_set_master_clock_id(MASTER_A);
  CHECK(!ptp_clock_is_locked(), "no samples for 2 s: lock kept");

  reset();
  a = master_a();
  settle(&a, 40);
  ptp_clock_end_session();
  a.offset -= 4445 * MS;
  exchange(&a, 1, 1);
  ptp_clock_set_master_clock_id(MASTER_A);
  CHECK(!ptp_clock_is_locked(), "master's clock just moved: lock kept");

  /* Another master on the network keeps sending between the two sessions;
   * the ended session's filter still turns it away. */
  reset();
  a = master_a();
  master_t b = master_b();
  ptp_clock_set_master_clock_id(MASTER_A);
  settle(&a, 40);
  ptp_clock_end_session();
  const uint32_t rejected = ptp.rejected_master_count;
  exchange(&b, 1, 1);
  CHECK(ptp.rejected_master_count == rejected + 1,
        "other master's SYNC let in between sessions");
  CHECK(ptp_clock_get_grandmaster_id() == MASTER_A,
        "GM changed between sessions");
  mock_now_us += 400000;
  ptp_clock_set_master_clock_id(MASTER_A);
  CHECK(ptp_clock_is_locked(), "other master between sessions: lock dropped");
}

/* ---------- lwIP input hook ---------- */

static size_t ip_packet(uint8_t *out, uint32_t src, uint16_t dport,
                        const uint8_t *msg, size_t len, uint16_t frag) {
  memset(out, 0, 28);
  out[0] = 0x45;
  put_be(out + 2, 28 + len, 2);
  put_be(out + 6, frag, 2);
  out[8] = 64;
  out[9] = 17;
  memcpy(out + 12, &src, 4);
  put_be(out + 20, 319, 2);
  put_be(out + 22, dport, 2);
  put_be(out + 24, 8 + len, 2);
  memcpy(out + 28, msg, len);
  return 28 + len;
}

static void test_rx_hook(void) {
  printf("lwIP RX timestamp hook\n");
  memset(rx_ring, 0, sizeof(rx_ring));
  uint8_t sync[44], pkt[128];
  header(sync, sizeof(sync), PTP_MSG_SYNC, MASTER_A, 1, 777, -3);
  const int64_t stamped = mock_now_us;
  rx_stamp_ip4(pkt, ip_packet(pkt, IP_A, 319, sync, sizeof(sync), 0), stamped);
  mock_now_us += 2500; /* the task gets to it 2.5 ms later */
  bool hook;
  int64_t t = rx_timestamp_ns(sync, sizeof(sync), IP_A, now_ns(), &hook);
  CHECK(hook && t == stamped * 1000, "hook stamp not found");
  t = rx_timestamp_ns(sync, sizeof(sync), IP_B, now_ns(), &hook);
  CHECK(!hook && t == now_ns(), "stamp matched another source");
  header(sync, sizeof(sync), PTP_MSG_SYNC, MASTER_A, 1, 778, -3);
  t = rx_timestamp_ns(sync, sizeof(sync), IP_A, now_ns(), &hook);
  CHECK(!hook, "stamp matched another sequence");
  /* Not a SYNC to 319, or a fragment: not recorded. */
  rx_stamp_ip4(pkt, ip_packet(pkt, IP_A, 320, sync, sizeof(sync), 0),
               mock_now_us);
  rx_stamp_ip4(pkt, ip_packet(pkt, IP_A, 319, sync, sizeof(sync), 0x2000),
               mock_now_us);
  t = rx_timestamp_ns(sync, sizeof(sync), IP_A, now_ns(), &hook);
  CHECK(!hook, "port 320 or a fragment was stamped");
  /* A stamp from 2^16 SYNCs ago is not this packet. */
  rx_stamp_ip4(pkt, ip_packet(pkt, IP_A, 319, sync, sizeof(sync), 0),
               mock_now_us);
  mock_now_us += 2000000;
  t = rx_timestamp_ns(sync, sizeof(sync), IP_A, now_ns(), &hook);
  CHECK(!hook, "an old stamp matched");
  /* The ring keeps the last 16. */
  for (int i = 0; i < PTP_RX_RING + 4; i++) {
    header(sync, sizeof(sync), PTP_MSG_SYNC, MASTER_A, 1, (uint16_t)(900 + i),
           -3);
    rx_stamp_ip4(pkt, ip_packet(pkt, IP_A, 319, sync, sizeof(sync), 0),
                 mock_now_us + i);
  }
  t = rx_timestamp_ns(sync, sizeof(sync), IP_A, now_ns() + 1000000, &hook);
  CHECK(hook && t == (mock_now_us + PTP_RX_RING + 3) * 1000,
        "newest stamp lost");
}

int main(void) {
  test_mul_q40();
  test_delay_req_format();
  test_two_way_accuracy();
  test_drift_ramp();
  test_long_run_precision();
  test_delay_resp_matching();
  test_delayed_first_sample();
  test_delay_burst_ignored();
  test_backward_step_followed();
  test_step_through_congestion();
  test_congestion_not_followed();
  test_small_step_followed();
  test_forward_step_followed();
  test_resume_after_pause();
  test_passive_fallback();
  test_passive_step();
  test_bmca_and_gm_change();
  test_relayed_port();
  test_time_base_and_tlv();
  test_peers();
  test_followup_source_identity();
  test_followup_invalidation();
  test_followup_does_not_refresh_old_sample();
  test_master_selection_log_yield();
  test_concurrent_packets_and_sessions();
  test_session_handover();
  test_rx_hook();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}

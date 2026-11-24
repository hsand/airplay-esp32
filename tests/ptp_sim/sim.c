// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Seeded network simulator for main/network/ptp_clock.c.
 *
 * The same scenario drives either the v0.3.0 clock (built with
 * -DSIM_BASELINE, from baseline/) or the current one.  It models the true
 * time, the masters' PTP timelines, the board's drifting crystal, both path
 * directions (base + exponential + spikes + congestion bursts + asymmetry +
 * loss, FIFO per path), the lwIP input and task latencies on receive, and
 * masters that answer Delay_Req or not.  Scripted events: a phone that
 * sleeps (its clock stops, so it steps back), a grandmaster change to an
 * unrelated timeline, a direct port that disappears behind a boundary relay,
 * and an unrelated better-priority master on the network.
 *
 * Every 20 ms it reads the clock at "now" and compares it with the true
 * network time of the grandmaster the anchor names; it prints one line of
 * metrics.  run.py runs the matrix and prints the table. */

#include "mocks.h"

#ifdef SIM_BASELINE
#include "baseline/ptp_clock.c"
#else
#include "ptp_clock.c"
#endif

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NS  1LL
#define US  1000LL
#define MS  1000000LL
#define SEC 1000000000LL

int64_t sim_local_us;
static bool verbose;

/* ---------- RNG ---------- */

/* Separate streams, so the board's Delay_Reqs (which only the new clock
 * sends) do not change what the master -> board path does. */
enum { R_MISC, R_FWD, R_REV, R_RX, R_RELAY, R_COUNT };
static uint64_t rng[R_COUNT];

static double urand_s(int s) {
  uint64_t *r = &rng[s];
  *r ^= *r >> 12;
  *r ^= *r << 25;
  *r ^= *r >> 27;
  return (double)((*r * 2685821657736338717ULL) >> 11) / 9007199254740992.0;
}

static double exprand_s(int s, double mean) {
  return -mean * log(1.0 - urand_s(s));
}

#define urand()       urand_s(R_MISC)
#define exprand(mean) exprand_s(R_MISC, mean)

/* ---------- scenario ---------- */

typedef struct {
  const char *name;
  double drift_ppm;
  double wander_ppm;    /* temperature: sinusoid amplitude */
  double wander_period; /* s */
  double base_us[2];    /* [0] master -> board, [1] board -> master */
  double exp_us[2];
  double spike_p; /* per packet, 5-40 ms */
  bool congestion;
  double loss;
  double task_exp_us; /* RX task latency */
  bool answers;
  double duration;
  double sleep_at, sleep_len; /* phone sleep on GM A */
  double gm_change_at;        /* anchor switches to B */
  double relay_drop_at;       /* A's direct port stops; relay R carries A */
  bool multi;                 /* unrelated better-priority master C */
  bool hook;
  bool tlv8021as;
  double storm_at, storm_len; /* congestion storm: late/lost Delay_Resps */
} scenario_t;

static scenario_t sc;

/* ---------- truth and local clock ---------- */

static int64_t now_T; /* true ns since start */
static double local_ns;

static double drift_ppm(int64_t t) {
  return sc.drift_ppm +
         sc.wander_ppm * sin(2 * M_PI * (double)t / (sc.wander_period * SEC));
}

static void advance_to(int64_t t) {
  if (t < now_T) {
    abort();
  }
  local_ns += (double)(t - now_T) * (1.0 + drift_ppm(now_T) * 1e-6);
  now_T = t;
  sim_local_us = (int64_t)floor(local_ns / 1000.0);
}

/* ---------- masters ---------- */

enum { M_A, M_B, M_C, M_R, M_COUNT };

typedef struct {
  bool present;
  uint64_t clock_id;
  uint32_t ip; /* network order */
  int64_t p0;  /* PTP ns at T = 0 */
  double rate_ppm;
  int64_t step; /* added after a sleep */
  bool answers;
  double on_from, off_from; /* s; off_from < 0: never */
  int relay_of;             /* -1 or the master whose time it relays */
  uint8_t p1, p2;
  uint16_t steps;
  uint16_t sync_seq, ann_seq;
  int64_t last_arrival[2]; /* FIFO per path direction */
  double relay_wander;
} master_t;

static master_t ms[M_COUNT];
static const uint32_t esp_ip __attribute__((unused)) = 0x3201a8c0;

static int64_t master_time(int m, int64_t t) {
  const master_t *x = &ms[m];
  if (x->relay_of >= 0) {
    /* A boundary clock's own sync error: slow wander plus a little noise */
    const double e = 20e3 * sin(2 * M_PI * (double)t / (97.0 * SEC)) +
                     (urand_s(R_RELAY) - 0.5) * 10e3;
    return master_time(x->relay_of, t) + (int64_t)e;
  }
  return x->p0 + t + (int64_t)((double)t * x->rate_ppm * 1e-6) + x->step;
}

static bool master_asleep(int m, int64_t t) {
  const master_t *x = &ms[m];
  if (m == M_A && sc.sleep_len > 0 && t >= (int64_t)(sc.sleep_at * SEC) &&
      t < (int64_t)((sc.sleep_at + sc.sleep_len) * SEC)) {
    return true;
  }
  if (t < (int64_t)(x->on_from * SEC)) {
    return true;
  }
  return x->off_from >= 0 && t >= (int64_t)(x->off_from * SEC);
}

/* ---------- network ---------- */

typedef struct {
  double start, end, level_us;
} burst_t;
static burst_t bursts[64];
static int nbursts;

static double congestion_us(int64_t t) {
  const double s = (double)t / SEC;
  for (int i = 0; i < nbursts; i++) {
    if (s >= bursts[i].start && s < bursts[i].end) {
      return bursts[i].level_us;
    }
  }
  return 0;
}

static bool in_storm(int64_t t) {
  const double s = (double)t / SEC;
  return sc.storm_len > 0 && s >= sc.storm_at && s < sc.storm_at + sc.storm_len;
}

static int64_t path_delay(int dir, int r) {
  double d = sc.base_us[dir] + exprand_s(r, sc.exp_us[dir]);
  if (in_storm(now_T)) {
    /* What a real burst looked like: retransmit round trips of ~40 ms, so
     * every packet waits 20-60 ms, differently each way. */
    d += 20000 + urand_s(r) * 40000;
  }
  if (urand_s(r) < sc.spike_p) {
    d += 5000 + urand_s(r) * 35000;
  }
  const double c = congestion_us(now_T);
  if (c > 0) {
    d += c * (0.5 + 0.5 * urand_s(r));
  }
  return (int64_t)(d * US);
}

/* ---------- events ---------- */

enum {
  EV_SYNC_TX,
  EV_ANN_TX,
  EV_HOOK,
  EV_TASK,
  EV_DREQ_AT_MASTER,
  EV_SAMPLE,
  EV_ANCHOR,
};

typedef struct {
  int64_t t;
  uint64_t order;
  int type;
  int m;
  int sock; /* 0 = event (319), 1 = general (320) */
  size_t len;
  uint8_t buf[128];
  uint64_t arg;
} event_t;

static event_t *heap;
static size_t heap_n, heap_cap;
static uint64_t event_order;

static bool ev_less(const event_t *a, const event_t *b) {
  return a->t < b->t || (a->t == b->t && a->order < b->order);
}

static void push(event_t e) {
  if (heap_n == heap_cap) {
    heap_cap = heap_cap ? heap_cap * 2 : 1024;
    heap = realloc(heap, heap_cap * sizeof(*heap));
  }
  e.order = event_order++;
  size_t i = heap_n++;
  heap[i] = e;
  while (i > 0) {
    size_t p = (i - 1) / 2;
    if (!ev_less(&heap[i], &heap[p])) {
      break;
    }
    event_t tmp = heap[i];
    heap[i] = heap[p];
    heap[p] = tmp;
    i = p;
  }
}

static event_t pop(void) {
  event_t top = heap[0];
  heap[0] = heap[--heap_n];
  size_t i = 0;
  for (;;) {
    size_t l = 2 * i + 1, r = l + 1, m = i;
    if (l < heap_n && ev_less(&heap[l], &heap[m])) {
      m = l;
    }
    if (r < heap_n && ev_less(&heap[r], &heap[m])) {
      m = r;
    }
    if (m == i) {
      break;
    }
    event_t tmp = heap[i];
    heap[i] = heap[m];
    heap[m] = tmp;
    i = m;
  }
  return top;
}

/* One PTP task reads both sockets, in the order the tcpip thread queued
 * the packets. */
static int64_t last_task, last_hook;

/* A packet from master m leaves now: schedule its trip to the board. */
static void send_to_board(int m, int sock, const uint8_t *buf, size_t len,
                          int64_t after) {
  const int r =
      sock == 1 && buf[0] == (0x10 | PTP_MSG_DELAY_RESP) ? R_REV : R_FWD;
  if (urand_s(r) < sc.loss) {
    return;
  }
  master_t *x = &ms[m];
  int64_t arrival = now_T + after + path_delay(0, r);
  if (arrival <= x->last_arrival[0]) {
    arrival = x->last_arrival[0] + 20 * US;
  }
  x->last_arrival[0] = arrival;
  event_t e = {0};
  /* to the tcpip thread, which takes packets in order */
  e.t = arrival + (int64_t)((20 + exprand_s(R_RX, 40)) * US);
  if (e.t <= last_hook) {
    e.t = last_hook + 2 * US;
  }
  last_hook = e.t;
  e.type = EV_HOOK;
  e.m = m;
  e.sock = sock;
  e.len = len;
  memcpy(e.buf, buf, len);
  push(e);
}

static void put_be(uint8_t *p, uint64_t v, int n) {
  for (int i = n - 1; i >= 0; i--) {
    p[i] = (uint8_t)v;
    v >>= 8;
  }
}

static void put_ts(uint8_t *p, int64_t ns) {
  put_be(p, (uint64_t)ns / SEC, 6);
  put_be(p + 6, (uint64_t)ns % SEC, 4);
}

static void header(uint8_t *p, int m, uint8_t type, uint16_t len, uint16_t seq,
                   uint8_t control) {
  memset(p, 0, len);
  p[0] = (uint8_t)(0x10 | type); /* transportSpecific 1, as gPTP */
  p[1] = 2;
  put_be(p + 2, len, 2);
  put_be(p + 20, ms[m].clock_id, 8);
  put_be(p + 28, 1, 2);
  put_be(p + 30, seq, 2);
  p[32] = control;
  p[33] = (uint8_t)(int8_t)(type == PTP_MSG_ANNOUNCE ? 0 : -3);
}

static uint64_t gm_of(int m) {
  return ms[m].relay_of >= 0 ? ms[ms[m].relay_of].clock_id : ms[m].clock_id;
}

static void sync_tx(int m) {
  master_t *x = &ms[m];
  const uint16_t seq = x->sync_seq++;
  if (master_asleep(m, now_T)) {
    return;
  }
  uint8_t sync[44];
  header(sync, m, PTP_MSG_SYNC, 44, seq, 0);
  sync[6] = 0x02; /* twoStep */
  const int64_t t1 = master_time(m, now_T);
  send_to_board(m, 0, sync, sizeof(sync), 0);

  uint8_t fu[100];
  size_t len = 44;
  uint8_t *tlv = fu + 44;
  memset(fu, 0, sizeof(fu));
  if (sc.tlv8021as) {
    put_be(tlv, 3, 2);
    put_be(tlv + 2, 28, 2);
    put_be(tlv + 4, 0x0080C2, 3);
    put_be(tlv + 7, 1, 3);
    put_be(tlv + 14, (uint16_t)(m + 1), 2); /* gmTimeBaseIndicator */
    tlv += 32;
    len += 32;
  }
  put_be(tlv, 3, 2); /* Apple TLV: GM id */
  put_be(tlv + 2, 14, 2);
  put_be(tlv + 4, 0x000D93, 3);
  put_be(tlv + 7, 4, 3);
  put_be(tlv + 10, gm_of(m), 8);
  len += 18;
  uint8_t hdr[sizeof(fu)];
  header(hdr, m, PTP_MSG_FOLLOW_UP, (uint16_t)len, seq, 2);
  memcpy(fu, hdr, 34);
  put_ts(fu + 34, t1);
  send_to_board(m, 1, fu, len, 300 * US);
}

static void ann_tx(int m) {
  master_t *x = &ms[m];
  const uint16_t seq = x->ann_seq++;
  if (master_asleep(m, now_T)) {
    return;
  }
  uint8_t a[64];
  header(a, m, PTP_MSG_ANNOUNCE, 64, seq, 5);
  a[47] = x->p1;
  a[48] = 248;
  a[49] = 0x21;
  put_be(a + 50, 0x436A, 2);
  a[52] = x->p2;
  put_be(a + 53, gm_of(m), 8);
  put_be(a + 61, x->steps, 2);
  a[63] = 0xA0;
  send_to_board(m, 1, a, sizeof(a), 0);
}

/* The board's Delay_Req, from inside the clock's packet handler. */
ssize_t sim_sendto(int fd, const void *buf, size_t len, int flags,
                   const struct sockaddr *dst, socklen_t dst_len) {
  (void)fd, (void)flags, (void)dst_len;
  const struct sockaddr_in *in = (const struct sockaddr_in *)dst;
  int m = -1;
  for (int i = 0; i < M_COUNT; i++) {
    if (ms[i].present && ms[i].ip == in->sin_addr.s_addr) {
      m = i;
    }
  }
  if (m < 0 || len != 44 || ntohs(in->sin_port) != 319) {
    fprintf(stderr, "bad Delay_Req\n");
    exit(2);
  }
  if (urand_s(R_REV) < sc.loss) {
    return (ssize_t)len;
  }
  master_t *x = &ms[m];
  int64_t arrival = now_T + (int64_t)((30 + exprand_s(R_REV, 60)) * US) +
                    path_delay(1, R_REV);
  if (arrival <= x->last_arrival[1]) {
    arrival = x->last_arrival[1] + 20 * US;
  }
  x->last_arrival[1] = arrival;
  event_t e = {0};
  e.t = arrival;
  e.type = EV_DREQ_AT_MASTER;
  e.m = m;
  e.len = len;
  memcpy(e.buf, buf, len);
  push(e);
  return (ssize_t)len;
}

static void dreq_at_master(const event_t *e) {
  master_t *x = &ms[e->m];
  if (!x->answers || master_asleep(e->m, now_T)) {
    return;
  }
  int64_t late = 300 * US; /* a little processing */
  if (in_storm(now_T)) {
    /* Most answers lost, the rest held up past the next SYNCs. */
    /* ...and for 4 s in the middle, every answer: what drove the board's
     * unanswered count to 16 on hardware. */
    const double into = (double)now_T / SEC - sc.storm_at;
    if (urand_s(R_REV) < 0.6 || (into >= 10 && into < 14)) {
      return;
    }
    late += (int64_t)(urand_s(R_REV) * 1500) * MS;
  }
  const int64_t t4 = master_time(e->m, now_T);
  uint8_t r[54];
  header(r, e->m, PTP_MSG_DELAY_RESP, 54,
         (uint16_t)((e->buf[30] << 8) | e->buf[31]), 3);
  put_ts(r + 34, t4);
  memcpy(r + 44, e->buf + 20, 10);
  send_to_board(e->m, 1, r, sizeof(r), late);
}

/* ---------- board side ---------- */

static void deliver(const event_t *e) {
  const uint32_t ip = ms[e->m].ip;
#ifdef SIM_BASELINE
  (void)ip;
  process_ptp_message(e->buf, e->len, e->sock == 0);
#else
  int64_t rx_ns = (int64_t)esp_timer_get_time() * 1000;
  if (e->sock == 0 && sc.hook) {
    bool hook;
    rx_ns = rx_timestamp_ns(e->buf, e->len, ip, rx_ns, &hook);
  }
  if (getenv("SIM_DEBUG_FU") && e->sock == 1 &&
      (e->buf[0] & 0x0F) == PTP_MSG_FOLLOW_UP && ptp.selected >= 0 &&
      ptp.ports[ptp.selected].ip == ip) {
    const ptp_port_t *p = &ptp.ports[ptp.selected];
    const uint16_t seq = (uint16_t)((e->buf[30] << 8) | e->buf[31]);
    if (!p->pending || p->pending_seq != seq) {
      printf("FU %u unmatched: pending=%d pending_seq=%u t=%.3f\n", seq,
             p->pending, p->pending_seq, (double)now_T / SEC);
    }
  }
  handle_packet(e->buf, e->len, e->sock == 0, ip, rx_ns);
#endif
}

static void hook(const event_t *e) {
#ifndef SIM_BASELINE
  if (e->sock == 0 && sc.hook) {
    uint8_t pkt[20 + 8 + 128] = {0};
    pkt[0] = 0x45;
    put_be(pkt + 2, 28 + e->len, 2);
    pkt[8] = 64;
    pkt[9] = 17;
    memcpy(pkt + 12, &ms[e->m].ip, 4);
    memcpy(pkt + 16, &esp_ip, 4);
    put_be(pkt + 20, 319, 2);
    put_be(pkt + 22, 319, 2);
    put_be(pkt + 24, 8 + e->len, 2);
    memcpy(pkt + 28, e->buf, e->len);
    rx_stamp_ip4(pkt, 28 + e->len, esp_timer_get_time());
  }
#endif
  /* then the socket mailbox and the PTP task */
  event_t t = *e;
  int64_t lat = (int64_t)((80 + exprand_s(R_RX, sc.task_exp_us)) * US);
  if (urand_s(R_RX) < 0.03) {
    lat += (int64_t)((2000 + urand_s(R_RX) * 6000) * US); /* a decode */
  }
  t.t = now_T + lat;
  if (t.t <= last_task) {
    t.t = last_task + 5 * US;
  }
  last_task = t.t;
  t.type = EV_TASK;
  push(t);
}

/* Read the clock at "now": network time on the named GM, or unlocked. */
static bool read_clock(uint64_t gm, int64_t *net) {
  const int64_t local = (int64_t)esp_timer_get_time() * 1000;
#ifdef SIM_BASELINE
  if (!ptp_clock_is_locked() || ptp_clock_get_master_clock_id() != gm) {
    return false;
  }
  *net = local + ptp_clock_get_offset_ns();
  return true;
#else
  uint64_t n, g;
  if (!ptp_clock_local_to_network(local, &n, &g) || g != gm) {
    return false;
  }
  *net = (int64_t)n;
  return true;
#endif
}

/* ---------- metrics ---------- */

typedef struct {
  int64_t t;
  int64_t err;
  bool locked;
} sample_t;

static sample_t *samples;
static size_t nsamples;

static int cmp_i64(const void *a, const void *b) {
  const int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
  return (x > y) - (x < y);
}

static bool in_event_window(int64_t t) {
  const double s = (double)t / SEC;
  const double recover = 60;
  if (sc.sleep_len > 0 && s >= sc.sleep_at &&
      s < sc.sleep_at + sc.sleep_len + recover) {
    return true;
  }
  if (sc.gm_change_at > 0 && s >= sc.gm_change_at &&
      s < sc.gm_change_at + recover) {
    return true;
  }
  if (sc.relay_drop_at > 0 && s >= sc.relay_drop_at &&
      s < sc.relay_drop_at + recover) {
    return true;
  }
  return false;
}

/* Seconds from `from` until the clock is locked and within `tol` of
 * `center` for 5 s. */
static double settle(double from, double center, double tol) {
  double good_since = -1;
  for (size_t i = 0; i < nsamples; i++) {
    const double s = (double)samples[i].t / SEC;
    if (s < from) {
      continue;
    }
    const bool good =
        samples[i].locked && fabs((double)samples[i].err - center) < tol;
    if (!good) {
      good_since = -1;
      continue;
    }
    if (good_since < 0) {
      good_since = s;
    }
    if (s - good_since >= 5) {
      return good_since - from;
    }
  }
  return -1;
}

static void report(void) {
  int64_t *errs = malloc(nsamples * sizeof(*errs));
  size_t n = 0, steady = 0, unlocked = 0;
  double sum = 0;
  int64_t lo = INT64_MAX, hi = INT64_MIN;
  double first_lock = -1;
  for (size_t i = 0; i < nsamples; i++) {
    const sample_t *x = &samples[i];
    if (first_lock < 0 && x->locked) {
      first_lock = (double)x->t / SEC;
    }
    if (x->t < 60 * SEC || in_event_window(x->t)) {
      continue;
    }
    steady++;
    if (!x->locked) {
      unlocked++;
      continue;
    }
    errs[n++] = x->err;
    sum += (double)x->err;
    if (x->err < lo) {
      lo = x->err;
    }
    if (x->err > hi) {
      hi = x->err;
    }
  }
  for (size_t i = 0; i < n; i++) {
    errs[i] = llabs(errs[i]);
  }
  qsort(errs, n, sizeof(*errs), cmp_i64);
  const double p95 = n ? (double)errs[(size_t)(0.95 * (double)(n - 1))] : -1;
  const double peak = n ? (double)errs[n - 1] : -1;
  const double mean = n ? sum / (double)n : 0;
  /* Settled: within 500 us of where the clock sits in steady state (its
   * bias is the mean column).  Recovered: back within 1 ms of it. */
  const double settled = settle(0, mean, 500 * US);
  double rec = -1;
  if (sc.sleep_len > 0) {
    rec = settle(sc.sleep_at + sc.sleep_len, mean, MS);
  } else if (sc.gm_change_at > 0) {
    rec = settle(sc.gm_change_at, mean, MS);
  } else if (sc.relay_drop_at > 0) {
    rec = settle(sc.relay_drop_at, mean, MS);
  }
  printf("mean_us=%.1f p95_us=%.1f p2p_us=%.1f peak_us=%.1f unlocked_pct=%.2f "
         "lock_s=%.2f settle_s=%.2f recovery_s=%.2f\n",
         mean / 1e3, p95 / 1e3, n ? (double)(hi - lo) / 1e3 : -1, peak / 1e3,
         steady ? 100.0 * (double)unlocked / (double)steady : 0, first_lock,
         settled, rec);
  free(errs);
}

/* ---------- scenarios ---------- */

static void base_scenario(void) {
  memset(&sc, 0, sizeof(sc));
  sc.drift_ppm = 20;
  sc.wander_ppm = 2;
  sc.wander_period = 400;
  sc.base_us[0] = 800;
  sc.base_us[1] = 800;
  sc.exp_us[0] = 300;
  sc.exp_us[1] = 300;
  sc.task_exp_us = 400;
  sc.answers = true;
  sc.duration = 660;
  sc.hook = true;
  sc.tlv8021as = true;
  sc.loss = 0.01;
}

static void wifi(void) {
  sc.base_us[0] = 900;
  sc.base_us[1] = 600; /* 150 us asymmetry after the midpoint */
  sc.exp_us[0] = 2000;
  sc.exp_us[1] = 1500;
  sc.spike_p = 0.05;
  sc.congestion = true;
  sc.task_exp_us = 800;
}

static bool setup(const char *name) {
  base_scenario();
  sc.name = name;
  if (!strcmp(name, "clean")) {
  } else if (!strcmp(name, "wifi")) {
    wifi();
  } else if (!strcmp(name, "wifi-fast50")) {
    wifi();
    sc.drift_ppm = 50;
  } else if (!strcmp(name, "wifi-slow50")) {
    wifi();
    sc.drift_ppm = -50;
  } else if (!strcmp(name, "wifi-passive")) {
    wifi();
    sc.answers = false;
  } else if (!strcmp(name, "clean-passive")) {
    sc.answers = false;
  } else if (!strcmp(name, "clean-passive-fast50")) {
    sc.answers = false;
    sc.drift_ppm = 50;
  } else if (!strcmp(name, "clean-passive-slow50")) {
    sc.answers = false;
    sc.drift_ppm = -50;
  } else if (!strcmp(name, "wifi-passive-fast50")) {
    wifi();
    sc.answers = false;
    sc.drift_ppm = 50;
  } else if (!strcmp(name, "wifi-nohook")) {
    wifi();
    sc.hook = false;
  } else if (!strcmp(name, "wifi-no8021as")) {
    wifi();
    sc.tlv8021as = false;
  } else if (!strcmp(name, "sleep")) {
    wifi();
    sc.sleep_at = 300;
    sc.sleep_len = 4.445;
  } else if (!strcmp(name, "sleep-passive")) {
    wifi();
    sc.answers = false;
    sc.sleep_at = 300;
    sc.sleep_len = 4.445;
  } else if (!strcmp(name, "gm-change")) {
    wifi();
    sc.gm_change_at = 300;
  } else if (!strcmp(name, "relay")) {
    wifi();
    sc.relay_drop_at = 200;
  } else if (!strcmp(name, "storm")) {
    wifi();
    sc.storm_at = 400;
    sc.storm_len = 25;
  } else if (!strcmp(name, "multi")) {
    wifi();
    sc.multi = true;
  } else {
    return false;
  }
  return true;
}

static void init_masters(void) {
  memset(ms, 0, sizeof(ms));
  for (int i = 0; i < M_COUNT; i++) {
    ms[i].relay_of = -1;
    ms[i].off_from = -1;
    ms[i].p1 = 248;
    ms[i].p2 = 247;
    ms[i].answers = sc.answers;
  }
  /* A: the phone, GM of the session */
  ms[M_A].present = true;
  ms[M_A].clock_id = 0x5c3e1b15cc4d0008ULL;
  ms[M_A].ip = 0x0a01a8c0;
  ms[M_A].p0 = 1763900000LL * SEC + (int64_t)(urand() * SEC);
  ms[M_A].rate_ppm = 0;
  if (sc.relay_drop_at > 0) {
    ms[M_A].off_from = sc.relay_drop_at;
    ms[M_R].present = true;
    ms[M_R].clock_id = 0x0011223344556677ULL;
    ms[M_R].ip = 0x1e01a8c0;
    ms[M_R].relay_of = M_A;
    ms[M_R].steps = 1;
  }
  if (sc.gm_change_at > 0) {
    ms[M_B].present = true;
    ms[M_B].clock_id = 0x7788990011223344ULL;
    ms[M_B].ip = 0x1401a8c0;
    ms[M_B].p0 = 912345LL * SEC + (int64_t)(urand() * SEC);
    ms[M_B].rate_ppm = 7;
  }
  if (sc.multi) {
    ms[M_C].present = true;
    ms[M_C].clock_id = 0x0000aabbccddeeffULL;
    ms[M_C].ip = 0x2801a8c0;
    ms[M_C].p0 = 77777LL * SEC;
    ms[M_C].rate_ppm = -12;
    ms[M_C].p1 = 16; /* a mains-powered Apple TV that prefers itself */
  }
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: sim <scenario> <seed> [-v]\n");
    return 2;
  }
  verbose = argc > 3;
  if (!setup(argv[1])) {
    fprintf(stderr, "unknown scenario %s\n", argv[1]);
    return 2;
  }
  for (int s = 0; s < R_COUNT; s++) {
    rng[s] = 0x9E3779B97F4A7C15ULL * (uint64_t)(atoi(argv[2]) + 1) +
             0xD1B54A32D192ED03ULL * (uint64_t)(s + 1);
    for (int i = 0; i < 8; i++) {
      urand_s(s);
    }
  }
  init_masters();
  nbursts = 0;
  if (sc.congestion) {
    double t = 20 + exprand(90);
    while (t < sc.duration && nbursts < 64) {
      const double len = 2 + urand() * 3;
      bursts[nbursts++] = (burst_t){t, t + len, 50000 + urand() * 150000};
      t += len + exprand(120);
    }
  }
  local_ns = 5.0 * SEC + urand() * SEC;
  now_T = 0;
  advance_to(0);

#ifndef SIM_BASELINE
  reset_all_state();
  init_identity();
  ptp.event_socket = 3;
#endif

  for (int m = 0; m < M_COUNT; m++) {
    if (!ms[m].present) {
      continue;
    }
    event_t e = {0};
    e.type = EV_SYNC_TX;
    e.m = m;
    e.t = (int64_t)(urand() * 125 * MS);
    push(e);
    e.type = EV_ANN_TX;
    e.t = (int64_t)(urand() * SEC);
    push(e);
  }
  event_t e = {0};
  e.type = EV_SAMPLE;
  e.t = 0;
  push(e);
  e.type = EV_ANCHOR;
  e.t = 1 * SEC;
  e.arg = ms[M_A].clock_id;
  push(e);
  if (sc.gm_change_at > 0) {
    e.t = (int64_t)(sc.gm_change_at * SEC);
    e.arg = ms[M_B].clock_id;
    push(e);
  }

  const int64_t end = (int64_t)(sc.duration * SEC);
  uint64_t truth_gm = 0;
  int truth_m = M_A;
  bool slept = false;
  samples = malloc(sizeof(*samples) * (size_t)(sc.duration * 50 + 10));
  nsamples = 0;

  while (heap_n > 0) {
    event_t ev = pop();
    if (ev.t > end) {
      break;
    }
    advance_to(ev.t);
    /* The phone's clock stopped while it slept: it resumes stepped back. */
    if (!slept && sc.sleep_len > 0 &&
        now_T >= (int64_t)((sc.sleep_at + sc.sleep_len) * SEC)) {
      slept = true;
      ms[M_A].step -= (int64_t)(sc.sleep_len * SEC);
    }
    switch (ev.type) {
    case EV_SYNC_TX:
      sync_tx(ev.m);
      ev.t = now_T + 125 * MS + (int64_t)((urand_s(R_FWD) - 0.5) * 2 * MS);
      push(ev);
      break;
    case EV_ANN_TX:
      ann_tx(ev.m);
      ev.t = now_T + SEC;
      push(ev);
      break;
    case EV_HOOK:
      hook(&ev);
      break;
    case EV_TASK:
      deliver(&ev);
      break;
    case EV_DREQ_AT_MASTER:
      dreq_at_master(&ev);
      break;
    case EV_ANCHOR:
      truth_gm = ev.arg;
      truth_m = ev.arg == ms[M_B].clock_id && ms[M_B].present ? M_B : M_A;
      ptp_clock_set_master_clock_id(ev.arg);
      break;
    case EV_SAMPLE:
      if (truth_gm != 0) {
        int64_t net = 0;
        const bool locked = read_clock(truth_gm, &net);
        samples[nsamples++] = (sample_t){
            now_T,
            locked ? net - master_time(truth_m == M_B ? M_B : M_A, now_T) : 0,
            locked};
      }
      if (getenv("SIM_TRACE") && nsamples % 50 == 0 && nsamples > 0) {
        const sample_t *x = &samples[nsamples - 1];
        printf("trace t=%.1f locked=%d err_us=%.1f", (double)x->t / SEC,
               x->locked, (double)x->err / 1e3);
#ifndef SIM_BASELINE
        printf(" mode=%d out=%s tw_n=%u tw_fs=%d ow_n=%u bias_us=%.0f/%u "
               "unans=%u",
               ptp.mode, output_servo() == &ptp.tw ? "tw" : "ow",
               (unsigned)ptp.tw.samples, ptp.tw.fast_start,
               (unsigned)ptp.ow.samples, (double)ptp.passive_bias_ns / 1e3,
               (unsigned)ptp.passive_bias_n, (unsigned)ptp.unanswered);
#endif
        printf("\n");
      }
      ev.t = now_T + 20 * MS;
      push(ev);
      break;
    default:
      break;
    }
  }
  report();
#ifndef SIM_BASELINE
  if (getenv("SIM_TRACE")) {
    printf("syncs=%u fu=%u dreq=%u dresp=%u superseded=%u ooo=%u ppm=%u "
           "delay=%u passive=%u resets drop=%u oob=%u gap=%u gm=%u "
           "limit_us=%lld\n",
           ptp.sync_count, ptp.followup_count, ptp.dreq_sent,
           ptp.dresp_received, ptp.superseded, ptp.out_of_order, ptp.ppm_drops,
           ptp.delay_drops, ptp.passive_drops, ptp.resets_drops,
           ptp.resets_bounds, ptp.resets_gap, ptp.gm_changes,
           (long long)ptp.delay_limit_ns / 1000);
  }
#endif
  return 0;
}

void sim_log(const char *tag, const char *format, ...) {
  if (!verbose) {
    return;
  }
  va_list args;
  va_start(args, format);
  printf("%9.3f [%s] ", (double)now_T / SEC, tag);
  vprintf(format, args);
  printf("\n");
  va_end(args);
}

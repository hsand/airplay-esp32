// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Seeded host simulation of the AirPlay 1 timing client.
 *
 * The real ntp_clock.c (NTP_SOURCE: the current one, or the v0.3.0 copy in
 * baseline/) runs its own task loop against a simulated sender over a
 * simulated Wi-Fi link: asymmetric delays with jitter, spikes and loss, a
 * receive-path latency before the task can stamp a reply, a local crystal
 * off by up to +-50 ppm that also wanders, and optionally a sender whose
 * clock jumps.  Every 10 ms of simulated time the clock's idea of network
 * time "now" is compared with the truth.
 *
 * Usage: sim [--units]   prints one line per scenario and seed:
 *   <scenario> <seed> <lock_s> <bias_us> <p95_us> <p95_dev_us> <pp_us>
 *   <saw_us> <startup_p95_us> <recovery_s>
 */

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "mocks.h"

#include NTP_SOURCE

/* ---------- scenarios ---------- */

typedef struct {
  const char *name;
  double up_ms, down_ms; // fixed one-way delays: their difference is a bias
  double jitter_ms;      // exponential, mean
  double spike_p, spike_min_ms, spike_max_ms;
  double loss_p;
  double local_ppm;  // the crystal is off by up to this, either way
  double wander_ppm; // random walk of the crystal, per second
  double sender_ppm;
  double step_at_s, step_ms;
  double duration_s;
} scenario_t;

static const scenario_t scenarios[] = {
    {"lan", 0.6, 0.9, 0.2, 0.01, 2, 10, 0.005, 50, 0.01, 5, 0, 0, 600},
    {"wifi", 1.5, 2.5, 2.0, 0.08, 5, 80, 0.03, 50, 0.02, 5, 0, 0, 600},
    {"wifi_bad", 2.0, 4.0, 8.0, 0.15, 20, 150, 0.05, 50, 0.02, 5, 0, 0, 600},
    {"step_30ms", 1.5, 2.5, 2.0, 0.08, 5, 80, 0.03, 50, 0.02, 5, 300, 30, 600},
    {"step_400ms", 1.5, 2.5, 2.0, 0.08, 5, 80, 0.03, 50, 0.02, 5, 300, 400,
     600},
};
#define N_SCENARIOS (sizeof(scenarios) / sizeof(scenarios[0]))
#define N_SEEDS     8

#define SAMPLE_NS     10000000LL   // compare every 10 ms
#define NTP_EPOCH_NOW 3976214400LL // 2026, in NTP seconds
#define LOCAL_BOOT_NS 5000000000LL // the board has been up 5 s
#define WARMUP_S      60.0
#define STEP_SETTLE_S 120.0

/* ---------- simulated world ---------- */

static const scenario_t *sc;
static uint64_t rng;
static int64_t t_ns;    // true time since the start
static double local_ns; // the board's clock
static double local_ppm;
static double sender_ppm;
static int64_t end_ns;
static int64_t next_sample_ns;
static bool finished;

typedef struct {
  int64_t visible_ns; // when the task can see it
  uint8_t data[32];
} reply_t;
#define MAX_REPLIES 16
static reply_t replies[MAX_REPLIES];
static int n_replies;

/* Error samples, ns, with their true times. */
static double *err;
static double *err_t;
static size_t n_err;
static double lock_s;

static double urand(void) {
  rng ^= rng << 13;
  rng ^= rng >> 7;
  rng ^= rng << 17;
  return (double)(rng >> 11) / 9007199254740992.0;
}

static double nrand(void) {
  const double u = urand() + 1e-12;
  const double v = urand();
  return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
}

uint32_t sim_random(void) {
  return (uint32_t)(urand() * 4294967296.0);
}

static bool verbose;
static bool trace;
void sim_log(const char *tag, const char *format, ...) {
  (void)tag;
  if (!verbose) {
    return;
  }
  va_list ap;
  va_start(ap, format);
  printf("  [%8.3f] ", (double)t_ns / 1e9);
  vprintf(format, ap);
  printf("\n");
  va_end(ap);
}

static int64_t sender_ns_at(int64_t at_ns) {
  int64_t net = NTP_EPOCH_NOW * 1000000000LL + at_ns +
                llround((double)at_ns * sender_ppm * 1e-6);
  if (sc->step_ms != 0 && at_ns >= (int64_t)(sc->step_at_s * 1e9)) {
    net += llround(sc->step_ms * 1e6);
  }
  return net;
}

static bool clock_reading(int64_t local, int64_t *net);

static void record_sample(void) {
  int64_t net = 0;
  if (!clock_reading((int64_t)local_ns, &net)) {
    return;
  }
  if (lock_s < 0) {
    lock_s = (double)t_ns / 1e9;
  }
  err[n_err] = (double)(net - sender_ns_at(t_ns));
  err_t[n_err] = (double)t_ns / 1e9;
  if (trace && t_ns % 1000000000LL == 0) {
    printf("trace %s %.0f %.1f ppm=%.2f\n", sc->name, err_t[n_err],
           err[n_err] / 1e3, local_ppm - sender_ppm);
  }
  n_err++;
}

/* Moves true time forward, the crystal with it, sampling on the way. */
static void advance_to(int64_t target_ns) {
  if (target_ns > end_ns) {
    target_ns = end_ns;
  }
  while (t_ns < target_ns) {
    int64_t next = target_ns < next_sample_ns ? target_ns : next_sample_ns;
    const int64_t dt = next - t_ns;
    // The crystal wanders as a random walk, a step every sample period.
    local_ns += (double)dt * (1.0 + local_ppm * 1e-6);
    t_ns = next;
    if (t_ns == next_sample_ns) {
      local_ppm += sc->wander_ppm * sqrt((double)SAMPLE_NS / 1e9) * nrand();
      record_sample();
      next_sample_ns += SAMPLE_NS;
    }
  }
  if (t_ns >= end_ns) {
    finished = true;
  }
}

int64_t sim_local_us(void) {
  return (int64_t)local_ns / 1000;
}

TickType_t sim_tick_count(void) {
  return (TickType_t)(sim_local_us() / 1000);
}

void sim_task_delay(uint32_t ms) {
  advance_to(t_ns + (int64_t)ms * 1000000LL);
}

static TaskFunction_t task_fn;
BaseType_t sim_task_create(TaskFunction_t fn, const char *name, uint32_t depth,
                           void *param, int prio, TaskHandle_t *handle) {
  (void)name, (void)depth, (void)param, (void)prio;
  task_fn = fn;
  *handle = (TaskHandle_t)1;
  return pdPASS;
}

static double one_way_ms(double base) {
  double d = base - sc->jitter_ms * log(urand() + 1e-12);
  if (urand() < sc->spike_p) {
    d += sc->spike_min_ms + (sc->spike_max_ms - sc->spike_min_ms) * urand();
  }
  return d;
}

/* From the reply reaching the board to the task being able to stamp it:
 * Wi-Fi driver, lwIP and the scheduler.  The same for old and new code. */
static double rx_latency_ms(void) {
  double d = 0.05 - 0.15 * log(urand() + 1e-12);
  if (urand() < 0.03) {
    d += 1.0 + 4.0 * urand();
  }
  return d;
}

static void put_ntp(uint8_t *p, int64_t ns) {
  const uint64_t secs = (uint64_t)(ns / 1000000000LL);
  const uint64_t frac = ((uint64_t)(ns % 1000000000LL) << 32) / 1000000000ULL;
  const uint64_t v = (secs << 32) | frac;
  for (int i = 7; i >= 0; i--) {
    p[7 - i] = (uint8_t)(v >> (i * 8));
  }
}

static ssize_t deliver_request(const void *buf, size_t len) {
  if (finished) {
    errno = EBADF;
    return -1;
  }
  const uint8_t *req = buf;
  if (len < 32 || req[1] != 0xD2) {
    return (ssize_t)len;
  }
  if (urand() < sc->loss_p) {
    return (ssize_t)len; // lost on the way out
  }
  const int64_t at_sender = t_ns + llround(one_way_ms(sc->up_ms) * 1e6);
  const int64_t leaves = at_sender + 40000; // 40 us to answer
  if (urand() < sc->loss_p || n_replies == MAX_REPLIES) {
    return (ssize_t)len; // lost on the way back
  }
  reply_t *r = &replies[n_replies++];
  memset(r->data, 0, sizeof(r->data));
  r->data[0] = 0x80;
  r->data[1] = 0xD3;
  r->data[3] = 0x07;
  memcpy(r->data + 8, req + 24, 8); // origin = our transmit timestamp
  put_ntp(r->data + 16, sender_ns_at(at_sender));
  put_ntp(r->data + 24, sender_ns_at(leaves));
  r->visible_ns =
      leaves + llround((one_way_ms(sc->down_ms) + rx_latency_ms()) * 1e6);
  return (ssize_t)len;
}

static int earliest_reply(void) {
  int best = -1;
  for (int i = 0; i < n_replies; i++) {
    if (best < 0 || replies[i].visible_ns < replies[best].visible_ns) {
      best = i;
    }
  }
  return best;
}

/* Waits until a reply is visible or timeout_ns passes.  True if one is. */
static bool wait_reply(int64_t timeout_ns) {
  const int64_t deadline = t_ns + timeout_ns;
  const int i = earliest_reply();
  if (i >= 0 && replies[i].visible_ns <= deadline) {
    advance_to(replies[i].visible_ns > t_ns ? replies[i].visible_ns : t_ns);
    return !finished;
  }
  advance_to(deadline);
  return false;
}

static ssize_t take_reply(void *buf, size_t len) {
  const int i = earliest_reply();
  if (i < 0 || replies[i].visible_ns > t_ns) {
    errno = EAGAIN;
    return -1;
  }
  const size_t n = len < 32 ? len : 32;
  memcpy(buf, replies[i].data, n);
  replies[i] = replies[--n_replies];
  return (ssize_t)n;
}

int sim_socket(int domain, int type, int protocol) {
  (void)domain, (void)type, (void)protocol;
  return 3;
}

static int64_t rcvtimeo_ns = 100000000LL;
int sim_setsockopt(int fd, int level, int name, const void *value,
                   socklen_t len) {
  (void)fd, (void)len;
  if (level == SOL_SOCKET && name == SO_RCVTIMEO) {
    const struct timeval *tv = value;
    rcvtimeo_ns =
        (int64_t)tv->tv_sec * 1000000000LL + (int64_t)tv->tv_usec * 1000LL;
  }
  return 0;
}

int sim_connect(int fd, const struct sockaddr *addr, socklen_t len) {
  (void)fd, (void)addr, (void)len;
  return 0;
}

ssize_t sim_sendto(int fd, const void *buf, size_t len, int flags,
                   const struct sockaddr *to, socklen_t tolen) {
  (void)fd, (void)flags, (void)to, (void)tolen;
  return deliver_request(buf, len);
}

ssize_t sim_send(int fd, const void *buf, size_t len, int flags) {
  (void)fd, (void)flags;
  return deliver_request(buf, len);
}

// v0.3.0: blocking with SO_RCVTIMEO.
ssize_t sim_recvfrom(int fd, void *buf, size_t len, int flags,
                     struct sockaddr *from, socklen_t *fromlen) {
  (void)fd, (void)flags, (void)from, (void)fromlen;
  if (finished) {
    errno = EBADF;
    return -1;
  }
  if (!wait_reply(rcvtimeo_ns)) {
    errno = finished ? EBADF : EAGAIN;
    return -1;
  }
  return take_reply(buf, len);
}

ssize_t sim_recv(int fd, void *buf, size_t len, int flags) {
  (void)fd, (void)flags;
  return take_reply(buf, len);
}

int sim_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
               struct timeval *timeout) {
  (void)nfds, (void)writefds, (void)exceptfds;
  if (finished) {
    errno = EBADF;
    return -1;
  }
  const int64_t wait = (int64_t)timeout->tv_sec * 1000000000LL +
                       (int64_t)timeout->tv_usec * 1000LL;
  if (wait_reply(wait)) {
    return 1;
  }
  if (finished) {
    errno = EBADF;
    return -1;
  }
  FD_ZERO(readfds);
  return 0;
}

int sim_close(int fd) {
  (void)fd;
  return 0;
}

/* ---------- the clock under test ---------- */

static bool clock_reading(int64_t local, int64_t *net) {
#ifdef NTP_NEW
  uint64_t n = 0;
  if (!ntp_clock_local_to_network(local, &n)) {
    return false;
  }
  *net = (int64_t)n;
  return true;
#else
  if (!ntp_clock_is_locked()) {
    return false;
  }
  // audio_receiver.c v0.3.0: network = local + offset.
  *net = local + ntp_clock_get_offset_ns();
  return true;
#endif
}

/* ---------- metrics ---------- */

static int cmp_double(const void *a, const void *b) {
  const double x = *(const double *)a, y = *(const double *)b;
  return x < y ? -1 : x > y;
}

static bool in_steady(double at) {
  if (at < WARMUP_S) {
    return false;
  }
  if (sc->step_ms != 0 && at >= sc->step_at_s &&
      at < sc->step_at_s + STEP_SETTLE_S) {
    return false;
  }
  return true;
}

static void report(unsigned seed) {
  double *abs_err = malloc(n_err * sizeof(double));
  double *startup = malloc(n_err * sizeof(double));
  size_t n = 0, n_start = 0;
  double sum = 0, lo = 1e300, hi = -1e300;
  for (size_t i = 0; i < n_err; i++) {
    if (err_t[i] < WARMUP_S) {
      startup[n_start++] = fabs(err[i]);
    }
    if (!in_steady(err_t[i])) {
      continue;
    }
    sum += err[i];
    abs_err[n++] = fabs(err[i]);
    lo = fmin(lo, err[i]);
    hi = fmax(hi, err[i]);
  }
  const double bias = n ? sum / (double)n : 0;
  // Spread about the bias: what moves, as against what is merely offset.
  double *dev = malloc(n_err * sizeof(double));
  size_t n_dev = 0;
  for (size_t i = 0; i < n_err; i++) {
    if (in_steady(err_t[i])) {
      dev[n_dev++] = fabs(err[i] - bias);
    }
  }
  qsort(dev, n_dev, sizeof(double), cmp_double);
  const double jit = n_dev ? dev[(size_t)(0.95 * (double)(n_dev - 1))] : 0;
  free(dev);
  qsort(abs_err, n, sizeof(double), cmp_double);
  qsort(startup, n_start, sizeof(double), cmp_double);
  const double p95 = n ? abs_err[(size_t)(0.95 * (double)(n - 1))] : 0;
  const double sp95 =
      n_start ? startup[(size_t)(0.95 * (double)(n_start - 1))] : 0;

  // Sawtooth: the mean spread inside 3 s windows, i.e. what one exchange
  // interval adds or removes.
  double saw = 0, wlo = 1e300, whi = -1e300;
  int windows = 0;
  double wstart = -1;
  for (size_t i = 0; i < n_err; i++) {
    if (!in_steady(err_t[i])) {
      continue;
    }
    if (wstart < 0 || err_t[i] >= wstart + 3.0) {
      if (wstart >= 0) {
        saw += whi - wlo;
        windows++;
      }
      wstart = err_t[i];
      wlo = 1e300;
      whi = -1e300;
    }
    wlo = fmin(wlo, err[i]);
    whi = fmax(whi, err[i]);
  }
  saw = windows ? saw / windows : 0;

  /* After a sender step: until the error, averaged over 3 s, is back within
   * 1 ms of where it stood before the step. */
  double recovery = 0;
  if (sc->step_ms != 0) {
    double before = 0;
    int nb = 0;
    for (size_t i = 0; i < n_err; i++) {
      if (err_t[i] >= WARMUP_S && err_t[i] < sc->step_at_s) {
        before += err[i];
        nb++;
      }
    }
    before = nb ? before / nb : 0;
    double wsum = 0;
    int wn = 0;
    double wbegin = sc->step_at_s;
    for (size_t i = 0; i < n_err; i++) {
      if (err_t[i] < sc->step_at_s) {
        continue;
      }
      if (err_t[i] >= wbegin + 3.0) {
        if (wn && fabs(wsum / wn - before) >= 1e6) {
          recovery = wbegin + 3.0 - sc->step_at_s;
        }
        wbegin += 3.0;
        wsum = 0;
        wn = 0;
      }
      wsum += err[i];
      wn++;
    }
  }
  printf("%s %u %.3f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", sc->name, seed,
         lock_s, bias / 1e3, p95 / 1e3, jit / 1e3, (hi - lo) / 1e3, saw / 1e3,
         sp95 / 1e3, recovery);
  free(abs_err);
  free(startup);
}

static void run(const scenario_t *scenario, unsigned seed) {
  sc = scenario;
  rng = 0x9E3779B97F4A7C15ULL * (seed + 1) ^ 0xD1B54A32D192ED03ULL;
  for (int i = 0; i < 10; i++) {
    (void)urand();
  }
  t_ns = 0;
  local_ns = (double)LOCAL_BOOT_NS;
  local_ppm = sc->local_ppm * (2 * urand() - 1);
  sender_ppm = sc->sender_ppm * (2 * urand() - 1);
  end_ns = (int64_t)(sc->duration_s * 1e9);
  next_sample_ns = SAMPLE_NS;
  finished = false;
  n_replies = 0;
  n_err = 0;
  lock_s = -1;
  task_fn = NULL;

  if (ntp_clock_start_client(htonl(0xC0A80102), 7010) != ESP_OK || !task_fn) {
    printf("%s %u start failed\n", sc->name, seed);
    return;
  }
  task_fn(NULL); // runs until the simulated time is up
  ntp_clock_stop();
  report(seed);
}

#ifdef NTP_NEW
/* ---------- unit checks on the discipline itself ---------- */

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

static void unit_checks(void) {
  printf("discipline unit checks\n");
  ntp_disc_t d;
  const int64_t t0 = 10 * NS_PER_S;

  // The first sample steps, whatever its size.
  disc_init(&d, t0);
  CHECK(disc_adjust(&d, t0, 12345678901LL, true), "first sample not a step");
  CHECK(disc_network_at(&d, t0) == t0 + 12345678901LL, "step not applied");

  // Under 100 ms is slewed: 1/16 of what is left each second.
  disc_init(&d, t0);
  CHECK(!disc_adjust(&d, t0, 10000000, false), "10 ms stepped");
  CHECK(disc_network_at(&d, t0) == t0, "slew applied at once");
  // After the first boundary the rate is +10 ms/16 per second.
  const int64_t after2 =
      disc_network_at(&d, t0 + 2 * NS_PER_S) - t0 - 2 * NS_PER_S;
  CHECK(after2 == 625000, "1 s of slew moved %lld ns, want 625000",
        (long long)after2);
  // After 64 s nearly all of it is in (1 - (15/16)^63).
  const int64_t after64 =
      disc_network_at(&d, t0 + 65 * NS_PER_S) - t0 - 65 * NS_PER_S;
  const double want = 10e6 * (1.0 - pow(15.0 / 16.0, 64));
  CHECK(fabs((double)after64 - want) < 1000, "64 s of slew: %lld, want %.0f",
        (long long)after64, want);

  // Over 100 ms steps, and leaves the frequency alone.
  disc_init(&d, t0);
  CHECK(disc_adjust(&d, t0, -150000000, false), "-150 ms slewed");
  CHECK(d.freq_q32 == 0 && d.offset_q32 == 0, "step touched freq/phase");
  CHECK(!disc_adjust(&d, t0, 100000000, false), "exactly 100 ms stepped");

  // The frequency integrates error x elapsed / 4096 and clamps at 500 ppm.
  disc_init(&d, t0);
  (void)disc_adjust(&d, t0, 1000000, false);
  (void)disc_adjust(&d, t0 + 4 * NS_PER_S, 1000000, false);
  const double ppb = (double)d.freq_q32 / 4294967296.0;
  CHECK(fabs(ppb - 1e6 * 4 / 4096) < 1e-3, "freq %.3f ns/s, want %.3f", ppb,
        1e6 * 4 / 4096);
  for (int i = 2; i < 100; i++) {
    (void)disc_adjust(&d, t0 + 4 * i * NS_PER_S, 90000000, false);
  }
  CHECK(d.freq_q32 == 500000LL * (1LL << 32), "freq not clamped: %lld",
        (long long)(d.freq_q32 >> 32));

  // A pure frequency correction keeps that rate across many seconds.
  disc_init(&d, t0);
  d.freq_q32 = 40000LL * (1LL << 32); // 40 ppm
  d.adj_q32 = d.freq_q32;
  const int64_t later = 100 * NS_PER_S + 123456789;
  const int64_t gained = disc_network_at(&d, t0 + later) - t0 - later;
  CHECK(llabs(gained - 4004938) <= 2, "40 ppm over %lld ns gained %lld",
        (long long)later, (long long)gained);

  // Network -> local inverts local -> network.
  ntp.disc = d;
  ntp.disc.valid = true;
  ntp.locked = true;
  int64_t back = 0;
  uint64_t net = 0;
  CHECK(ntp_clock_local_to_network(t0 + later, &net), "not locked");
  CHECK(ntp_clock_network_to_local(net, &back) && llabs(back - t0 - later) <= 1,
        "inverse off by %lld ns", (long long)(back - t0 - later));
  ntp.locked = false;

  // NTP timestamps round-trip to the nanosecond.
  const int64_t ns = 3976214400LL * NS_PER_S + 987654321;
  CHECK(llabs(ntp_to_ns(ns_to_ntp(ns)) - ns) <= 1, "timestamp round trip");
}
#endif

int main(int argc, char **argv) {
  verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
  // sim --trace <scenario> <seed>: the error once a second.
  if (argc > 3 && strcmp(argv[1], "--trace") == 0) {
    trace = true;
    err = malloc(sizeof(double) * 70000);
    err_t = malloc(sizeof(double) * 70000);
    for (size_t s = 0; s < N_SCENARIOS; s++) {
      if (strcmp(scenarios[s].name, argv[2]) == 0) {
        run(&scenarios[s], (unsigned)atoi(argv[3]));
      }
    }
    return 0;
  }
  err = malloc(sizeof(double) * 70000);
  err_t = malloc(sizeof(double) * 70000);
#ifdef NTP_NEW
  if (argc > 1 && strcmp(argv[1], "--units") == 0) {
    unit_checks();
    printf(failures ? "%d failure(s)\n" : "all passed\n", failures);
    return failures ? 1 : 0;
  }
#endif
  for (size_t s = 0; s < N_SCENARIOS; s++) {
    for (unsigned seed = 1; seed <= N_SEEDS; seed++) {
      run(&scenarios[s], seed);
    }
  }
  free(err);
  free(err_t);
  return 0;
}

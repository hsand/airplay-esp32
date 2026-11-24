// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* AirPlay 1 timing: an NTP-style client and the software clock it steers.
 *
 * The clock follows the sender the way Apple's receiver does, which is the
 * way a kernel PLL follows NTP:
 *
 *  - It runs on the local crystal at a rate that is corrected once a second:
 *    the frequency estimate plus 1/16 of the phase error still outstanding.
 *    So a measured error is slewed out over about 16 s, never stepped, and
 *    between measurements network time keeps the sender's rate.
 *  - Each accepted measurement replaces the outstanding phase error and
 *    nudges the frequency by error x elapsed / 4096, clamped to +-500 ppm.
 *  - It steps only on the first measurement, or when the error is over
 *    100 ms, which a slew at 1/16 per second would take minutes to remove.
 *
 * The requests carry the disciplined time, so the offset a reply measures is
 * what the clock still has wrong, as in ntpd.  A reply is used only when its
 * round trip is no longer than any of the previous eight: a long round trip
 * is the sign of a delay on one leg, which the midpoint cannot cancel.
 *
 * v0.3.0 used the offset of the shortest of the last eight round trips as a
 * constant, so between changes of that sample the error grew at the full
 * crystal error (20-50 ppm, up to a millisecond over its 24 s life) and then
 * stepped; the drift servo chased that sawtooth. */

#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ntp_clock.h"

static const char *TAG = "ntp_clock";

#define TIMING_REQUEST  0xD2
#define TIMING_RESPONSE 0xD3
#define TIMING_PACKET   32

#define NS_PER_S  1000000000LL
#define NS_PER_US 1000LL
// One second in an NTP timestamp's fraction.
#define NTP_FRAC_ONE (1ULL << 32)

/* ---- Clock discipline -------------------------------------------------- */

// An error above this is stepped rather than slewed.
#define NTP_CLOCK_STEP_NS 100000000LL
// Phase is slewed at 1/2^SHIFT of what is left, once a second.
#define NTP_CLOCK_PHASE_SHIFT 4
// Frequency gain: error x elapsed seconds / 2^SHIFT, in ns/s.
#define NTP_CLOCK_FREQ_SHIFT 12
// Frequency correction limit, ns/s (500 ppm).
#define NTP_CLOCK_MAX_FREQ_NS 500000LL
// Elapsed time counted for one frequency update: a long silence must not
// turn one measurement into a frequency jump the clamp then hides.
#define NTP_CLOCK_MAX_ELAPSED_S 1024
// Readers wind a copy of the model forward second by second; past this many
// seconds without an update the rest is extrapolated at the last rate.
#define NTP_CLOCK_MAX_WIND_S 64

/* The model: network time at base_local_ns, and the rate until the next
 * once-a-second update.  Rates and phase are in ns (or ns/s) x 2^32, so a
 * slew of 1/16 per second keeps its precision down to nothing. */
typedef struct {
  bool valid;
  int64_t base_local_ns;
  int64_t base_net_ns;
  int64_t second_local_ns; // local time of the next once-a-second update
  int64_t adj_q32;         // rate correction for this second, ns/s
  int64_t offset_q32;      // phase error still to slew out, ns
  int64_t freq_q32;        // frequency correction, ns/s
  bool have_ref;
  int64_t ref_s; // local second of the last slew
} ntp_disc_t;

static int64_t div_round(int64_t num, int64_t den) {
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

/* dt_ns x rate, rate in ns/s x 2^32.  No 128-bit arithmetic on RV32, so the
 * rate drops to 8 fractional bits (1/256 ppb) and whole seconds are taken
 * apart from the rest; neither product can overflow within the clamp. */
static int64_t disc_correction_ns(int64_t dt_ns, int64_t adj_q32) {
  const int64_t adj_q8 = adj_q32 / (1LL << 24);
  const int64_t whole_s = dt_ns / NS_PER_S;
  const int64_t rest_ns = dt_ns % NS_PER_S;
  return div_round(whole_s * adj_q8, 256) +
         div_round(rest_ns * adj_q8, NS_PER_S * 256);
}

static void disc_init(ntp_disc_t *d, int64_t local_ns) {
  memset(d, 0, sizeof(*d));
  d->base_local_ns = local_ns;
  d->base_net_ns = local_ns;
  d->second_local_ns = local_ns + NS_PER_S;
}

// FreeBSD's ntp_update_second() with SHIFT_PLL 4 and no FLL.
static void disc_second(ntp_disc_t *d) {
  // Division, not a shift: a negative error must decay toward zero too.
  const int64_t phase = d->offset_q32 / (1LL << NTP_CLOCK_PHASE_SHIFT);
  d->offset_q32 -= phase;
  d->adj_q32 = d->freq_q32 + phase;
}

// Moves the model to local_ns, applying the once-a-second updates passed.
static void disc_advance(ntp_disc_t *d, int64_t local_ns) {
  int seconds = 0;
  while (local_ns >= d->second_local_ns && seconds < NTP_CLOCK_MAX_WIND_S) {
    const int64_t dt = d->second_local_ns - d->base_local_ns;
    d->base_net_ns += dt + disc_correction_ns(dt, d->adj_q32);
    d->base_local_ns = d->second_local_ns;
    d->second_local_ns += NS_PER_S;
    disc_second(d);
    seconds++;
  }
  if (local_ns >= d->second_local_ns) {
    // Far behind: carry on at the last rate and resume the updates from here.
    d->second_local_ns = local_ns + NS_PER_S;
  }
  const int64_t dt = local_ns - d->base_local_ns;
  if (dt > 0) {
    d->base_net_ns += dt + disc_correction_ns(dt, d->adj_q32);
    d->base_local_ns = local_ns;
  }
}

/* Network time at local_ns, which may lie before the model's base (a packet
 * stamped just before the last update): that is extrapolated back at the
 * current rate.  Works on a copy, so readers never write the model. */
static int64_t disc_network_at(const ntp_disc_t *model, int64_t local_ns) {
  ntp_disc_t d = *model;
  if (local_ns > d.base_local_ns) {
    disc_advance(&d, local_ns);
    return d.base_net_ns;
  }
  const int64_t dt = local_ns - d.base_local_ns;
  return d.base_net_ns + dt + disc_correction_ns(dt, d.adj_q32);
}

/* Feeds one measured error (network minus the clock, ns) to the clock at
 * local_ns.  Returns true when it stepped. */
static bool disc_adjust(ntp_disc_t *d, int64_t local_ns, int64_t offset_ns,
                        bool reset) {
  disc_advance(d, local_ns);
  if (reset || offset_ns > NTP_CLOCK_STEP_NS ||
      offset_ns < -NTP_CLOCK_STEP_NS) {
    // A step leaves the frequency and the outstanding phase as they were.
    d->base_net_ns += offset_ns;
    return true;
  }

  const int64_t now_s = local_ns / NS_PER_S;
  int64_t elapsed_s = d->have_ref ? now_s - d->ref_s : 0;
  if (elapsed_s > NTP_CLOCK_MAX_ELAPSED_S) {
    elapsed_s = NTP_CLOCK_MAX_ELAPSED_S;
  } else if (elapsed_s < 0) {
    elapsed_s = 0;
  }
  // error x 2^32 / 2^12 x elapsed, in ns/s x 2^32
  d->freq_q32 += offset_ns * (1LL << (32 - NTP_CLOCK_FREQ_SHIFT)) * elapsed_s;
  const int64_t max_q32 = NTP_CLOCK_MAX_FREQ_NS * (1LL << 32);
  if (d->freq_q32 > max_q32) {
    d->freq_q32 = max_q32;
  } else if (d->freq_q32 < -max_q32) {
    d->freq_q32 = -max_q32;
  }
  // The new measurement replaces what was left to slew; it does not add.
  d->offset_q32 = offset_ns * (1LL << 32);
  d->ref_s = now_s;
  d->have_ref = true;
  return false;
}

/* ---- Client ------------------------------------------------------------ */

// Startup: this many good exchanges back to back before the clock is locked.
#define NTP_BURST_EXCHANGES  3
#define NTP_BURST_TIMEOUT_US 500000LL
// Startup failures after which it falls back to the normal cadence (still
// unlocked until it has its exchanges).
#define NTP_BURST_MAX_FAILURES 64
// Then one exchange every 2-3 s, randomised.
#define NTP_INTERVAL_US        2000000LL
#define NTP_INTERVAL_JITTER_US 1000000U
#define NTP_SEND_RETRY_US      100000LL
// Minimum-delay gate over this many previous replies.
#define NTP_FILTER_SIZE 8
// Sanity bound on a round trip; anything longer is not a measurement.
#define NTP_MAX_ROUND_TRIP_NS 1000000000LL
// Longest single wait, so a stop is noticed promptly.
#define NTP_POLL_US   100000LL
#define NTP_STATUS_US 10000000LL

/* Frequency acquisition, which Apple's clock does not have.  Its loop pulls
 * in a frequency error only through the integrator, with a time constant of
 * minutes, so a crystal 30-50 ppm off holds a phase error of up to a couple
 * of milliseconds for the first minute or two (tests/ntp_sim).  For the first
 * NTP_FREQ_ACQUIRE_S after the first answer the frequency and phase are
 * instead set outright from a least-squares line through every reply,
 * weighted by round trip, much as ntpd measures the frequency directly in its
 * FREQ state (RFC 5905, 11.3); the PLL proper takes over after.  A fit is
 * used only once it is certain enough, so on bad Wi-Fi this changes little.
 * 0 gives Apple's behaviour alone. */
#ifndef NTP_FREQ_ACQUIRE
#define NTP_FREQ_ACQUIRE 1
#endif
#define NTP_FREQ_ACQUIRE_S          64
#define NTP_FREQ_ACQUIRE_MIN_SPAN_S 8
#define NTP_FREQ_POINTS             32
// Added to a reply's excess round trip before weighting by its inverse
// square: about the jitter of the fastest replies.
#define NTP_FREQ_ACQUIRE_DELAY_NS 200000.0
// A fit is used only with this many points and a slope this certain (ppb).
#define NTP_FREQ_ACQUIRE_MIN_POINTS 5
#define NTP_FREQ_ACQUIRE_MAX_SE_PPB 10000.0

/* Above lwIP's tcpip task (18).  A reply is stamped when select() returns
 * in this task, so below tcpip the stamp would also wait for whatever else
 * tcpip has queued, and that wait lands on one leg of the round trip.  The
 * priority is safe only because the task never holds the CPU for long: per
 * reply it does a few microseconds of integer arithmetic, plus, in the first
 * NTP_FREQ_ACQUIRE_S, a fit of a few hundred double operations (software
 * floating point on most of these chips); it sleeps in select() otherwise.
 * The 10 s status line is DEBUG for the same reason: a UART log line takes
 * milliseconds, which would hold tcpip off on this core.  Only the lock, a
 * step and errors log above DEBUG, a few lines per session.
 *
 * 4 KB of stack: a log line formats 64-bit integers through vprintf, and with
 * the web log viewer on, log_stream.c formats it again into a 256-byte buffer
 * of its own.  3 KB left too little margin for that on top of select(). */
#ifndef NTP_TASK_PRIORITY
#define NTP_TASK_PRIORITY 19
#endif
#define NTP_STACK_SIZE 4096

static struct {
  bool running;
  TaskHandle_t task_handle;
  int socket;
  struct sockaddr_in remote_addr;

  // The model, shared with readers under `lock`; only the task writes it.
  portMUX_TYPE lock;
  ntp_disc_t disc;
  bool locked;

  // Task-private from here on.
  uint64_t sent_origin; // transmit timestamp of the request in flight, or 0
  int64_t sent_local_us;
  int64_t next_send_us;
  bool burst;
  uint32_t good;
  uint32_t failures;
  uint32_t applied;
  bool refill;
  int64_t delays[NTP_FILTER_SIZE];
  int64_t offsets[NTP_FILTER_SIZE];
  uint32_t filter_index;
  // The best reply of the startup burst, used for the first step.
  int64_t startup_delay;
  int64_t startup_offset;
  int64_t startup_local;
  // Frequency acquisition: used measurements as (local, network - local).
  int64_t acquire_until_ns;
  uint32_t points;
  int64_t point_local[NTP_FREQ_POINTS];
  int64_t point_offset[NTP_FREQ_POINTS];
  int32_t point_delay[NTP_FREQ_POINTS];

  // For the status line.
  uint32_t sent, replies, used, steps, stale, timeouts;
  int64_t delay_avg, delay_min, delay_max;
  int64_t offset_avg, offset_min, offset_max;
  int64_t last_offset;
  int64_t start_us;
  int64_t status_us;
} ntp = {.socket = -1, .lock = portMUX_INITIALIZER_UNLOCKED};

static int64_t ntp_to_ns(uint64_t ts) {
  const uint64_t secs = ts >> 32;
  const uint64_t frac = ts & 0xFFFFFFFFULL;
  return (int64_t)(secs * (uint64_t)NS_PER_S) +
         (int64_t)((frac * (uint64_t)NS_PER_S) >> 32);
}

/* Only the upper 32 bits of the fraction exist, as in Apple's requests; the
 * round trip back through ntp_to_ns() loses under a nanosecond. */
static uint64_t ns_to_ntp(int64_t ns) {
  const uint64_t secs = (uint64_t)(ns / NS_PER_S);
  const uint64_t rest = (uint64_t)(ns % NS_PER_S);
  return (secs << 32) | ((rest * NTP_FRAC_ONE) / (uint64_t)NS_PER_S);
}

static uint64_t read_be64(const uint8_t *p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) {
    v = (v << 8) | p[i];
  }
  return v;
}

static void write_be64(uint8_t *p, uint64_t v) {
  for (int i = 7; i >= 0; i--) {
    p[i] = (uint8_t)v;
    v >>= 8;
  }
}

static int64_t clock_now_network(int64_t local_ns) {
  portENTER_CRITICAL(&ntp.lock);
  const int64_t net = disc_network_at(&ntp.disc, local_ns);
  portEXIT_CRITICAL(&ntp.lock);
  return net;
}

static int64_t interval_us(void) {
  return NTP_INTERVAL_US + (int64_t)(esp_random() % NTP_INTERVAL_JITTER_US);
}

static void send_request(void) {
  uint8_t req[TIMING_PACKET] = {0x80, TIMING_REQUEST, 0x00, 0x07};
  const int64_t now_us = esp_timer_get_time();
  // The disciplined time, so the reply measures what the clock has wrong.
  uint64_t ts = ns_to_ntp(clock_now_network(now_us * NS_PER_US));
  if (ts == 0) {
    ts = 1; // 0 means "nothing in flight"
  }
  write_be64(req + 24, ts);
  if (send(ntp.socket, req, sizeof(req), 0) != (ssize_t)sizeof(req)) {
    ESP_LOGD(TAG, "send failed: %d", errno);
    ntp.next_send_us = now_us + NTP_SEND_RETRY_US;
    return;
  }
  ntp.sent++;
  ntp.sent_origin = ts;
  ntp.sent_local_us = now_us;
  ntp.next_send_us =
      now_us + (ntp.burst ? NTP_BURST_TIMEOUT_US : interval_us());
}

static void stats_add(int64_t value, int64_t *avg, int64_t *min, int64_t *max) {
  if (ntp.replies == 1) {
    *avg = *min = *max = value;
    return;
  }
  *avg = (value + 15 * *avg) / 16;
  if (value < *min) {
    *min = value;
  }
  if (value > *max) {
    *max = value;
  }
}

/* Point i relative to now: weight, seconds, and ns of offset against the
 * newest reply's. */
static void freq_point(uint32_t i, int32_t min_delay, int64_t local_ns,
                       int64_t absolute_ns, double *w, double *x, double *y) {
  const double excess =
      (double)(ntp.point_delay[i] - min_delay) + NTP_FREQ_ACQUIRE_DELAY_NS;
  *w = 1.0 / (excess * excess);
  *x = (double)(ntp.point_local[i] - local_ns) / 1e9;
  *y = (double)(ntp.point_offset[i] - absolute_ns);
}

/* Frequency acquisition.  Every reply in the window is a point, weighted
 * by how close its round trip came to the shortest seen: the excess is what
 * a one-sided delay could have added to its offset, so a reply delayed 20 ms
 * counts for nothing and a fast one for most.  The weighted line through the
 * points gives the frequency, and its value now gives the phase, both set
 * outright.  Doubles are fine: once per exchange, on the timing task. */
static void freq_acquire(int64_t local_ns, int64_t absolute_ns,
                         int64_t delay_ns) {
  if (ntp.acquire_until_ns == 0) {
    ntp.acquire_until_ns = local_ns + NTP_FREQ_ACQUIRE_S * NS_PER_S;
  }
  if (!NTP_FREQ_ACQUIRE || local_ns >= ntp.acquire_until_ns ||
      ntp.points >= NTP_FREQ_POINTS) {
    return;
  }
  const uint32_t n = ntp.points++;
  ntp.point_local[n] = local_ns;
  ntp.point_offset[n] = absolute_ns;
  ntp.point_delay[n] = (int32_t)delay_ns; // under 1 s: checked on receipt
  if (ntp.applied == 0 ||
      local_ns - ntp.point_local[0] < NTP_FREQ_ACQUIRE_MIN_SPAN_S * NS_PER_S) {
    return;
  }

  int32_t min_delay = ntp.point_delay[0];
  for (uint32_t i = 1; i < ntp.points; i++) {
    if (ntp.point_delay[i] < min_delay) {
      min_delay = ntp.point_delay[i];
    }
  }
  /* Three passes over at most NTP_FREQ_POINTS, recomputing each point's
   * weight rather than keeping arrays of doubles on the task's stack. */
  double sw = 0, sx = 0, sy = 0, w, x, y;
  for (uint32_t i = 0; i < ntp.points; i++) {
    freq_point(i, min_delay, local_ns, absolute_ns, &w, &x, &y);
    sw += w;
    sx += w * x;
    sy += w * y;
  }
  const double mx = sx / sw, my = sy / sw;
  double sxy = 0, sxx = 0;
  for (uint32_t i = 0; i < ntp.points; i++) {
    freq_point(i, min_delay, local_ns, absolute_ns, &w, &x, &y);
    sxy += w * (x - mx) * (y - my);
    sxx += w * (x - mx) * (x - mx);
  }
  if (sxx <= 0 || ntp.points < NTP_FREQ_ACQUIRE_MIN_POINTS) {
    return;
  }
  double ppb = sxy / sxx; // ns/s
  /* Only a fit that knows the frequency better than the loop would: a few
   * fast replies over a short span on bad Wi-Fi can put the line hundreds
   * of ppm out.  Standard error from the weighted residuals. */
  double chi2 = 0;
  for (uint32_t i = 0; i < ntp.points; i++) {
    freq_point(i, min_delay, local_ns, absolute_ns, &w, &x, &y);
    const double r = (y - my) - ppb * (x - mx);
    chi2 += w * r * r;
  }
  const double se_ppb = sqrt(chi2 / (double)(ntp.points - 2) / sxx);
  if (se_ppb > NTP_FREQ_ACQUIRE_MAX_SE_PPB) {
    return;
  }
  if (ppb > (double)NTP_CLOCK_MAX_FREQ_NS) {
    ppb = (double)NTP_CLOCK_MAX_FREQ_NS;
  } else if (ppb < -(double)NTP_CLOCK_MAX_FREQ_NS) {
    ppb = -(double)NTP_CLOCK_MAX_FREQ_NS;
  }
  // The line at x = 0 (now), relative to this reply's own absolute offset.
  const double at_now = my - ppb * mx;

  portENTER_CRITICAL(&ntp.lock);
  const int64_t model = disc_network_at(&ntp.disc, local_ns) - local_ns;
  const int64_t phase = absolute_ns + (int64_t)at_now - model;
  ntp.disc.freq_q32 = (int64_t)(ppb * 4294967296.0);
  if (phase < NTP_CLOCK_STEP_NS && phase > -NTP_CLOCK_STEP_NS) {
    ntp.disc.offset_q32 = phase * (1LL << 32);
    ntp.disc.ref_s = local_ns / NS_PER_S;
    ntp.disc.have_ref = true;
  }
  portEXIT_CRITICAL(&ntp.lock);
}

/* Apple's filter: the first reply, and the first after a step, fill the
 * ring and are used; after that a reply is used only if its round trip is
 * no longer than any of the previous eight.  It goes into the ring either
 * way.  Returns whether it was used. */
static bool filter_and_apply(int64_t delay, int64_t offset, int64_t local_ns) {
  bool use = true;
  if (ntp.applied == 0 || ntp.refill) {
    for (int i = 0; i < NTP_FILTER_SIZE; i++) {
      ntp.delays[i] = delay;
      ntp.offsets[i] = offset;
    }
    ntp.refill = false;
  } else {
    for (int i = 0; i < NTP_FILTER_SIZE; i++) {
      if (delay > ntp.delays[i]) {
        use = false;
        break;
      }
    }
    ntp.delays[ntp.filter_index] = delay;
    ntp.offsets[ntp.filter_index] = offset;
    ntp.filter_index = (ntp.filter_index + 1U) % NTP_FILTER_SIZE;
  }
  if (!use) {
    return false;
  }

  portENTER_CRITICAL(&ntp.lock);
  const bool stepped =
      disc_adjust(&ntp.disc, local_ns, offset, ntp.applied == 0);
  ntp.disc.valid = true;
  portEXIT_CRITICAL(&ntp.lock);
  if (stepped) {
    if (ntp.applied > 0) {
      ntp.steps++;
      ESP_LOGW(TAG, "Clock stepped by %" PRId64 " us", offset / NS_PER_US);
      // A sender step breaks the line the fit follows; start it again.
      ntp.points = 0;
    }
    ntp.refill = true;
  }
  ntp.applied++;
  ntp.used++;
  return true;
}

// One reply, stamped at local_ns.  Returns true if it answered our request.
static bool process_reply(const uint8_t *p, size_t len, int64_t local_ns) {
  if (len < TIMING_PACKET || p[0] < 0x80 || p[0] > 0xBF ||
      p[1] != TIMING_RESPONSE) {
    return false;
  }
  // Only the answer to the request in flight: a duplicate, or a late answer
  // to an earlier one, would pair our new clock with an old send time.
  const uint64_t origin = read_be64(p + 8);
  if (ntp.sent_origin == 0 || origin != ntp.sent_origin) {
    ntp.stale++;
    return false;
  }
  ntp.sent_origin = 0;

  const int64_t t1 = ntp_to_ns(origin);
  const int64_t t2 = ntp_to_ns(read_be64(p + 16));
  const int64_t t3 = ntp_to_ns(read_be64(p + 24));
  const int64_t t4 = clock_now_network(local_ns);

  const int64_t round_trip = t4 - t1;
  if (round_trip < 0 || round_trip > NTP_MAX_ROUND_TRIP_NS) {
    ESP_LOGD(TAG, "Rejecting reply: round trip %" PRId64 " us",
             round_trip / NS_PER_US);
    return false;
  }
  const int64_t delay = round_trip - (t3 - t2);
  const int64_t offset = ((t2 - t1) + (t3 - t4)) / 2;

  ntp.replies++;
  stats_add(delay, &ntp.delay_avg, &ntp.delay_min, &ntp.delay_max);
  stats_add(offset, &ntp.offset_avg, &ntp.offset_min, &ntp.offset_max);
  // Where the sender's clock stands against the crystal, whatever the model.
  const int64_t absolute = t4 - local_ns + offset;
  if (!ntp.locked && ntp.applied == 0) {
    // Startup: the first step waits for the best of the burst.
    if (ntp.startup_delay == 0 || delay < ntp.startup_delay) {
      ntp.startup_delay = delay;
      ntp.startup_offset = offset;
      ntp.startup_local = local_ns;
    }
  } else if (filter_and_apply(delay, offset, local_ns)) {
    ntp.last_offset = offset;
  }
  freq_acquire(local_ns, absolute, delay);
  return true;
}

static void status_line(int64_t now_us) {
  if (now_us - ntp.status_us < NTP_STATUS_US) {
    return;
  }
  ntp.status_us = now_us;
  portENTER_CRITICAL(&ntp.lock);
  const int64_t freq_q32 = ntp.disc.freq_q32;
  const int64_t phase_q32 = ntp.disc.offset_q32;
  portEXIT_CRITICAL(&ntp.lock);
  // ppm with three decimals, in integers: ns/s is ppb.
  const int64_t freq_ppb = div_round(freq_q32 / (1LL << 16), 65536);
  const int64_t abs_ppb = freq_ppb < 0 ? -freq_ppb : freq_ppb;
  ESP_LOGD(TAG,
           "ntp: locked=%d freq=%c%" PRId64 ".%03" PRId64 " ppm last=%+" PRId64
           " us phase=%+" PRId64 " us delay min/avg/max="
           "%" PRId64 "/%" PRId64 "/%" PRId64 " us used=%" PRIu32 "/%" PRIu32
           " sent=%" PRIu32 " timeouts=%" PRIu32 " stale=%" PRIu32
           " steps=%" PRIu32,
           ntp.locked ? 1 : 0, freq_ppb < 0 ? '-' : '+', abs_ppb / 1000,
           abs_ppb % 1000, ntp.last_offset / NS_PER_US,
           div_round(phase_q32 / (1LL << 16), 65536) / NS_PER_US,
           ntp.delay_min / NS_PER_US, ntp.delay_avg / NS_PER_US,
           ntp.delay_max / NS_PER_US, ntp.used, ntp.replies, ntp.sent,
           ntp.timeouts, ntp.stale, ntp.steps);
}

// An exchange is over: answered, or its startup timeout ran out.
static void exchange_done(bool answered, int64_t now_us) {
  if (answered) {
    ntp.good++;
  }
  if (!ntp.locked && ntp.good >= NTP_BURST_EXCHANGES) {
    /* Apple steps on the first reply, delayed or not.  One that met a 50 ms
     * spike on one leg then takes a minute to slew out, so the step uses
     * the burst's shortest round trip instead; lock waits for the burst
     * either way. */
    (void)filter_and_apply(ntp.startup_delay, ntp.startup_offset,
                           ntp.startup_local);
    // That reply is the ring's yardstick already; the next one has to earn
    // its place like any other rather than refill the ring unchecked.
    ntp.refill = false;
    ntp.last_offset = ntp.startup_offset;
    portENTER_CRITICAL(&ntp.lock);
    ntp.locked = true;
    portEXIT_CRITICAL(&ntp.lock);
    ntp.burst = false;
    ESP_LOGI(TAG,
             "NTP timing locked after %" PRIu32 " exchanges in %" PRId64
             " ms, delay %" PRId64 " us",
             ntp.good + ntp.failures, (now_us - ntp.start_us) / 1000,
             ntp.delay_min / NS_PER_US);
    ntp.status_us = now_us; // first status line 10 s after lock
  }
  if (ntp.burst) {
    if (!answered && ++ntp.failures >= NTP_BURST_MAX_FAILURES) {
      ESP_LOGW(TAG,
               "No answer to %" PRIu32 " startup requests (%" PRIu32
               " good); slowing down",
               ntp.failures, ntp.good);
      ntp.burst = false;
    }
    ntp.next_send_us = now_us; // the next request goes straight out
  } else {
    ntp.next_send_us = now_us + interval_us();
  }
}

static void ntp_task(void *pvParameters) {
  uint8_t packet[64];

  while (ntp.running) {
    int64_t now_us = esp_timer_get_time();
    if (now_us >= ntp.next_send_us) {
      if (ntp.sent_origin != 0) {
        // The request in flight went unanswered.
        ntp.sent_origin = 0;
        ntp.timeouts++;
        exchange_done(false, now_us);
      }
      send_request();
    }

    int64_t wait_us = ntp.next_send_us - esp_timer_get_time();
    if (wait_us < 0) {
      wait_us = 0;
    } else if (wait_us > NTP_POLL_US) {
      wait_us = NTP_POLL_US;
    }
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(ntp.socket, &readable);
    struct timeval tv = {.tv_sec = 0, .tv_usec = (long)wait_us};
    const int ready = select(ntp.socket + 1, &readable, NULL, NULL, &tv);
    // Stamped before anything else: the earliest this task can see it.
    const int64_t rx_us = esp_timer_get_time();
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (ntp.running) {
        ESP_LOGE(TAG, "select error: %d", errno);
      }
      break;
    }
    if (ready > 0) {
      const ssize_t len = recv(ntp.socket, packet, sizeof(packet), 0);
      if (len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        if (ntp.running) {
          ESP_LOGE(TAG, "recv error: %d", errno);
        }
        break;
      }
      if (process_reply(packet, (size_t)len, rx_us * NS_PER_US)) {
        exchange_done(true, rx_us);
      }
    }

    // Keep the shared model wound to now, so readers rarely have to.
    now_us = esp_timer_get_time();
    portENTER_CRITICAL(&ntp.lock);
    disc_advance(&ntp.disc, now_us * NS_PER_US);
    portEXIT_CRITICAL(&ntp.lock);
    if (ntp.locked) {
      status_line(now_us);
    }
  }

  ntp.task_handle = NULL;
  vTaskDelete(NULL);
}

static bool ntp_wait_for_task_stopped(int timeout_ticks) {
  while (ntp.task_handle != NULL && timeout_ticks-- > 0) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return ntp.task_handle == NULL;
}

static void reset_state(void) {
  portENTER_CRITICAL(&ntp.lock);
  ntp.locked = false;
  disc_init(&ntp.disc, esp_timer_get_time() * NS_PER_US);
  portEXIT_CRITICAL(&ntp.lock);

  ntp.sent_origin = 0;
  ntp.next_send_us = 0;
  ntp.burst = true;
  ntp.good = ntp.failures = ntp.applied = 0;
  ntp.refill = false;
  ntp.filter_index = 0;
  ntp.points = 0;
  ntp.acquire_until_ns = 0;
  ntp.startup_delay = 0;
  memset(ntp.delays, 0, sizeof(ntp.delays));
  memset(ntp.offsets, 0, sizeof(ntp.offsets));
  ntp.sent = ntp.replies = ntp.used = ntp.steps = 0;
  ntp.stale = ntp.timeouts = 0;
  ntp.delay_avg = ntp.delay_min = ntp.delay_max = 0;
  ntp.offset_avg = ntp.offset_min = ntp.offset_max = 0;
  ntp.last_offset = 0;
  ntp.start_us = esp_timer_get_time();
  ntp.status_us = ntp.start_us;
}

esp_err_t ntp_clock_start_client(uint32_t remote_ip, uint16_t remote_port) {
  if (ntp.running) {
    // If already running to same target, keep going
    if (ntp.remote_addr.sin_addr.s_addr == remote_ip &&
        ntohs(ntp.remote_addr.sin_port) == remote_port) {
      return ESP_OK;
    }
    // Stop existing and restart with new target
    ntp_clock_stop();
    if (!ntp_wait_for_task_stopped(20)) {
      ESP_LOGE(TAG, "Previous NTP task did not stop");
      return ESP_ERR_INVALID_STATE;
    }
  }
  if (ntp.task_handle != NULL) {
    ESP_LOGW(TAG, "NTP task still stopping, waiting");
    if (!ntp_wait_for_task_stopped(20)) {
      ESP_LOGE(TAG, "NTP task still active");
      return ESP_ERR_INVALID_STATE;
    }
  }

  ntp.socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (ntp.socket < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %d", errno);
    return ESP_FAIL;
  }

  memset(&ntp.remote_addr, 0, sizeof(ntp.remote_addr));
  ntp.remote_addr.sin_family = AF_INET;
  ntp.remote_addr.sin_addr.s_addr = remote_ip;
  ntp.remote_addr.sin_port = htons(remote_port);
  // Connected, as Apple's is: only the sender's replies reach the socket.
  if (connect(ntp.socket, (struct sockaddr *)&ntp.remote_addr,
              sizeof(ntp.remote_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to connect timing socket: %d", errno);
    close(ntp.socket);
    ntp.socket = -1;
    return ESP_FAIL;
  }

  reset_state();
  ntp.running = true;

  ntp.task_handle = NULL;
  BaseType_t task_ret = xTaskCreate(ntp_task, "ntp_clock", NTP_STACK_SIZE, NULL,
                                    NTP_TASK_PRIORITY, &ntp.task_handle);
  if (task_ret != pdPASS || ntp.task_handle == NULL) {
    ESP_LOGE(TAG, "Failed to create NTP task");
    close(ntp.socket);
    ntp.socket = -1;
    ntp.running = false;
    return ESP_FAIL;
  }

  uint8_t *ip = (uint8_t *)&remote_ip;
  ESP_LOGI(TAG, "NTP timing client started -> %d.%d.%d.%d:%d", ip[0], ip[1],
           ip[2], ip[3], remote_port);
  return ESP_OK;
}

void ntp_clock_stop(void) {
  if (!ntp.running) {
    return;
  }

  ntp.running = false;

  if (ntp.socket >= 0) {
    close(ntp.socket);
    ntp.socket = -1;
  }

  if (ntp.task_handle) {
    if (!ntp_wait_for_task_stopped(20)) {
      ESP_LOGW(TAG, "NTP task did not exit within timeout");
    }
  }

  portENTER_CRITICAL(&ntp.lock);
  ntp.locked = false;
  ntp.disc.valid = false;
  portEXIT_CRITICAL(&ntp.lock);
  ESP_LOGI(TAG, "NTP timing stopped");
}

bool ntp_clock_is_locked(void) {
  return ntp.locked;
}

int64_t ntp_clock_get_offset_ns(void) {
  const int64_t local_ns = esp_timer_get_time() * NS_PER_US;
  portENTER_CRITICAL(&ntp.lock);
  const bool have = ntp.disc.valid;
  const int64_t net = have ? disc_network_at(&ntp.disc, local_ns) : local_ns;
  portEXIT_CRITICAL(&ntp.lock);
  return net - local_ns;
}

bool ntp_clock_local_to_network(int64_t local_ns, uint64_t *network_ns) {
  portENTER_CRITICAL(&ntp.lock);
  const bool locked = ntp.locked && ntp.disc.valid;
  const int64_t net = locked ? disc_network_at(&ntp.disc, local_ns) : 0;
  portEXIT_CRITICAL(&ntp.lock);
  if (locked && network_ns) {
    *network_ns = (uint64_t)net;
  }
  return locked;
}

bool ntp_clock_network_to_local(uint64_t network_ns, int64_t *local_ns) {
  portENTER_CRITICAL(&ntp.lock);
  const bool locked = ntp.locked && ntp.disc.valid;
  ntp_disc_t d = ntp.disc;
  portEXIT_CRITICAL(&ntp.lock);
  if (!locked) {
    return false;
  }
  /* The rate is within 1e-3 of 1, so two fixed-point passes leave well under
   * a nanosecond: guess from the base, then correct by what the guess maps
   * to. */
  int64_t guess = d.base_local_ns + ((int64_t)network_ns - d.base_net_ns);
  for (int i = 0; i < 2; i++) {
    guess += (int64_t)network_ns - disc_network_at(&d, guess);
  }
  if (local_ns) {
    *local_ns = guess;
  }
  return true;
}

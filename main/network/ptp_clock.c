// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

// gPTP slave modelled on Apple's receiver (see ptp_clock.h for the outline).
//
// Data flow, per SYNC from the selected master port:
//
//   SYNC (t2, stamped in lwIP's input hook) --> Delay_Req sent at once (t3)
//   FOLLOW_UP (t1)  --> one-way sample (t1, t2) --> passive estimator
//   Delay_Resp (t4) --> two-way sample          --> delay gate, ppm gate,
//                                                   two-way estimator
//
// Each estimator ("servo") filters the remote and the local timeline with a
// lag-compensated two-stage IIR and publishes an anchor + rate.  The clock
// uses the two-way servo whenever the master answers, the passive one
// otherwise.  All timestamps are int64 ns; filter state is kept relative to
// the last input so no product ever needs more than 64 bits, and the only
// floating point runs once per SYNC.

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ptp_clock.h"
#include "ptp_lwip_hook.h"
#include "spiram_task.h"

#ifdef ESP_PLATFORM
#include "lwip/pbuf.h"
#include "sdkconfig.h"
// esp_wifi_set_tx_done_cb(): the in-chip Wi-Fi driver's TX-done callback,
// used for t3 as Apple's stack uses its driver's transmit completion.  It
// is declared in ESP-IDF's private Wi-Fi header, not a public API, so it
// may change between IDF releases.  Chips without in-chip Wi-Fi (the
// ESP32-P4) do not have it.  The host tests define PTP_HAVE_TX_DONE
// themselves, with a mock driver.
#if defined(CONFIG_SOC_WIFI_SUPPORTED) && CONFIG_SOC_WIFI_SUPPORTED && \
    defined(CONFIG_ESP_WIFI_ENABLED) && CONFIG_ESP_WIFI_ENABLED
#include "esp_private/wifi.h"
#define PTP_HAVE_TX_DONE 1
#endif
#endif

static const char *TAG = "ptp_clock";

// Network-order IPv4 address for logs
#define PTP_IP_FMT "%u.%u.%u.%u"
#define PTP_IP_ARGS(ip)                                              \
  (unsigned)(ntohl(ip) >> 24), (unsigned)((ntohl(ip) >> 16) & 0xFF), \
      (unsigned)((ntohl(ip) >> 8) & 0xFF), (unsigned)(ntohl(ip) & 0xFF)

// PTP multicast addresses and ports
#define PTP_MULTICAST_ADDR "224.0.1.129"
#define PTP_EVENT_PORT     319
#define PTP_GENERAL_PORT   320

// PTP message types
#define PTP_MSG_SYNC       0x0
#define PTP_MSG_DELAY_REQ  0x1
#define PTP_MSG_FOLLOW_UP  0x8
#define PTP_MSG_DELAY_RESP 0x9
#define PTP_MSG_ANNOUNCE   0xB

// PTP header size and timestamp offset
#define PTP_HEADER_SIZE      34
#define PTP_TIMESTAMP_OFFSET 34
#define PTP_TIMESTAMP_SIZE   10
#define PTP_DELAY_REQ_LEN    44
#define PTP_DELAY_RESP_LEN   54
#define PTP_ANNOUNCE_LEN     64

// ---- Delay_Req -------------------------------------------------------------
// Apple sends one Delay_Req per accepted SYNC, at once, unicast with flags
// 0x0408 (unicastFlag | ptpTimescale), and logMessageInterval set to its own
// SYNC interval, -3.
#define PTP_DELAY_REQ_FLAGS        0x0408
#define PTP_DELAY_REQ_LOG_INTERVAL (-3)
#define PTP_LOCAL_PORT_NUMBER      1
// Delay_Reqs in a row without any Delay_Resp before the clock follows the
// master passively: 16 for a master that has never answered on this port,
// Apple's 80-SYNC gap (10 s) for one that has; and how often it then tries
// two-way again.
#define PTP_DREQ_UNANSWERED_LIMIT 16
#define PTP_DREQ_LOST_LIMIT       PTP_GAP_RESET_SYNCS
#define PTP_TWO_WAY_RETRY_MS      10000
// t3, as Apple takes it: when the driver reports the Delay_Req transmitted
// (esp_wifi_set_tx_done_cb(), the Wi-Fi driver's TX-done callback).  Where
// that is unavailable (Ethernet, a chip without in-chip Wi-Fi, or a driver
// that does not report our frame) t3 is a local read right after sendto(),
// bracketed by one before it; a bracket wider than this means the task was
// preempted around the send, and the exchange is dropped rather than biased
// by half the preemption -- the software analogue of Apple rejecting
// implausible TX timestamps.
#define PTP_TX_STAMP_MAX_NS 500000LL
// The two sources differ: the TX-done stamp comes after the driver's queue
// and the air time (retries included), the software one before them, so
// their t3s for the same frame differ by a variable few hundred us, which
// moves a midpoint by half that.  The two-way filter therefore takes t3 from
// one source only.  Once TX-done stamps arrive it uses them alone, and an
// exchange whose stamp is missing is dropped; after this many in a row the
// driver is taken to have stopped reporting (Wi-Fi was re-initialised, or
// the traffic moved to Ethernet) and the filter goes back to software t3.
// Each change of source re-seeds the filter.  The same run of misses, on
// either source, has the callback registered again.
#define PTP_TX_DONE_MISS_LIMIT 16
// Registration is retried at most this often while it fails (Wi-Fi not
// initialised), or after the driver stopped reporting.
#define PTP_TX_DONE_RETRY_MS 10000
// A TX-done callback frame longer than this cannot be a Delay_Req (44 bytes
// of PTP in UDP/IPv4 under an 802.11 or Ethernet header); not scanned.
#define PTP_TX_DONE_MAX_FRAME 200

// ---- Sync filter (Apple's kernel EtE filter) -------------------------------
// Gain 2^-shift per stage.  Shift and ppm bound add up per link end: Wi-Fi
// contributes +4 / +5000 ppm, Ethernet with hardware timestamps +2 / +100.
// This board is always Wi-Fi-like (software timestamps, even on W5500), the
// sender usually Wi-Fi: 8 and 10000 ppm, a 32 s time constant at 8 Hz.  A
// wired sender would allow less (6 / 5100) but we cannot tell it apart.
#define PTP_FILTER_SHIFT 8
#define PTP_PPM_LIMIT    10000
// Fast start.  Apple's reset pre-fills both timelines with the same nominal
// interval, i.e. a rate of exactly 1, and the double EWMA then learns the
// real rate with an offset transient of ~11 us per ppm that peaks after
// ~40 s: fine between two Apple TCXOs, ~0.5 ms with a 50 ppm ESP32
// crystal.  So for the first 256 SYNC intervals after a reset (32 s, one
// filter time constant) the line is a least-squares fit of offset and rate
// (offset only for the first 16, on the last known rate), and the IIR then
// starts from that fit's rate.  0 = Apple's behaviour exactly.
#ifndef PTP_FAST_START_SYNCS
#define PTP_FAST_START_SYNCS 256
#endif
#define PTP_FAST_START_RATE_SYNCS 16
// The fitted rate is weighed against what was known before (Bayes, both
// Gaussian): early fits over a few noisy samples would otherwise hand a
// rate error of hundreds of ppm to the extrapolation across a gap.  A new
// timeline knows only crystal tolerances (board + sender); a reset on the
// same timeline knows the rate it had.
#define PTP_RATE_PRIOR_NEW_PPM   30.0
#define PTP_RATE_PRIOR_KNOWN_PPM 2.0
// Consecutive ppm-gate drops before the filter is reset onto the next
// sample ("Allowing through"): 24 at 8 Hz, ~3 s.  An iPhone's clock steps
// back when it sleeps; this is what follows it.
#define PTP_PPM_DROP_LIMIT 24
// A SYNC sequence gap this large resets; smaller ones are bridged by
// interpolation, each synthesised interval within +-10 % of nominal.
#define PTP_GAP_RESET_SYNCS        80
#define PTP_GAP_INTERVAL_TOLERANCE 10 // percent
// Propagation delay limit: whole samples above it are dropped.  During the
// first 480 samples after a reset it adapts every 80 samples to
// max(median of the last 80 delays, 10 ms).
#define PTP_DELAY_LIMIT_NS      10000000LL
#define PTP_DELAY_WINDOW        80
#define PTP_DELAY_ADAPT_SAMPLES 480
// Lucky-packet selection (not Apple's: its kernel only runs one as a shadow
// experiment).  A two-way sample's midpoint error is at most half its
// round trip's excess over the floor, and with software timestamps on
// Wi-Fi that excess is often milliseconds, so the filter is fed only the
// exchanges whose delay is within the lowest quarter of the last 80 (plus
// a little slack).  The ppm gate still sees every sample, so a clock step
// is still followed after 24 SYNCs.  100 = feed every sample, as Apple.
#ifndef PTP_LUCKY_PERCENTILE
#define PTP_LUCKY_PERCENTILE 25
#endif
#define PTP_LUCKY_SLACK_NS 50000LL
#define PTP_LUCKY_EVERY    16 // samples between threshold updates
// Nominal SYNC interval bounds.  The interval is measured per port from
// successive preciseOriginTimestamps, seeded by logMessageInterval; a
// sender whose real rate differs from what it advertises would otherwise
// fail every +-10 % gap check.
#define PTP_MIN_INTERVAL_NS     7812500LL    // 2^-7 s
#define PTP_MAX_INTERVAL_NS     1000000000LL // 1 s
#define PTP_DEFAULT_INTERVAL_NS 125000000LL

// ---- Passive fallback -------------------------------------------------------
// One-way samples carry the path delay.  Every window of SYNCs the least
// delayed one (largest residual) goes to the filter.  The window grows
// 2, 2, 4, 8, 16 after a reset so the clock locks in ~1 s.
#define PTP_PASSIVE_WINDOW       16
#define PTP_PASSIVE_FIRST_WINDOW 2
#define PTP_PASSIVE_GATE_NS      2000000LL
// A window whose best sample is late by more than the gate is congestion or
// the master's clock moving back.  A fall larger than congestion causes
// (500 ms) is followed after 16 SYNCs, a smaller one only after 160, longer
// than the congestion bursts seen on the board.  An early sample cannot be
// congestion and is followed at once.
#define PTP_PASSIVE_BIG_STEP_NS      500000000LL
#define PTP_PASSIVE_BIG_STEP_SYNCS   16
#define PTP_PASSIVE_SMALL_STEP_SYNCS 160
// While two-way works, the passive estimate's offset from it (the one-way
// delay floor) is learnt, and removed again after a fallback.  It is kept
// until reboot: the path is mostly the board's own Wi-Fi hop, so the floor
// learnt from one master is the best guess for the next.  Before anything
// is learnt, a typical floor is assumed [inferred: an AP relays the frame,
// two short hops of airtime plus queueing].
#define PTP_PASSIVE_BIAS_MIN_SAMPLES 8
#define PTP_PASSIVE_BIAS_SHIFT       4
#define PTP_PASSIVE_DELAY_PRIOR_NS   500000LL
// Two-way samples after a reset before the two-way line is preferred over a
// calibrated passive one.
#define PTP_TWO_WAY_SETTLE_SAMPLES  32
#define PTP_PASSIVE_SETTLED_SAMPLES 8

// ---- Grandmaster / ports ----------------------------------------------------
#define PTP_MAX_PORTS            8
#define PTP_MAX_PEERS            16
#define PTP_ANNOUNCE_TIMEOUT_MS  10000 // 10 x the 1 s announce interval
#define PTP_PORT_SYNC_TIMEOUT_MS 3000

// ---- Lock ------------------------------------------------------------------
// Two-way exchanges a (re)seed chooses its seed from, and samples after it
// before the clock reports locked: ~1 s from the first SYNC.
#define PTP_SEED_SAMPLES             6
#define MIN_SAMPLES_FOR_LOCK         2
#define MIN_PASSIVE_SAMPLES_FOR_LOCK 2
// Apple bridges a SYNC gap of up to 80 intervals (10 s) on the last anchor
// and rate; the lock lasts as long without a SYNC.
#define LOCK_TIMEOUT_MS 10000
// Below this a resume needs nothing: the model kept running.
#define PTP_LONG_PAUSE_THRESHOLD_MS 30000
// A sample newer than this means the master is still sending, so the next
// session can carry on with the lock rather than spend ~0.5 s re-acquiring it.
#define SESSION_KEEP_MS 2000

#define PTP_STATUS_INTERVAL_MS 10000
#define PTP_TASK_PRIORITY      7 // above the decoder (6), below playback (9)
#define PTP_TASK_STACK         4608

// Fixed point: filter state in 2^-16 ns, rates as (ratio - 1) in 2^-40.
#define PTP_IIR_FRAC  16
#define PTP_RATE_FRAC 40

// ---- RX timestamp ring (lwIP input hook -> PTP task) -----------------------
#define PTP_RX_RING 16

typedef struct {
  uint32_t ver; // odd while being written
  uint32_t ip;  // network order
  uint32_t us_lo;
  uint32_t us_hi;
  uint16_t seq;
  uint8_t type;
} ptp_rx_stamp_t;

static ptp_rx_stamp_t rx_ring[PTP_RX_RING];
static uint32_t rx_ring_head;

// ---- Types -----------------------------------------------------------------

typedef struct {
  uint8_t type;
  uint8_t transport;
  uint8_t domain;
  int8_t log_interval;
  uint16_t flags;
  uint16_t length;
  uint16_t seq;
  int64_t correction_ns;
  uint64_t clock_id;
  uint8_t spid[10]; // sourcePortIdentity
} ptp_hdr_t;

// One IIR timeline.  Values are relative to `base`, which is moved to the
// latest input before every update.
typedef struct {
  int64_t base; // ns
  int64_t y1;   // Q16 ns
  int64_t y2;   // Q16 ns: stage-1 lag, = interval * (2^shift - 1)
} ptp_iir_t;

typedef struct {
  ptp_iir_t r, l;
  int64_t anchor_r; // filtered point, ns
  int64_t anchor_l;
  int64_t dev_q;     // remote per local - 1, Q40
  int64_t inv_dev_q; // local per remote - 1, Q40
  int64_t nominal_ns;
  int64_t resid_ms; // EWMA of residual^2, ns^2
  int64_t last_r;   // last input
  int64_t last_l;
  uint32_t samples; // real samples since the last reset
  uint32_t last_accept_ms;
  uint16_t last_seq;
  uint16_t drops; // consecutive gate drops
  // Fast start: least-squares sums of y = (R - r0) - (L - l0) on x = L - l0
  double fs_sx, fs_sy, fs_sxx, fs_sxy, fs_syy;
  double fs_prior; // rate (ratio - 1) known at the seed
  bool rate_known;
  int64_t fs_r0, fs_l0;
  uint32_t fs_n;
  uint32_t fs_syncs;
  bool fast_start;
  bool seeded;
  bool locked;
} ptp_servo_t;

typedef enum {
  FEED_OK,
  FEED_STALE,
  FEED_RESET_GAP,
  FEED_RESET_BOUNDS,
} ptp_feed_t;

typedef struct {
  uint64_t gm;
  uint64_t clock_id;
  uint32_t ip;
  uint32_t last_sync_ms;
  uint32_t last_announce_ms;
  int64_t interval_ns; // measured SYNC interval, 0 = unknown
  int64_t last_t1;
  int64_t pending_t2;
  uint32_t syncs;
  uint16_t ann_variance;
  uint16_t steps_removed;
  uint16_t last_t1_seq;
  uint16_t pending_seq;
  uint16_t time_base;
  uint8_t interval_misses;
  uint8_t spid[10];
  uint8_t ann_p1, ann_class, ann_accuracy, ann_p2;
  uint8_t transport;
  uint8_t domain;
  int8_t log_interval;
  bool used;
  bool has_announce;
  bool has_apple_gm;
  bool has_time_base;
  bool pending;
  bool has_last_t1;
  uint64_t apple_gm;
} ptp_port_t;

typedef enum {
  PTP_MODE_PROBING, // sending Delay_Req, no answer yet
  PTP_MODE_TWO_WAY,
  PTP_MODE_PASSIVE,
} ptp_mode_t;

// Log events gathered under the mux and printed after it is released.
enum {
  EV_LOCKED = 1 << 0,
  EV_GM_CHANGE = 1 << 1,
  EV_PASSIVE = 1 << 2,
  EV_TWO_WAY = 1 << 3,
  EV_RESET_DROPS = 1 << 4,
  EV_RESET_GAP = 1 << 5,
  EV_RESET_BOUNDS = 1 << 6,
  EV_PASSIVE_STEP = 1 << 7,
  EV_TIME_BASE = 1 << 8,
  EV_PORT_CHANGE = 1 << 9,
  EV_APPLE_GM_MISMATCH = 1 << 10,
  EV_ANSWERS_STOPPED = 1 << 11,
};

typedef struct {
  uint32_t events;
  uint64_t gm_old, gm_new;
  uint32_t port_ip;
  int64_t step_ns;
  // Delay_Req to send once the mux is released
  bool send_dreq;
  uint32_t dreq_ip;
  uint16_t dreq_seq;
  uint8_t dreq[PTP_DELAY_REQ_LEN];
  bool need_median;
  bool need_lucky;
  bool lock_two_way;
  uint64_t lock_gm;
  uint32_t lock_syncs;
} ptp_actions_t;

static struct {
  // Published model (the output servo's anchor and rate)
  int64_t model_l;
  int64_t model_r;
  int64_t model_dev_q;
  int64_t model_inv_dev_q;
  int64_t raw_offset_ns;
  uint64_t gm_id; // timeline of the model (0 = none)
  bool model_valid;

  uint64_t expected_clock_id;   // GM named by the anchor (0 = BMCA)
  uint32_t expected_gm_seen_ms; // last time a live port carried it
  // The anchor's GM was left for BMCA (handover or silence); stays set until
  // the next anchor names a GM, so the pin does not re-assert and flap.
  bool expected_released;
  ptp_servo_t tw; // two-way estimator
  ptp_servo_t ow; // passive (one-way) estimator

  // Passive front end
  int64_t win_best_r, win_best_l, win_best_res;
  int64_t run_best_r, run_best_l, run_best_res;
  int64_t passive_bias_ns;
  uint32_t passive_bias_n;
  uint16_t win_best_seq, run_best_seq;
  uint16_t win_n, win_size;
  uint16_t run_syncs;
  bool win_have;

  // Delay exchange with the selected port
  int64_t x_t1, x_t2, x_t3, x_t4;
  uint32_t x_sync_ms;
  uint16_t x_sync_seq;
  uint16_t x_dreq_seq;
  uint16_t dreq_seq;
  bool x_active, x_have_t1, x_dreq_sent, x_have_t3, x_have_t4;
  ptp_mode_t mode;
  uint32_t unanswered;
  uint32_t last_probe_ms;

  // Two-way seed candidates
  int64_t seed_r, seed_l, seed_delay;
  uint32_t seed_ms;
  uint16_t seed_seq;
  uint16_t seed_n;

  // Delay statistics (rate-corrected one-way path delay, ns)
  int32_t delays[PTP_DELAY_WINDOW];
  uint32_t delay_n; // since the last reset
  uint32_t delay_total;
  int64_t delay_limit_ns;
  int64_t lucky_limit_ns;
  uint32_t lucky_skips;

  ptp_port_t ports[PTP_MAX_PORTS];
  int selected; // index into ports, -1 = none
  uint32_t peers[PTP_MAX_PEERS];
  size_t peer_count;
  uint8_t local_identity[8];

  TaskHandle_t task_handle;
  int event_socket;
  int general_socket;
  uint32_t lock_start_ms;
  uint32_t last_sync_rx_ms; // last SYNC from the selected port
  uint32_t last_status_ms;

  // Statistics
  uint32_t sync_count;
  uint32_t followup_count;
  uint32_t announce_count;
  uint32_t rejected_master_count; // SYNC from a port not on the chosen GM
  uint32_t dreq_sent;
  uint32_t dresp_received;
  uint32_t last_dresp_ms;  // last answer on the selected port (0 = none)
  uint32_t tx_slow;        // exchanges dropped for a wide t3 bracket
  uint32_t tx_done_used;   // exchanges whose t3 came from the TX-done callback
  uint32_t tx_done_lost;   // exchanges dropped for a missing TX-done stamp
  uint32_t tx_done_misses; // completed exchanges in a row without one
  bool t3_tx_done;         // the two-way filter is on TX-done t3
  bool tx_done_retry;      // the driver stopped reporting: register again
  int64_t x_t3_pre;        // local time just before sendto() of x_dreq_seq
  uint32_t dreq_tx_errors;
  uint32_t superseded;
  uint32_t out_of_order;
  uint32_t ppm_drops;
  uint32_t delay_drops;
  uint32_t passive_drops;
  uint32_t resets_drops;
  uint32_t resets_bounds;
  uint32_t resets_gap;
  uint32_t gm_changes;
  uint32_t rx_hook_stamps;
  uint32_t rx_task_stamps;
  uint32_t status_syncs; // SYNCs seen since the last status line
  spiram_task_mem_t task_mem;

  bool running;
  bool locked;
  // The session that named expected_clock_id has ended. The filter stays on,
  // so no other master's samples reach the kept lock, but the next session's
  // master is checked against the samples again.
  bool session_ended;
  bool have_identity;
} ptp = {0};

// PTP packets and RTSP session changes run on different tasks (and cores).
// Accepting a sample and deciding whether its lock can be kept must be one
// coherent transition. Only clock state and bounded packet parsing belong
// under this mux; logs, sockets, sorting and task lifecycle stay outside.
static portMUX_TYPE ptp_state_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- Small helpers ---------------------------------------------------------

static inline int64_t get_local_time_ns(void) {
  return (int64_t)esp_timer_get_time() * 1000LL;
}

static inline uint32_t now_ms(void) {
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static inline int64_t abs64(int64_t v) {
  return v < 0 ? -v : v;
}

// a * q / 2^40, rounded, exact for any operands whose result fits in 64
// bits.  RV32 has no __int128, so the 128-bit product is built from four
// 32x32 multiplies.
static int64_t mul_q40(int64_t a, int64_t q) {
  const bool neg = (a < 0) != (q < 0);
  const uint64_t ua = a < 0 ? (uint64_t)0 - (uint64_t)a : (uint64_t)a;
  const uint64_t uq = q < 0 ? (uint64_t)0 - (uint64_t)q : (uint64_t)q;
  const uint64_t a_lo = ua & 0xffffffffu, a_hi = ua >> 32;
  const uint64_t q_lo = uq & 0xffffffffu, q_hi = uq >> 32;
  const uint64_t ll = a_lo * q_lo;
  const uint64_t lh = a_lo * q_hi;
  const uint64_t hl = a_hi * q_lo;
  const uint64_t mid = (ll >> 32) + (lh & 0xffffffffu) + (hl & 0xffffffffu);
  uint64_t lo = (ll & 0xffffffffu) | (mid << 32);
  uint64_t hi = a_hi * q_hi + (lh >> 32) + (hl >> 32) + (mid >> 32);
  const uint64_t rounded = lo + (1ULL << (PTP_RATE_FRAC - 1));
  if (rounded < lo) {
    hi++;
  }
  lo = rounded;
  const uint64_t r = (hi << (64 - PTP_RATE_FRAC)) | (lo >> PTP_RATE_FRAC);
  return neg ? -(int64_t)r : (int64_t)r;
}

// Arithmetic shift right with rounding to nearest.
static inline int64_t round_shift(int64_t v, unsigned s) {
  return (v + ((int64_t)1 << (s - 1))) >> s;
}

static uint32_t isqrt64(uint64_t v) {
  uint64_t r = 0;
  uint64_t bit = 1ULL << 62;
  while (bit > v) {
    bit >>= 2;
  }
  while (bit != 0) {
    if (v >= r + bit) {
      v -= r + bit;
      r = (r >> 1) + bit;
    } else {
      r >>= 1;
    }
    bit >>= 2;
  }
  return (uint32_t)r;
}

static uint64_t read_be(const uint8_t *p, int n) {
  uint64_t v = 0;
  for (int i = 0; i < n; i++) {
    v = (v << 8) | p[i];
  }
  return v;
}

// 48-bit seconds + 32-bit nanoseconds
static int64_t parse_ptp_timestamp_ns(const uint8_t *data) {
  return (int64_t)(read_be(data, 6) * 1000000000ULL + read_be(data + 6, 4));
}

static bool parse_header(const uint8_t *data, size_t len, ptp_hdr_t *h) {
  if (len < PTP_HEADER_SIZE || (data[1] & 0x0F) != 2) {
    return false;
  }
  h->type = data[0] & 0x0F;
  h->transport = data[0] >> 4;
  h->length = (uint16_t)read_be(data + 2, 2);
  h->domain = data[4];
  h->flags = (uint16_t)read_be(data + 6, 2);
  // correctionField: signed, 2^-16 ns (IEEE 1588 11.4.4.2.1)
  h->correction_ns = (int64_t)read_be(data + 8, 8) / 65536;
  memcpy(h->spid, data + 20, sizeof(h->spid));
  h->clock_id = read_be(data + 20, 8);
  h->seq = (uint16_t)read_be(data + 30, 2);
  h->log_interval = (int8_t)data[33];
  return true;
}

static int64_t clamp_interval(int64_t ns) {
  if (ns < PTP_MIN_INTERVAL_NS) {
    return PTP_MIN_INTERVAL_NS;
  }
  return ns > PTP_MAX_INTERVAL_NS ? PTP_MAX_INTERVAL_NS : ns;
}

static int64_t log_interval_ns(int8_t log) {
  if (log < -7 || log > 0) {
    return PTP_DEFAULT_INTERVAL_NS;
  }
  return 1000000000LL >> -log;
}

// ---- RX timestamps ---------------------------------------------------------

// Called for every IPv4 packet in lwIP's input path.  Only a SYNC to UDP 319
// is recorded; everything else costs a few compares.
static void rx_stamp_ip4(const uint8_t *ip, size_t len, int64_t now_us) {
  if (len < 20 || (ip[0] >> 4) != 4 || ip[9] != 17) {
    return;
  }
  const size_t ihl = (size_t)(ip[0] & 0x0F) * 4;
  // A fragment's UDP header is not where we look; PTP is never fragmented.
  if (ihl < 20 || (read_be(ip + 6, 2) & 0x3FFF) != 0 ||
      len < ihl + 8 + PTP_HEADER_SIZE) {
    return;
  }
  const uint8_t *udp = ip + ihl;
  if (read_be(udp + 2, 2) != PTP_EVENT_PORT) {
    return;
  }
  const uint8_t *msg = udp + 8;
  if ((msg[0] & 0x0F) != PTP_MSG_SYNC) {
    return;
  }
  // Single writer (the tcpip thread): a per-slot sequence lock lets the PTP
  // task read without blocking it.
  // Only this thread moves the head, so no read-modify-write is needed (and
  // none is emulated on cores without atomics).
  const uint32_t idx = rx_ring_head++ % PTP_RX_RING;
  ptp_rx_stamp_t *e = &rx_ring[idx];
  const uint32_t v = __atomic_load_n(&e->ver, __ATOMIC_RELAXED);
  __atomic_store_n(&e->ver, v + 1, __ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_RELEASE);
  uint32_t src;
  memcpy(&src, ip + 12, 4);
  __atomic_store_n(&e->ip, src, __ATOMIC_RELAXED);
  __atomic_store_n(&e->seq, (uint16_t)read_be(msg + 30, 2), __ATOMIC_RELAXED);
  __atomic_store_n(&e->type, (uint8_t)PTP_MSG_SYNC, __ATOMIC_RELAXED);
  __atomic_store_n(&e->us_lo, (uint32_t)now_us, __ATOMIC_RELAXED);
  __atomic_store_n(&e->us_hi, (uint32_t)((uint64_t)now_us >> 32),
                   __ATOMIC_RELAXED);
  __atomic_store_n(&e->ver, v + 2, __ATOMIC_RELEASE);
}

// ---- TX-done timestamps -----------------------------------------------------

// The last Delay_Req the driver reported transmitted.  Single writer (the
// Wi-Fi driver's task), read without blocking it through a sequence lock.
typedef struct {
  uint32_t ver; // odd while being written
  uint16_t seq;
  uint32_t us_lo;
  uint32_t us_hi;
} ptp_tx_stamp_t;

static ptp_tx_stamp_t tx_slot;

// Find our own Delay_Req in a transmitted frame and record when it left.
// The callback's buffer layout is not documented (802.11 or Ethernet
// framing), so the message is found by content -- our sourcePortIdentity
// at offset 20 of a version-2 Delay_Req -- rather than by header offsets.
static void tx_done_scan(const uint8_t *data, size_t len, int64_t now_us) {
  if (data == NULL || len < PTP_DELAY_REQ_LEN || len > PTP_TX_DONE_MAX_FRAME ||
      !ptp.have_identity) {
    return;
  }
  for (size_t i = 20; i + 10 + (PTP_DELAY_REQ_LEN - 30) <= len; i++) {
    if (data[i] != ptp.local_identity[0] ||
        memcmp(data + i, ptp.local_identity, 8) != 0 ||
        read_be(data + i + 8, 2) != PTP_LOCAL_PORT_NUMBER) {
      continue;
    }
    const uint8_t *msg = data + i - 20;
    if ((msg[0] & 0x0F) != PTP_MSG_DELAY_REQ || (msg[1] & 0x0F) != 2) {
      continue;
    }
    const uint32_t v = __atomic_load_n(&tx_slot.ver, __ATOMIC_RELAXED);
    __atomic_store_n(&tx_slot.ver, v + 1, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&tx_slot.seq, (uint16_t)read_be(msg + 30, 2),
                     __ATOMIC_RELAXED);
    __atomic_store_n(&tx_slot.us_lo, (uint32_t)now_us, __ATOMIC_RELAXED);
    __atomic_store_n(&tx_slot.us_hi, (uint32_t)((uint64_t)now_us >> 32),
                     __ATOMIC_RELAXED);
    __atomic_store_n(&tx_slot.ver, v + 2, __ATOMIC_RELEASE);
    return;
  }
}

// The TX-done time of Delay_Req `seq`, if the driver reported it.
static bool tx_done_stamp_ns(uint16_t seq, int64_t *ns) {
  const uint32_t v1 = __atomic_load_n(&tx_slot.ver, __ATOMIC_ACQUIRE);
  if (v1 == 0 || (v1 & 1)) {
    return false;
  }
  const uint16_t s = __atomic_load_n(&tx_slot.seq, __ATOMIC_RELAXED);
  const uint32_t lo = __atomic_load_n(&tx_slot.us_lo, __ATOMIC_RELAXED);
  const uint32_t hi = __atomic_load_n(&tx_slot.us_hi, __ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  if (__atomic_load_n(&tx_slot.ver, __ATOMIC_RELAXED) != v1 || s != seq) {
    return false;
  }
  *ns = (int64_t)(((uint64_t)hi << 32) | lo) * 1000LL;
  return true;
}

#ifdef PTP_HAVE_TX_DONE
static void ptp_wifi_tx_done(uint8_t ifidx, uint8_t *data, uint16_t *data_len,
                             bool tx_ok) {
  (void)ifidx;
  if (tx_ok && data_len != NULL) {
    tx_done_scan(data, *data_len, esp_timer_get_time());
  }
}

// Only the PTP task registers, and ptp_clock_stop() unregisters once that
// task has exited.
static bool tx_done_registered;
static uint32_t tx_done_attempt_ms;
static bool tx_done_attempted;
#endif

// Register the TX-done callback, before a Delay_Req is sent.  It fails until
// esp_wifi_init() has run and is then retried every 10 s, as it is when the
// driver stopped reporting our frames: esp_wifi_deinit() may have dropped
// the registration.  The driver has a single slot; nothing else in this
// firmware uses it.
static void ensure_tx_done_cb(uint32_t now) {
#ifdef PTP_HAVE_TX_DONE
  portENTER_CRITICAL(&ptp_state_mux);
  const bool retry = ptp.tx_done_retry;
  ptp.tx_done_retry = false;
  portEXIT_CRITICAL(&ptp_state_mux);
  if (retry) {
    tx_done_registered = false;
  }
  if (tx_done_registered ||
      (tx_done_attempted && now - tx_done_attempt_ms < PTP_TX_DONE_RETRY_MS)) {
    return;
  }
  tx_done_attempted = true;
  tx_done_attempt_ms = now;
  tx_done_registered = esp_wifi_set_tx_done_cb(ptp_wifi_tx_done) == ESP_OK;
#else
  (void)now;
#endif
}

// Empty the driver's slot.  It checks the slot for NULL before calling it
// (ESP-IDF 5.5's Wi-Fi library), so NULL unregisters.
static void tx_done_unregister(void) {
#ifdef PTP_HAVE_TX_DONE
  if (tx_done_registered) {
    (void)esp_wifi_set_tx_done_cb(NULL);
    tx_done_registered = false;
  }
  tx_done_attempted = false;
#endif
}

#ifdef ESP_PLATFORM
int ptp_lwip_ip4_input_hook(struct pbuf *p, struct netif *inp) {
  (void)inp;
  if (p != NULL && p->payload != NULL) {
    rx_stamp_ip4((const uint8_t *)p->payload, p->len, esp_timer_get_time());
  }
  return 0; // never consume
}
#endif

// The receive time of a SYNC: the hook's stamp when it has one for this
// packet, else the task-context time (no hook, or the ring wrapped).
static int64_t rx_timestamp_ns(const uint8_t *data, size_t len, uint32_t src_ip,
                               int64_t task_now_ns, bool *from_hook) {
  *from_hook = false;
  if (len < PTP_HEADER_SIZE || (data[0] & 0x0F) != PTP_MSG_SYNC) {
    return task_now_ns;
  }
  const uint16_t seq = (uint16_t)read_be(data + 30, 2);
  for (int i = 0; i < PTP_RX_RING; i++) {
    const ptp_rx_stamp_t *e = &rx_ring[i];
    const uint32_t v1 = __atomic_load_n(&e->ver, __ATOMIC_ACQUIRE);
    if (v1 == 0 || (v1 & 1)) {
      continue;
    }
    const uint32_t ip = __atomic_load_n(&e->ip, __ATOMIC_RELAXED);
    const uint16_t s = __atomic_load_n(&e->seq, __ATOMIC_RELAXED);
    const uint8_t type = __atomic_load_n(&e->type, __ATOMIC_RELAXED);
    const uint32_t lo = __atomic_load_n(&e->us_lo, __ATOMIC_RELAXED);
    const uint32_t hi = __atomic_load_n(&e->us_hi, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    if (__atomic_load_n(&e->ver, __ATOMIC_RELAXED) != v1) {
      continue;
    }
    if (ip != src_ip || s != seq || type != PTP_MSG_SYNC) {
      continue;
    }
    const int64_t ns = (int64_t)(((uint64_t)hi << 32) | lo) * 1000LL;
    // Same packet only if it is recent and not from the future: sequence
    // numbers repeat every 2^16 SYNCs.
    if (ns <= task_now_ns && task_now_ns - ns < 1000000000LL) {
      *from_hook = true;
      return ns;
    }
  }
  return task_now_ns;
}

// ---- IIR filter ------------------------------------------------------------

// Pre-fill with an interval: stage 2 holds the steady lag of stage 1 on a
// ramp of that slope, so the output starts at x with zero lag.
static void iir_reset(ptp_iir_t *f, int64_t x, int64_t interval_ns,
                      unsigned shift) {
  f->base = x;
  f->y2 = interval_ns * (((int64_t)1 << shift) - 1) * (1LL << PTP_IIR_FRAC);
  f->y1 = -f->y2;
}

// Move the reference to x so every later value stays small.
static void iir_rebase(ptp_iir_t *f, int64_t x) {
  const int64_t d = x - f->base;
  f->base = x;
  f->y1 -= d * (1LL << PTP_IIR_FRAC);
}

// One step with input x_rel (ns relative to base).  Stage 1 follows x, stage
// 2 the residual stage 1 leaves; their sum has no lag on a ramp.
static void iir_step(ptp_iir_t *f, int64_t x_rel, unsigned shift) {
  const int64_t xq = x_rel * (1LL << PTP_IIR_FRAC);
  f->y1 += round_shift(xq - f->y1, shift);
  f->y2 += round_shift((xq - f->y1) - f->y2, shift);
}

static int64_t iir_out_ns(const ptp_iir_t *f) {
  return f->base + round_shift(f->y1 + f->y2, PTP_IIR_FRAC);
}

// ---- Servo -----------------------------------------------------------------

static int64_t servo_predict(const ptp_servo_t *s, int64_t local_ns) {
  const int64_t dl = local_ns - s->anchor_l;
  return s->anchor_r + dl + mul_q40(dl, s->dev_q);
}

static void servo_publish_rate(ptp_servo_t *s, double dev) {
  s->dev_q = (int64_t)(dev * (double)(1LL << PTP_RATE_FRAC));
  s->inv_dev_q = (int64_t)(-dev / (1.0 + dev) * (double)(1LL << PTP_RATE_FRAC));
}

// Forget everything but, when keep_lock, the lock and the rate: the next
// sample is the seed.  Used for steps on the same timeline.
static void servo_unseed(ptp_servo_t *s, bool keep_lock) {
  const bool locked = keep_lock && s->locked;
  const int64_t dev_q = keep_lock ? s->dev_q : 0;
  const int64_t inv_q = keep_lock ? s->inv_dev_q : 0;
  const bool rate_known = keep_lock && s->rate_known;
  const bool seeded = keep_lock && s->seeded;
  const int64_t ar = s->anchor_r, al = s->anchor_l;
  const uint32_t accepted = s->last_accept_ms;
  memset(s, 0, sizeof(*s));
  // A kept lock keeps converting on the last anchor until the next sample.
  s->locked = locked;
  s->dev_q = dev_q;
  s->inv_dev_q = inv_q;
  s->rate_known = rate_known;
  if (seeded) {
    s->anchor_r = ar;
    s->anchor_l = al;
    s->last_accept_ms = accepted;
  }
}

// Seed the filter on (r, l).  A same-timeline reset keeps the rate it had
// learnt.
static void servo_seed(ptp_servo_t *s, int64_t r, int64_t l, uint16_t seq,
                       int64_t nominal_ns, uint32_t sync_ms) {
  s->nominal_ns = nominal_ns;
  s->anchor_r = r;
  s->anchor_l = l;
  s->last_seq = seq;
  s->samples = 1;
  s->drops = 0;
  s->resid_ms = 0;
  s->last_accept_ms = sync_ms;
  s->seeded = true;
  s->last_r = r;
  s->last_l = l;
  s->fast_start = PTP_FAST_START_SYNCS > 0;
  s->fs_r0 = r;
  s->fs_l0 = l;
  s->fs_n = 1;
  s->fs_syncs = 0;
  s->fs_sx = s->fs_sy = s->fs_sxx = s->fs_sxy = s->fs_syy = 0;
  s->fs_prior = (double)s->dev_q / (double)(1LL << PTP_RATE_FRAC);
  if (!s->fast_start) {
    // Pre-fill with the nominal interval: stage 2 holds the steady lag of
    // stage 1 on a ramp, so the output starts at the sample with no lag.
    iir_reset(&s->r, r, nominal_ns + mul_q40(nominal_ns, s->dev_q),
              PTP_FILTER_SHIFT);
    iir_reset(&s->l, l, nominal_ns, PTP_FILTER_SHIFT);
  }
}

// Fast start: add (r, l) to the fit and move the anchor and rate to it.
// Returns false when the fitted rate is out of bounds.
static bool fast_start_add(ptp_servo_t *s, int64_t r, int64_t l, uint16_t k) {
  const double x = (double)(l - s->fs_l0);
  const double y = (double)((r - s->fs_r0) - (l - s->fs_l0));
  s->fs_n++;
  s->fs_syncs += k;
  s->fs_sx += x;
  s->fs_sy += y;
  s->fs_sxx += x * x;
  s->fs_sxy += x * y;
  s->fs_syy += y * y;
  const double n = (double)s->fs_n;
  double dev = s->fs_prior;
  if (s->fs_syncs >= PTP_FAST_START_RATE_SYNCS && s->fs_n > 3) {
    const double sxx = s->fs_sxx - s->fs_sx * s->fs_sx / n;
    const double sxy = s->fs_sxy - s->fs_sx * s->fs_sy / n;
    const double syy = s->fs_syy - s->fs_sy * s->fs_sy / n;
    if (sxx > 0) {
      const double fit = sxy / sxx;
      double sse = syy - fit * sxy;
      if (sse < 0) {
        sse = 0;
      }
      // Variance of the fitted rate; a floor keeps a perfect fit finite.
      const double var_fit = (sse / (n - 2) + 1.0) / sxx;
      const double ppm =
          s->rate_known ? PTP_RATE_PRIOR_KNOWN_PPM : PTP_RATE_PRIOR_NEW_PPM;
      const double var_prior = ppm * ppm * 1e-12;
      dev = (fit * var_prior + s->fs_prior * var_fit) / (var_prior + var_fit);
    }
  }
  if (dev > PTP_PPM_LIMIT * 1e-6 || dev < -PTP_PPM_LIMIT * 1e-6) {
    return false;
  }
  const double c = (s->fs_sy - dev * s->fs_sx) / n;
  servo_publish_rate(s, dev);
  s->anchor_l = l;
  s->anchor_r = s->fs_r0 + (l - s->fs_l0) + (int64_t)(c + dev * x);
  if (s->fs_syncs >= PTP_FAST_START_SYNCS) {
    // Hand over to the IIR, pre-filled with the fitted rate so it starts
    // in its steady state on this line.
    s->fast_start = false;
    s->rate_known = true;
    iir_reset(&s->r, s->anchor_r,
              s->nominal_ns + mul_q40(s->nominal_ns, s->dev_q),
              PTP_FILTER_SHIFT);
    iir_reset(&s->l, s->anchor_l, s->nominal_ns, PTP_FILTER_SHIFT);
  }
  return true;
}

static void servo_step(ptp_servo_t *s, int64_t dr, int64_t dl) {
  iir_step(&s->r, dr, PTP_FILTER_SHIFT);
  iir_step(&s->l, dl, PTP_FILTER_SHIFT);
}

// Add an accepted sample.  Gaps are bridged by interpolated samples so the
// filter always advances one nominal interval per SYNC.  A reset leaves the
// servo unseeded (lock, rate and last anchor kept: same timeline) for the
// caller to seed from its next good sample.
static ptp_feed_t servo_feed(ptp_servo_t *s, int64_t r, int64_t l, uint16_t seq,
                             int64_t nominal_ns, uint32_t sync_ms) {
  const uint16_t k = (uint16_t)(seq - s->last_seq);
  if (k == 0 || k >= 0x8000) {
    return FEED_STALE;
  }
  if (k >= PTP_GAP_RESET_SYNCS) {
    servo_unseed(s, true);
    return FEED_RESET_GAP;
  }
  // Inputs relative to the previous input.
  const int64_t prev_r = s->last_r, prev_l = s->last_l;
  const int64_t dr = r - prev_r;
  const int64_t dl = l - prev_l;
  if (k > 1) {
    const int64_t ir = dr / k, il = dl / k;
    const int64_t tol = s->nominal_ns * PTP_GAP_INTERVAL_TOLERANCE / 100;
    if (abs64(ir - s->nominal_ns) > tol || abs64(il - s->nominal_ns) > tol) {
      servo_unseed(s, true);
      return FEED_RESET_BOUNDS;
    }
  }
  s->last_r = r;
  s->last_l = l;
  if (s->fast_start) {
    // A fit needs no interpolation: the gap is just a longer x step.
    s->last_seq = seq;
    s->nominal_ns = nominal_ns;
    if (!fast_start_add(s, r, l, k)) {
      servo_unseed(s, true);
      s->dev_q = 0;
      s->rate_known = false;
      s->inv_dev_q = 0;
      return FEED_RESET_BOUNDS;
    }
    s->samples++;
    s->last_accept_ms = sync_ms;
    return FEED_OK;
  }
  iir_rebase(&s->r, prev_r);
  iir_rebase(&s->l, prev_l);
  for (uint16_t j = 1; j < k; j++) {
    servo_step(s, dr * j / k, dl * j / k);
  }
  servo_step(s, dr, dl);
  iir_rebase(&s->r, r);
  iir_rebase(&s->l, l);
  s->last_seq = seq;
  s->nominal_ns = nominal_ns;

  // Rate from the slope both filters have settled on (stage 2 holds the
  // interval times 2^shift - 1 on each timeline, so their ratio is the
  // rate).  This runs once per SYNC, where soft-float is affordable.
  const double dev =
      s->l.y2 > 0 ? (double)(s->r.y2 - s->l.y2) / (double)s->l.y2 : 1.0;
  if (dev > PTP_PPM_LIMIT * 1e-6 || dev < -PTP_PPM_LIMIT * 1e-6) {
    // Reset; the rate starts again from nominal.
    servo_unseed(s, true);
    s->dev_q = 0;
    s->rate_known = false;
    s->inv_dev_q = 0;
    return FEED_RESET_BOUNDS;
  }
  servo_publish_rate(s, dev);
  s->anchor_r = iir_out_ns(&s->r);
  s->anchor_l = iir_out_ns(&s->l);
  s->samples++;
  s->last_accept_ms = sync_ms;
  return FEED_OK;
}

// ---- Model / lock
// ------------------------------------------------------------

// A servo reset on the same timeline stays usable on its last anchor until
// the next sample re-seeds it.
static bool servo_usable(const ptp_servo_t *s) {
  return s->locked;
}

// The servo the clock reads: two-way whenever the master answers.
// Right after two-way resumes from passive (a gap reset) the passive line,
// once calibrated against two-way, is better than a few fresh two-way
// samples, so it stays in use until the two-way fit has settled.  It is not
// preferred while it is itself off its line (a step it has not followed).
static const ptp_servo_t *output_servo(void) {
  const bool tw = servo_usable(&ptp.tw);
  const bool ow = servo_usable(&ptp.ow);
  if (ptp.mode == PTP_MODE_PASSIVE) {
    return ow ? &ptp.ow : tw ? &ptp.tw : NULL;
  }
  if (tw && ptp.tw.samples < PTP_TWO_WAY_SETTLE_SAMPLES && ow &&
      ptp.ow.samples >= PTP_PASSIVE_SETTLED_SAMPLES && ptp.run_syncs == 0 &&
      ptp.passive_bias_n >= PTP_PASSIVE_BIAS_MIN_SAMPLES &&
      now_ms() - ptp.ow.last_accept_ms < PTP_PORT_SYNC_TIMEOUT_MS &&
      ptp.ow.seeded) {
    return &ptp.ow;
  }
  return tw ? &ptp.tw : ow ? &ptp.ow : NULL;
}

// Caller holds ptp_state_mux.
static void publish_model(ptp_actions_t *act) {
  const ptp_servo_t *s = output_servo();
  if (s == NULL) {
    return;
  }
  ptp.model_valid = true;
  ptp.model_l = s->anchor_l;
  ptp.model_r = s->anchor_r;
  ptp.model_dev_q = s->dev_q;
  ptp.model_inv_dev_q = s->inv_dev_q;
  if (s == &ptp.ow) {
    ptp.model_r -= ptp.passive_bias_n >= PTP_PASSIVE_BIAS_MIN_SAMPLES
                       ? ptp.passive_bias_ns
                       : -PTP_PASSIVE_DELAY_PRIOR_NS;
  }
  if (!ptp.locked) {
    ptp.locked = true;
    ptp.lock_start_ms = now_ms();
    if (act) {
      act->events |= EV_LOCKED;
      act->lock_gm = ptp.gm_id;
      act->lock_two_way = s == &ptp.tw;
      act->lock_syncs = ptp.sync_count;
    }
  }
}

// Caller holds ptp_state_mux.
static int64_t model_predict(int64_t local_ns) {
  const int64_t dl = local_ns - ptp.model_l;
  return ptp.model_r + dl + mul_q40(dl, ptp.model_dev_q);
}

// Caller holds ptp_state_mux.  The lock lasts while the master sends: SYNCs
// held up by congestion are dropped by the delay gate but still show the
// master is there, and the model extrapolates on its rate meanwhile.
static bool check_locked(void) {
  if (ptp.locked) {
    const ptp_servo_t *s = output_servo();
    if (s == NULL || now_ms() - ptp.last_sync_rx_ms > LOCK_TIMEOUT_MS) {
      ptp.locked = false;
      ptp.lock_start_ms = 0;
    }
  }
  return ptp.locked;
}

// The lucky threshold describes the path, not the filter: a reset on the
// same port keeps it.
static void reset_delay_stats(void) {
  ptp.delay_n = 0;
  ptp.delay_limit_ns = PTP_DELAY_LIMIT_NS;
  ptp.seed_n = 0;
}

static void reset_passive_window(void) {
  ptp.win_n = 0;
  ptp.win_have = false;
  ptp.win_size = PTP_PASSIVE_FIRST_WINDOW;
  ptp.run_syncs = 0;
}

// Caller holds ptp_state_mux.  Drop both estimators and the lock: a new
// timeline, or a session that cannot use the old one.
static void reset_timeline(void) {
  servo_unseed(&ptp.tw, false);
  servo_unseed(&ptp.ow, false);
  ptp.locked = false;
  ptp.lock_start_ms = 0;
  ptp.model_valid = false;
  ptp.x_active = false;
  reset_delay_stats();
  reset_passive_window();
}

// Caller holds ptp_state_mux.  Same timeline, new estimate: the lock and the
// last model stay until the next sample re-seeds.
static void soft_reset(void) {
  servo_unseed(&ptp.tw, true);
  servo_unseed(&ptp.ow, true);
  reset_delay_stats();
  reset_passive_window();
}

// ---- Ports and grandmaster -------------------------------------------------

static uint64_t port_gm(const ptp_port_t *p, uint32_t now) {
  if (p->has_announce && now - p->last_announce_ms < PTP_ANNOUNCE_TIMEOUT_MS) {
    return p->gm;
  }
  if (p->has_apple_gm) {
    return p->apple_gm;
  }
  return p->clock_id;
}

// A port carries GM `gm` when that is what it announces (or, without a fresh
// Announce, what its Apple TLV names or its own identity).  Its own identity
// alone is not enough: a GM that hands over to another device keeps sending
// SYNCs from the same port, now as a boundary clock carrying the *new* GM's
// timeline, and only its Announce says so.
static bool port_on_gm(const ptp_port_t *p, uint64_t gm, uint32_t now) {
  return gm != 0 && port_gm(p, now) == gm;
}

static bool peer_allowed(uint32_t ip) {
  if (ptp.peer_count == 0) {
    return true;
  }
  for (size_t i = 0; i < ptp.peer_count; i++) {
    if (ptp.peers[i] == ip) {
      return true;
    }
  }
  return false;
}

static bool port_live(const ptp_port_t *p, uint32_t now) {
  return p->used && p->syncs > 0 &&
         now - p->last_sync_ms < PTP_PORT_SYNC_TIMEOUT_MS &&
         peer_allowed(p->ip);
}

// IEEE 1588 dataset comparison on what an Announce carries, lower is
// better: priority1, class, accuracy, variance, priority2, then (when
// `identity`) GM identity and steps removed.  A port without a current
// Announce ranks last.
static int bmca_compare(const ptp_port_t *a, const ptp_port_t *b, uint32_t now,
                        bool identity) {
  const bool ha =
      a->has_announce && now - a->last_announce_ms < PTP_ANNOUNCE_TIMEOUT_MS;
  const bool hb =
      b->has_announce && now - b->last_announce_ms < PTP_ANNOUNCE_TIMEOUT_MS;
  if (ha != hb) {
    return ha ? -1 : 1;
  }
  if (ha) {
    const uint64_t va = ((uint64_t)a->ann_p1 << 32) |
                        ((uint64_t)a->ann_class << 24) |
                        ((uint64_t)a->ann_accuracy << 16) | a->ann_variance;
    const uint64_t vb = ((uint64_t)b->ann_p1 << 32) |
                        ((uint64_t)b->ann_class << 24) |
                        ((uint64_t)b->ann_accuracy << 16) | b->ann_variance;
    if (va != vb) {
      return va < vb ? -1 : 1;
    }
    if (a->ann_p2 != b->ann_p2) {
      return a->ann_p2 < b->ann_p2 ? -1 : 1;
    }
  }
  if (!identity) {
    return 0;
  }
  const uint64_t ga = port_gm(a, now), gb = port_gm(b, now);
  if (ga != gb) {
    return ga < gb ? -1 : 1;
  }
  if (a->steps_removed != b->steps_removed) {
    return a->steps_removed < b->steps_removed ? -1 : 1;
  }
  return 0;
}

// Rank ports on the chosen GM: its own (direct) port first, then fewer
// steps removed.  Relayed ports are the fallback.
static int port_rank(const ptp_port_t *p, uint64_t gm, uint32_t now) {
  if (p->clock_id == gm) {
    return 0;
  }
  const bool ann =
      p->has_announce && now - p->last_announce_ms < PTP_ANNOUNCE_TIMEOUT_MS;
  return 1 + (ann ? p->steps_removed : 255);
}

// Caller holds ptp_state_mux.  Choose the GM (anchor's, else BMCA) and the
// port to measure it through.  A GM change starts a new timeline; a port
// change on the same GM keeps the filter (same timeline, other path).
static void select_port(ptp_actions_t *act) {
  const uint32_t now = now_ms();
  uint64_t gm = ptp.expected_clock_id;
  if (gm != 0 && ptp.expected_released) {
    gm = 0;
  } else if (gm != 0 && ptp.gm_id == gm) {
    // The anchor's GM is a preference, not a pin, as in Apple's kernel, which
    // always follows BMCA; the audio side sees the timeline change through
    // ptp_clock_network_to_local() and carries its anchor across.  Leave it
    //  - at once when the GM itself announces another GM (a HomePod or Apple
    //    TV took over mid-session), or
    //  - when no live port has carried it for an Announce receipt timeout,
    //    so a few seconds of missing SYNCs do not flip the timeline.
    bool carried = false, abdicated = false;
    for (int i = 0; i < PTP_MAX_PORTS; i++) {
      const ptp_port_t *p = &ptp.ports[i];
      if (!port_live(p, now)) {
        continue;
      }
      if (port_on_gm(p, gm, now)) {
        carried = true;
        // Time the silence from the GM's last SYNC, like Apple's Announce
        // receipt timeout, not from when its port stopped counting as live.
        if ((int32_t)(p->last_sync_ms - ptp.expected_gm_seen_ms) > 0) {
          ptp.expected_gm_seen_ms = p->last_sync_ms;
        }
      } else if (p->clock_id == gm) {
        abdicated = true;
      }
    }
    if (!carried && (abdicated || now - ptp.expected_gm_seen_ms >=
                                      PTP_ANNOUNCE_TIMEOUT_MS)) {
      gm = 0;
      ptp.expected_released = true;
    }
  }
  if (gm == 0) {
    // The best GM by BMCA; the current one stays while it ranks as well
    // (same priority vector), so equal masters do not take turns.
    int best = -1, current = -1;
    for (int i = 0; i < PTP_MAX_PORTS; i++) {
      const ptp_port_t *p = &ptp.ports[i];
      if (!port_live(p, now)) {
        continue;
      }
      if (best < 0 || bmca_compare(p, &ptp.ports[best], now, true) < 0) {
        best = i;
      }
      if (ptp.gm_id != 0 && port_on_gm(p, ptp.gm_id, now) &&
          (current < 0 ||
           bmca_compare(p, &ptp.ports[current], now, true) < 0)) {
        current = i;
      }
    }
    gm = ptp.gm_id;
    if (best >= 0 &&
        (current < 0 ||
         bmca_compare(&ptp.ports[best], &ptp.ports[current], now, false) < 0)) {
      gm = port_gm(&ptp.ports[best], now);
    }
  }

  int chosen = -1;
  int chosen_rank = 0;
  for (int i = 0; i < PTP_MAX_PORTS; i++) {
    const ptp_port_t *p = &ptp.ports[i];
    if (!port_live(p, now) || !port_on_gm(p, gm, now)) {
      continue;
    }
    int rank = port_rank(p, gm, now) * 2;
    if (i != ptp.selected) {
      rank++; // hysteresis: keep the current port on a tie
    }
    if (chosen < 0 || rank < chosen_rank) {
      chosen = i;
      chosen_rank = rank;
    }
  }

  if (gm != ptp.gm_id && chosen >= 0) {
    if (ptp.gm_id != 0) {
      act->events |= EV_GM_CHANGE;
      act->gm_old = ptp.gm_id;
      act->gm_new = gm;
      ptp.gm_changes++;
    }
    reset_timeline();
    ptp.gm_id = gm;
  }
  if (chosen != ptp.selected) {
    if (chosen >= 0 && ptp.selected >= 0) {
      act->events |= EV_PORT_CHANGE;
      act->port_ip = ptp.ports[chosen].ip;
    }
    ptp.selected = chosen;
    ptp.x_active = false;
    ptp.mode = PTP_MODE_PROBING;
    ptp.unanswered = 0;
    ptp.last_dresp_ms = 0;
    ptp.last_probe_ms = 0;
    ptp.delay_total = 0;
    ptp.lucky_limit_ns = INT64_MAX;
    reset_delay_stats();
    reset_passive_window();
  }
}

static uint32_t port_heard(const ptp_port_t *p) {
  const uint32_t now = now_ms();
  return now - p->last_sync_ms < now - p->last_announce_ms
             ? p->last_sync_ms
             : p->last_announce_ms;
}

// Find or allocate the port for (ip, sourcePortIdentity).
static ptp_port_t *port_lookup(uint32_t ip, const ptp_hdr_t *h, bool create) {
  int free_slot = -1, oldest = -1;
  const uint32_t now = now_ms();
  for (int i = 0; i < PTP_MAX_PORTS; i++) {
    ptp_port_t *p = &ptp.ports[i];
    if (!p->used) {
      if (free_slot < 0) {
        free_slot = i;
      }
      continue;
    }
    if (p->ip == ip && memcmp(p->spid, h->spid, sizeof(p->spid)) == 0) {
      return p;
    }
    if (i != ptp.selected &&
        (oldest < 0 ||
         now - port_heard(p) > now - port_heard(&ptp.ports[oldest]))) {
      oldest = i;
    }
  }
  if (!create) {
    return NULL;
  }
  const int slot = free_slot >= 0 ? free_slot : oldest;
  if (slot < 0) {
    return NULL;
  }
  ptp_port_t *p = &ptp.ports[slot];
  memset(p, 0, sizeof(*p));
  p->used = true;
  p->ip = ip;
  memcpy(p->spid, h->spid, sizeof(p->spid));
  p->clock_id = h->clock_id;
  p->last_sync_ms = now;
  p->last_announce_ms = now - PTP_ANNOUNCE_TIMEOUT_MS;
  return p;
}

static int64_t port_nominal(const ptp_port_t *p) {
  return p->interval_ns > 0 ? p->interval_ns
                            : clamp_interval(log_interval_ns(p->log_interval));
}

// ---- Delay measurement -----------------------------------------------------

static void build_delay_req(uint8_t *out, uint8_t transport, uint8_t domain,
                            uint16_t seq) {
  memset(out, 0, PTP_DELAY_REQ_LEN);
  out[0] = (uint8_t)((transport << 4) | PTP_MSG_DELAY_REQ);
  out[1] = 2; // versionPTP
  out[3] = PTP_DELAY_REQ_LEN;
  out[4] = domain;
  out[6] = (uint8_t)(PTP_DELAY_REQ_FLAGS >> 8);
  out[7] = (uint8_t)PTP_DELAY_REQ_FLAGS;
  memcpy(out + 20, ptp.local_identity, 8);
  out[28] = (uint8_t)(PTP_LOCAL_PORT_NUMBER >> 8);
  out[29] = (uint8_t)PTP_LOCAL_PORT_NUMBER;
  out[30] = (uint8_t)(seq >> 8);
  out[31] = (uint8_t)seq;
  out[32] = 1; // controlField: Delay_Req
  out[33] = (uint8_t)(int8_t)PTP_DELAY_REQ_LOG_INTERVAL;
  // originTimestamp stays 0, which IEEE 1588 allows.
}

// Caller holds ptp_state_mux.  One-way sample (t1, t2) for the passive
// estimator.  The least delayed SYNC of each window is the one fed; runs of
// windows far off the line are told apart as described at the constants.
static void passive_sample(int64_t t1, int64_t t2, uint16_t seq,
                           int64_t nominal_ns, uint32_t sync_ms,
                           ptp_actions_t *act) {
  ptp_servo_t *s = &ptp.ow;
  const int64_t res = s->seeded ? t1 - servo_predict(s, t2) : t1 - t2;
  if (!ptp.win_have || res > ptp.win_best_res) {
    ptp.win_have = true;
    ptp.win_best_res = res;
    ptp.win_best_r = t1;
    ptp.win_best_l = t2;
    ptp.win_best_seq = seq;
  }
  // A sample far off the line ends its window at once, so a step is timed
  // in SYNCs, not windows.
  const bool far = s->seeded && (res > PTP_PASSIVE_GATE_NS ||
                                 res < -PTP_PASSIVE_BIG_STEP_NS);
  if (++ptp.win_n < ptp.win_size && !far) {
    return;
  }
  const int64_t br = ptp.win_best_r, bl = ptp.win_best_l;
  const int64_t bres = ptp.win_best_res;
  const uint16_t bseq = ptp.win_best_seq;
  const uint16_t window = ptp.win_n;
  ptp.win_n = 0;
  ptp.win_have = false;
  if (ptp.win_size < PTP_PASSIVE_WINDOW && s->samples > 0) {
    ptp.win_size *= 2;
  }

  if (!s->seeded) {
    servo_seed(s, br, bl, bseq, nominal_ns, sync_ms);
    ptp.run_syncs = 0;
    return;
  }
  if (bres > PTP_PASSIVE_GATE_NS) {
    // Earlier than any delay explains: the line is low (seeded on a delayed
    // SYNC) or the master stepped forward.
    servo_seed(s, br, bl, bseq, nominal_ns, sync_ms);
    ptp.run_syncs = 0;
    ptp.passive_drops += window;
    return;
  }
  if (bres < -PTP_PASSIVE_GATE_NS) {
    if (ptp.run_syncs == 0 || bres > ptp.run_best_res) {
      ptp.run_best_res = bres;
      ptp.run_best_r = br;
      ptp.run_best_l = bl;
      ptp.run_best_seq = bseq;
    }
    ptp.run_syncs += window;
    ptp.passive_drops += window;
    const uint32_t needed = -ptp.run_best_res >= PTP_PASSIVE_BIG_STEP_NS
                                ? PTP_PASSIVE_BIG_STEP_SYNCS
                                : PTP_PASSIVE_SMALL_STEP_SYNCS;
    if (ptp.run_syncs >= needed) {
      act->events |= EV_PASSIVE_STEP;
      act->step_ns = ptp.run_best_res;
      servo_seed(s, ptp.run_best_r, ptp.run_best_l, ptp.run_best_seq,
                 nominal_ns, sync_ms);
      ptp.run_syncs = 0;
    }
    return;
  }
  ptp.run_syncs = 0;
  const ptp_feed_t fr = servo_feed(s, br, bl, bseq, nominal_ns, sync_ms);
  if (fr == FEED_STALE) {
    return;
  }
  if (s->samples >= MIN_PASSIVE_SAMPLES_FOR_LOCK) {
    s->locked = true;
  }
  // Learn the one-way delay floor while two-way is trustworthy.
  if (fr == FEED_OK && ptp.mode == PTP_MODE_TWO_WAY && servo_usable(&ptp.tw) &&
      ptp.tw.drops == 0 && ptp.tw.samples >= 32) {
    const int64_t b = s->anchor_r - servo_predict(&ptp.tw, s->anchor_l);
    if (ptp.passive_bias_n == 0) {
      ptp.passive_bias_ns = b;
    } else {
      ptp.passive_bias_ns +=
          (b - ptp.passive_bias_ns) >> PTP_PASSIVE_BIAS_SHIFT;
    }
    ptp.passive_bias_n++;
  }
}

// Caller holds ptp_state_mux.  Complete exchange: gate and feed the two-way
// estimator.
static void two_way_sample(const ptp_port_t *port, ptp_actions_t *act) {
  const int64_t t1 = ptp.x_t1, t2 = ptp.x_t2, t3 = ptp.x_t3, t4 = ptp.x_t4;
  const uint32_t sync_ms = ptp.x_sync_ms;
  ptp.x_active = false;
  if (t4 <= t1 || t3 < t2 || t4 - t1 > 1000000000LL) {
    ptp.out_of_order++;
    return;
  }
  ptp_servo_t *s = &ptp.tw;

  // Path delay, rate-corrected: the remote span in local units, less the
  // local turnaround, halved.  Negative (timestamp noise) counts as 0.
  const int64_t remote_span = t4 - t1;
  const int64_t scaled = remote_span + mul_q40(remote_span, s->inv_dev_q);
  int64_t delay = (scaled - (t3 - t2)) / 2;
  if (delay < 0) {
    delay = 0;
  }
  ptp.delays[ptp.delay_total % PTP_DELAY_WINDOW] =
      delay > INT32_MAX ? INT32_MAX : (int32_t)delay;
  ptp.delay_total++;
  ptp.delay_n++;
  if (ptp.delay_n <= PTP_DELAY_ADAPT_SAMPLES &&
      ptp.delay_n % PTP_DELAY_WINDOW == 0) {
    act->need_median = true;
  }
  if (ptp.delay_n % PTP_LUCKY_EVERY == 0) {
    act->need_lucky = true;
  }
  if (delay > ptp.delay_limit_ns) {
    ptp.delay_drops++;
    return;
  }

  // Midpoints: with a symmetric path they are simultaneous, so the offset
  // needs no delay subtraction.
  const int64_t r = t1 + (t4 - t1) / 2;
  const int64_t l = t2 + (t3 - t2) / 2;
  const int64_t nominal = port_nominal(port);
  ptp.raw_offset_ns = r - l;

  if (!s->seeded) {
    // Seed on the exchange with the shortest delay of the next few: one
    // sample far off would otherwise gate out the good ones after it.
    if (ptp.seed_n == 0 || delay < ptp.seed_delay) {
      ptp.seed_delay = delay;
      ptp.seed_r = r;
      ptp.seed_l = l;
      ptp.seed_seq = ptp.x_sync_seq;
      ptp.seed_ms = sync_ms;
    }
    if (++ptp.seed_n >= PTP_SEED_SAMPLES) {
      servo_seed(s, ptp.seed_r, ptp.seed_l, ptp.seed_seq, nominal, ptp.seed_ms);
      ptp.seed_n = 0;
    }
    return;
  }
  const int64_t dl = l - s->anchor_l;
  if (dl <= 0) {
    ptp.out_of_order++;
    return;
  }
  const int64_t resid = r - servo_predict(s, l);
  const int64_t tol = (dl * PTP_PPM_LIMIT + 999999) / 1000000;
  if (abs64(resid) > tol) {
    ptp.ppm_drops++;
    // Only a normal round trip makes a far-off midpoint evidence that the
    // master's clock moved; past the 10 ms base limit (admitted only by
    // the adaptive limit) it is congestion.
    if (delay > PTP_DELAY_LIMIT_NS) {
      return;
    }
    ++s->drops;
    // The last few of a run are the seed candidates if it reaches the
    // limit: those after a step are all on the new line.
    if (s->drops > PTP_PPM_DROP_LIMIT - PTP_SEED_SAMPLES &&
        (ptp.seed_n == 0 || delay < ptp.seed_delay)) {
      ptp.seed_n = 1;
      ptp.seed_delay = delay;
      ptp.seed_r = r;
      ptp.seed_l = l;
      ptp.seed_seq = ptp.x_sync_seq;
      ptp.seed_ms = sync_ms;
    }
    if (s->drops < PTP_PPM_DROP_LIMIT) {
      return;
    }
    // Persistent, with a normal round trip: the master's clock moved.
    // Allow it through: reset onto the best of the last few.
    act->events |= EV_RESET_DROPS;
    act->step_ns = resid;
    ptp.resets_drops++;
    const int64_t sr = ptp.seed_r, sl = ptp.seed_l;
    const uint16_t sseq = ptp.seed_seq;
    const uint32_t sms = ptp.seed_ms;
    servo_unseed(s, true);
    reset_delay_stats();
    servo_seed(s, sr, sl, sseq, nominal, sms);
    return;
  }
  ptp.seed_n = 0;
  s->drops = 0;
  if (delay > ptp.lucky_limit_ns) {
    ptp.lucky_skips++; // agrees with the line, but a noisy one: bridged
    return;
  }
  const ptp_feed_t fr = servo_feed(s, r, l, ptp.x_sync_seq, nominal, sync_ms);
  if (fr == FEED_RESET_GAP) {
    ptp.resets_gap++;
    act->events |= EV_RESET_GAP;
  } else if (fr == FEED_RESET_BOUNDS) {
    ptp.resets_bounds++;
    act->events |= EV_RESET_BOUNDS;
  } else if (fr == FEED_OK) {
    s->resid_ms += (resid * resid - s->resid_ms) / 64;
    if (s->samples >= MIN_SAMPLES_FOR_LOCK) {
      s->locked = true;
    }
  }
}

// ---- Packet handlers (caller holds ptp_state_mux) --------------------------

static void handle_sync(const uint8_t *data, size_t len, const ptp_hdr_t *h,
                        uint32_t src_ip, int64_t rx_ns, ptp_actions_t *act) {
  ptp_port_t *p = port_lookup(src_ip, h, true);
  if (p == NULL) {
    return;
  }
  const uint32_t now = now_ms();
  p->last_sync_ms = now;
  p->syncs++;
  p->transport = h->transport;
  p->domain = h->domain;
  p->log_interval = h->log_interval;
  p->pending = true;
  p->pending_seq = h->seq;
  p->pending_t2 = rx_ns;
  ptp.status_syncs++;

  select_port(act);
  if (ptp.selected < 0 || p != &ptp.ports[ptp.selected]) {
    if (ptp.expected_clock_id != 0 || ptp.peer_count != 0) {
      ptp.rejected_master_count++;
    }
    return;
  }
  ptp.sync_count++;
  ptp.last_sync_rx_ms = now;

  // A new SYNC supersedes the exchange of the previous one.
  if (ptp.x_active && ptp.x_dreq_sent && !ptp.x_have_t4) {
    ptp.unanswered++;
  }
  ptp.x_active = true;
  ptp.x_sync_seq = h->seq;
  ptp.x_sync_ms = now;
  ptp.x_t2 = rx_ns;
  ptp.x_have_t1 = false;
  ptp.x_dreq_sent = false;
  ptp.x_have_t3 = false;
  ptp.x_t3_pre = 0;
  ptp.x_have_t4 = false;

  // Apple's stack has no passive mode: a missing Delay_Resp is a missing
  // sample, and the next sample after a gap of 80 SYNCs resets its filter.
  // Here a master that has never answered on this port is followed
  // passively after 16 unanswered Delay_Reqs.  Once one has answered, lost
  // or late answers (Wi-Fi congestion) only cost samples, as on Apple's
  // receiver, and the two-way line extrapolates on its rate meanwhile; any
  // answer, even one too late to use, shows the master is still there.
  // After 80 SYNCs without a single answer the two-way line is stale, as a
  // gap that long is to Apple.  Apple would extrapolate on it until the
  // next sample, blind to the master's clock moving; the passive estimator
  // can see that, so the clock drops the stale line and follows passively,
  // and reports itself unlocked if it has no passive line either.
  const bool answered = ptp.last_dresp_ms != 0;
  if (ptp.mode != PTP_MODE_PASSIVE &&
      ptp.unanswered >=
          (answered ? PTP_DREQ_LOST_LIMIT : PTP_DREQ_UNANSWERED_LIMIT)) {
    ptp.mode = PTP_MODE_PASSIVE;
    ptp.last_probe_ms = now;
    if (answered) {
      // Same timeline: the rate is kept for when answers come back.
      servo_unseed(&ptp.tw, true);
      ptp.tw.locked = false;
      act->events |= EV_ANSWERS_STOPPED;
    } else {
      act->events |= EV_PASSIVE;
    }
    publish_model(NULL);
  }
  bool send = ptp.mode != PTP_MODE_PASSIVE;
  if (!send && now - ptp.last_probe_ms >= PTP_TWO_WAY_RETRY_MS) {
    send = true;
    ptp.last_probe_ms = now;
  }
  if (send && ptp.have_identity) {
    ptp.dreq_seq++;
    ptp.x_dreq_seq = ptp.dreq_seq;
    ptp.x_dreq_sent = true;
    act->send_dreq = true;
    act->dreq_ip = src_ip;
    act->dreq_seq = ptp.dreq_seq;
    build_delay_req(act->dreq, h->transport, h->domain, ptp.dreq_seq);
  }

  // One-step: the origin time is in the SYNC itself.
  if (!(h->flags & 0x0200) && len >= PTP_HEADER_SIZE + PTP_TIMESTAMP_SIZE) {
    ptp.x_t1 =
        parse_ptp_timestamp_ns(data + PTP_TIMESTAMP_OFFSET) + h->correction_ns;
    ptp.x_have_t1 = true;
    p->pending = false;
    passive_sample(ptp.x_t1, rx_ns, h->seq, port_nominal(p), now, act);
    publish_model(NULL);
  }
}

// Parse the Follow_Up TLVs: the 802.1AS information TLV (gmTimeBaseIndicator)
// and Apple's (00-0D-93, subtype 4: the GM id).  Apple discards a Follow_Up
// without the 802.1AS TLV; we accept it, since non-Apple senders omit it.
static void parse_followup_tlvs(ptp_port_t *p, const uint8_t *data, size_t len,
                                ptp_actions_t *act) {
  size_t off = PTP_HEADER_SIZE + PTP_TIMESTAMP_SIZE;
  while (off + 4 <= len) {
    const uint16_t type = (uint16_t)read_be(data + off, 2);
    const size_t tlv_len = (size_t)read_be(data + off + 2, 2);
    if (off + 4 + tlv_len > len) {
      break;
    }
    const uint8_t *v = data + off + 4;
    if (type == 0x0003 && tlv_len >= 6) {
      const uint32_t org = (uint32_t)read_be(v, 3);
      const uint32_t sub = (uint32_t)read_be(v + 3, 3);
      if (org == 0x0080C2 && sub == 1 && tlv_len >= 28) {
        const uint16_t tbi = (uint16_t)read_be(v + 10, 2);
        if (p->has_time_base && tbi != p->time_base && ptp.selected >= 0 &&
            p == &ptp.ports[ptp.selected]) {
          // The GM's time base had a discontinuity: same GM, new line.
          act->events |= EV_TIME_BASE;
          soft_reset();
        }
        p->time_base = tbi;
        p->has_time_base = true;
      } else if (org == 0x000D93 && sub == 4 && tlv_len >= 14) {
        const uint64_t gm = read_be(v + 6, 8);
        if (p->has_announce && gm != p->gm) {
          act->events |= EV_APPLE_GM_MISMATCH;
        }
        p->apple_gm = gm;
        p->has_apple_gm = true;
      }
    }
    off += 4 + tlv_len;
  }
}

static void handle_followup(const uint8_t *data, size_t len, const ptp_hdr_t *h,
                            uint32_t src_ip, ptp_actions_t *act) {
  ptp_port_t *p = port_lookup(src_ip, h, false);
  // Sequence numbers are only unique within one source port: pairing one
  // clock's SYNC with another's FOLLOW_UP would fabricate an offset.
  if (p == NULL || !p->pending || p->pending_seq != h->seq ||
      len < PTP_HEADER_SIZE + PTP_TIMESTAMP_SIZE) {
    return;
  }
  p->pending = false;
  const size_t end =
      h->length >= PTP_HEADER_SIZE + PTP_TIMESTAMP_SIZE && h->length < len
          ? h->length
          : len;
  parse_followup_tlvs(p, data, end, act);

  const int64_t t1 =
      parse_ptp_timestamp_ns(data + PTP_TIMESTAMP_OFFSET) + h->correction_ns;
  const int64_t t2 = p->pending_t2;
  // Measured SYNC interval from successive origin times, starting from the
  // advertised one.  A measurement far from the estimate (a late send, a
  // clock step) is ignored unless 8 in a row agree it is wrong.
  if (p->has_last_t1) {
    const uint16_t k = (uint16_t)(h->seq - p->last_t1_seq);
    if (k >= 1 && k <= 4 && t1 > p->last_t1) {
      const int64_t m = clamp_interval((t1 - p->last_t1) / k);
      const int64_t est = port_nominal(p);
      if (abs64(m - est) <= est / 4) {
        p->interval_ns = est + (m - est) / 8;
        p->interval_misses = 0;
      } else if (++p->interval_misses >= 8) {
        p->interval_ns = m;
        p->interval_misses = 0;
      }
    }
  }
  p->last_t1 = t1;
  p->last_t1_seq = h->seq;
  p->has_last_t1 = true;

  if (ptp.selected < 0 || p != &ptp.ports[ptp.selected]) {
    return;
  }
  ptp.followup_count++;
  if (ptp.mode == PTP_MODE_PASSIVE || !servo_usable(&ptp.tw)) {
    ptp.raw_offset_ns = t1 - t2;
  }
  passive_sample(t1, t2, h->seq, port_nominal(p), p->last_sync_ms, act);
  if (ptp.x_active && ptp.x_sync_seq == h->seq) {
    ptp.x_t1 = t1;
    ptp.x_have_t1 = true;
    if (ptp.x_have_t3 && ptp.x_have_t4) {
      two_way_sample(p, act);
    }
  }
  publish_model(act);
}

// Caller holds ptp_state_mux.  The two-way filter has a new t3 source: the
// midpoints move by half the difference, and the delays it gates on change,
// so it starts over on this timeline (lock and rate kept).
static void t3_source_changed(bool tx_done) {
  ptp.t3_tx_done = tx_done;
  ptp.tx_done_misses = 0;
  servo_unseed(&ptp.tw, true);
  reset_delay_stats();
  ptp.delay_total = 0;
  ptp.lucky_limit_ns = INT64_MAX;
}

// Caller holds ptp_state_mux.  t3 for the exchange whose Delay_Resp just
// arrived, from one source as described at PTP_TX_DONE_MISS_LIMIT: the
// driver's TX-done stamp when it reported this Delay_Req, after it was
// handed to the stack and not in the future; else the software stamp taken
// at sendto(), unless the filter is on TX-done stamps.
static void select_t3(void) {
  int64_t tx_ns;
  if (tx_done_stamp_ns(ptp.x_dreq_seq, &tx_ns) && ptp.x_t3_pre != 0 &&
      tx_ns >= ptp.x_t3_pre && tx_ns <= get_local_time_ns()) {
    if (!ptp.t3_tx_done) {
      t3_source_changed(true);
    }
    ptp.tx_done_misses = 0;
    ptp.x_t3 = tx_ns;
    ptp.x_have_t3 = true;
    ptp.tx_done_used++;
    return;
  }
  const bool stopped = ++ptp.tx_done_misses >= PTP_TX_DONE_MISS_LIMIT;
  if (stopped) {
    ptp.tx_done_misses = 0;
    ptp.tx_done_retry = true; // register again (at most every 10 s)
  }
  if (!ptp.t3_tx_done) {
    return; // software t3, if its bracket was tight
  }
  ptp.x_have_t3 = false;
  ptp.tx_done_lost++;
  if (stopped) {
    t3_source_changed(false);
  }
}

static void handle_delay_resp(const uint8_t *data, size_t len,
                              const ptp_hdr_t *h, uint32_t src_ip,
                              ptp_actions_t *act) {
  if (len < PTP_DELAY_RESP_LEN) {
    return;
  }
  const uint8_t *req = data + PTP_TIMESTAMP_OFFSET + PTP_TIMESTAMP_SIZE;
  if (memcmp(req, ptp.local_identity, 8) != 0 ||
      read_be(req + 8, 2) != PTP_LOCAL_PORT_NUMBER) {
    return; // someone else's
  }
  if (ptp.selected < 0 || ptp.ports[ptp.selected].ip != src_ip) {
    return;
  }
  ptp_port_t *p = &ptp.ports[ptp.selected];
  // Even a late answer shows the master answers: congestion that holds
  // responses past the next SYNC is no reason to go passive.
  ptp.dresp_received++;
  ptp.unanswered = 0;
  ptp.last_dresp_ms = now_ms();
  if (ptp.mode != PTP_MODE_TWO_WAY) {
    if (ptp.mode == PTP_MODE_PASSIVE) {
      act->events |= EV_TWO_WAY;
    }
    ptp.mode = PTP_MODE_TWO_WAY;
  }
  if (!ptp.x_active || !ptp.x_dreq_sent || ptp.x_have_t4 ||
      h->seq != ptp.x_dreq_seq) {
    ptp.superseded++;
    return;
  }
  // The Delay_Resp's correctionField is the Delay_Req's residence time in
  // transparent clocks on the way (the master copies it over), so it is
  // subtracted: t4 is when the Delay_Req would have arrived without them
  // (IEEE 1588-2019 11.3.2).  Apple's kernel adds it instead; AirPlay paths
  // have no transparent clocks and the field is 0, so this only matters on
  // a network that does have one, where adding would double the error.
  ptp.x_t4 =
      parse_ptp_timestamp_ns(data + PTP_TIMESTAMP_OFFSET) - h->correction_ns;
  ptp.x_have_t4 = true;
  select_t3();
  if (ptp.x_have_t1 && ptp.x_have_t3) {
    two_way_sample(p, act);
    publish_model(act);
  }
}

static void handle_announce(const uint8_t *data, size_t len, const ptp_hdr_t *h,
                            uint32_t src_ip) {
  ptp.announce_count++;
  if (len < PTP_ANNOUNCE_LEN) {
    return;
  }
  ptp_port_t *p = port_lookup(src_ip, h, true);
  if (p == NULL) {
    return;
  }
  p->ann_p1 = data[47];
  p->ann_class = data[48];
  p->ann_accuracy = data[49];
  p->ann_variance = (uint16_t)read_be(data + 50, 2);
  p->ann_p2 = data[52];
  p->gm = read_be(data + 53, 8);
  p->steps_removed = (uint16_t)read_be(data + 61, 2);
  p->has_announce = true;
  p->last_announce_ms = now_ms();
}

static void log_actions(const ptp_actions_t *act) {
  const uint32_t ev = act->events;
  if (ev == 0) {
    return;
  }
  if (ev & EV_GM_CHANGE) {
    ESP_LOGI(TAG, "grandmaster changed %016llx -> %016llx, clock reset",
             (unsigned long long)act->gm_old, (unsigned long long)act->gm_new);
  }
  if (ev & EV_PORT_CHANGE) {
    ESP_LOGI(TAG, "now measuring through " PTP_IP_FMT,
             PTP_IP_ARGS(act->port_ip));
  }
  if (ev & EV_PASSIVE) {
    ESP_LOGW(TAG,
             "master does not answer Delay_Req (%d in a row): passive mode, "
             "retrying every %d s",
             PTP_DREQ_UNANSWERED_LIMIT, PTP_TWO_WAY_RETRY_MS / 1000);
  }
  if (ev & EV_ANSWERS_STOPPED) {
    ESP_LOGW(TAG,
             "no Delay_Resp for %d SYNCs: passive mode, retrying every %d s",
             PTP_DREQ_LOST_LIMIT, PTP_TWO_WAY_RETRY_MS / 1000);
  }
  if (ev & EV_TWO_WAY) {
    ESP_LOGI(TAG, "master answers Delay_Req again: two-way mode");
  }
  if (ev & EV_RESET_DROPS) {
    ESP_LOGI(TAG,
             "%d samples in a row off by up to %lld us: master's clock moved, "
             "filter reset",
             PTP_PPM_DROP_LIMIT, (long long)(act->step_ns / 1000));
  }
  if (ev & EV_PASSIVE_STEP) {
    ESP_LOGI(TAG, "offset fell by %lld ms and stayed there, following it",
             (long long)(-act->step_ns / 1000000));
  }
  if (ev & EV_RESET_GAP) {
    ESP_LOGI(TAG, "SYNC gap of %d or more: filter reset", PTP_GAP_RESET_SYNCS);
  }
  if (ev & EV_RESET_BOUNDS) {
    ESP_LOGI(TAG, "interval or rate out of bounds: filter reset");
  }
  if (ev & EV_TIME_BASE) {
    ESP_LOGI(TAG, "gmTimeBaseIndicator changed: filter reset");
  }
  if (ev & EV_APPLE_GM_MISMATCH) {
    ESP_LOGD(TAG, "Apple Follow_Up TLV names another GM than Announce");
  }
  if (ev & EV_LOCKED) {
    ESP_LOGI(TAG, "LOCKED: gm=%016llx mode=%s after %lu SYNCs",
             (unsigned long long)act->lock_gm,
             act->lock_two_way ? "two-way" : "passive",
             (unsigned long)act->lock_syncs);
  }
}

static int cmp_i32(const void *a, const void *b) {
  const int32_t x = *(const int32_t *)a, y = *(const int32_t *)b;
  return (x > y) - (x < y);
}

// Copy the delay window under the mux, sort outside it.  Returns how many
// samples it holds.
static size_t sorted_delays(int32_t *out) {
  portENTER_CRITICAL(&ptp_state_mux);
  size_t n =
      ptp.delay_total < PTP_DELAY_WINDOW ? ptp.delay_total : PTP_DELAY_WINDOW;
  memcpy(out, ptp.delays, n * sizeof(*out));
  portEXIT_CRITICAL(&ptp_state_mux);
  qsort(out, n, sizeof(*out), cmp_i32);
  return n;
}

// Adaptive delay limit, max(median of the last 80, 10 ms), and the lucky
// threshold.
static void update_delay_thresholds(bool median, bool lucky) {
  static int32_t scratch[PTP_DELAY_WINDOW];
  const size_t n = sorted_delays(scratch);
  if (n < PTP_LUCKY_EVERY) {
    return;
  }
  int64_t limit = scratch[n / 2];
  if (limit < PTP_DELAY_LIMIT_NS) {
    limit = PTP_DELAY_LIMIT_NS;
  }
  const int64_t luck =
      PTP_LUCKY_PERCENTILE >= 100
          ? INT64_MAX
          : scratch[n * PTP_LUCKY_PERCENTILE / 100] + PTP_LUCKY_SLACK_NS;
  portENTER_CRITICAL(&ptp_state_mux);
  if (median) {
    ptp.delay_limit_ns = limit;
  }
  if (lucky) {
    ptp.lucky_limit_ns = luck;
  }
  portEXIT_CRITICAL(&ptp_state_mux);
}

// Process one received PTP message.  rx_ns is its receive time (only used
// for SYNC).
static void handle_packet(const uint8_t *data, size_t len, bool is_event_port,
                          uint32_t src_ip, int64_t rx_ns) {
  ptp_hdr_t h;
  if (!parse_header(data, len, &h)) {
    return;
  }
  ptp_actions_t act = {0};

  portENTER_CRITICAL(&ptp_state_mux);
  switch (h.type) {
  case PTP_MSG_SYNC:
    if (is_event_port) {
      handle_sync(data, len, &h, src_ip, rx_ns, &act);
    }
    break;
  case PTP_MSG_FOLLOW_UP:
    if (!is_event_port) {
      handle_followup(data, len, &h, src_ip, &act);
    }
    break;
  case PTP_MSG_DELAY_RESP:
    if (!is_event_port) {
      handle_delay_resp(data, len, &h, src_ip, &act);
    }
    break;
  case PTP_MSG_ANNOUNCE:
    handle_announce(data, len, &h, src_ip);
    break;
  default:
    break;
  }
  portEXIT_CRITICAL(&ptp_state_mux);

  if (act.send_dreq) {
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(PTP_EVENT_PORT);
    dst.sin_addr.s_addr = act.dreq_ip;
    ensure_tx_done_cb(now_ms());
    const int64_t t3_pre = get_local_time_ns();
    const ssize_t sent = sendto(ptp.event_socket, act.dreq, PTP_DELAY_REQ_LEN,
                                0, (struct sockaddr *)&dst, sizeof(dst));
    // t3: right after the packet has been handed to the stack, the closest
    // software point to its departure -- unless the task was preempted
    // around the send, which the bracket shows.
    const int64_t t3 = get_local_time_ns();
    const bool t3_ok = t3 - t3_pre <= PTP_TX_STAMP_MAX_NS;
    // Only this task handles packets, so the Delay_Resp cannot have been
    // processed yet: the exchange completes in handle_delay_resp().
    portENTER_CRITICAL(&ptp_state_mux);
    const bool mine =
        ptp.x_active && ptp.x_dreq_sent && ptp.x_dreq_seq == act.dreq_seq;
    if (sent == PTP_DELAY_REQ_LEN) {
      ptp.dreq_sent++;
      if (!t3_ok) {
        ptp.tx_slow++;
      }
      if (mine) {
        ptp.x_t3_pre = t3_pre;
        if (t3_ok) {
          ptp.x_t3 = t3;
          ptp.x_have_t3 = true;
        }
      }
    } else {
      ptp.dreq_tx_errors++;
      if (mine) {
        ptp.x_dreq_sent = false; // never left: not "unanswered"
      }
    }
    portEXIT_CRITICAL(&ptp_state_mux);
  }
  if (act.need_median || act.need_lucky) {
    update_delay_thresholds(act.need_median, act.need_lucky);
  }
  log_actions(&act);
}

// ---- Status line -----------------------------------------------------------

static void log_status(void) {
  static int32_t scratch[PTP_DELAY_WINDOW];
  portENTER_CRITICAL(&ptp_state_mux);
  const uint32_t syncs = ptp.status_syncs;
  ptp.status_syncs = 0;
  const bool two_way = output_servo() == &ptp.tw;
  const bool locked = check_locked();
  const ptp_servo_t *s = output_servo();
  const int64_t resid_ms = s ? s->resid_ms : 0;
  const int64_t dev_q = s ? s->dev_q : 0;
  const uint64_t gm = ptp.gm_id;
  const uint32_t ip = ptp.selected >= 0 ? ptp.ports[ptp.selected].ip : 0;
  const uint32_t sent = ptp.dreq_sent, recv = ptp.dresp_received;
  const uint32_t ppm = ptp.ppm_drops, dly = ptp.delay_drops;
  const uint32_t pas = ptp.passive_drops, lucky = ptp.lucky_skips;
  const uint32_t r_drop = ptp.resets_drops, r_oob = ptp.resets_bounds;
  const uint32_t r_gap = ptp.resets_gap, r_gm = ptp.gm_changes;
  const uint32_t hook = ptp.rx_hook_stamps, task = ptp.rx_task_stamps;
  const uint32_t rejected = ptp.rejected_master_count;
  const uint32_t tx_slow = ptp.tx_slow, tx_done = ptp.tx_done_used;
  const uint32_t tx_lost = ptp.tx_done_lost;
  const int64_t bias = ptp.passive_bias_n ? ptp.passive_bias_ns : 0;
  portEXIT_CRITICAL(&ptp_state_mux);
  if (syncs == 0) {
    return; // nothing to report while no master is sending
  }
  const size_t n = sorted_delays(scratch);
  // RTT = 2 x the rate-corrected one-way delay
  const long rtt_min = n ? (long)scratch[0] * 2 / 1000 : -1;
  const long rtt_med = n ? (long)scratch[n / 2] * 2 / 1000 : -1;
  // dev_q is bounded by the 10000 ppm gate, so this fits in 32 bits.
  const long ppb = (long)mul_q40(1000000000LL, dev_q);
  const long ppb_abs = ppb < 0 ? -ppb : ppb;
  // Two lines: one long line is cut off by the log buffer.
  ESP_LOGI(TAG,
           "status: %s %s gm=%016llx via " PTP_IP_FMT
           " dreq=%lu resp=%lu rtt_us min=%ld med=%ld resid_rms_us=%lu "
           "rate=%c%ld.%03ldppm",
           locked ? "locked" : "unlocked", two_way ? "two-way" : "passive",
           (unsigned long long)gm, PTP_IP_ARGS(ip), (unsigned long)sent,
           (unsigned long)recv, rtt_min, rtt_med,
           (unsigned long)(isqrt64((uint64_t)resid_ms) / 1000),
           ppb < 0 ? '-' : '+', ppb_abs / 1000, ppb_abs % 1000);
  ESP_LOGI(
      TAG,
      "status: drops gate=%lu delay=%lu passive=%lu lucky=%lu "
      "t3 txdone=%lu lost=%lu slow=%lu resets drop=%lu oob=%lu gap=%lu gm=%lu "
      "bias_us=%ld "
      "rx hook=%lu task=%lu rejected=%lu",
      (unsigned long)ppm, (unsigned long)dly, (unsigned long)pas,
      (unsigned long)lucky, (unsigned long)tx_done, (unsigned long)tx_lost,
      (unsigned long)tx_slow, (unsigned long)r_drop, (unsigned long)r_oob,
      (unsigned long)r_gap, (unsigned long)r_gm, (long)(bias / 1000),
      (unsigned long)hook, (unsigned long)task, (unsigned long)rejected);
}

// ---- Sockets and task
// --------------------------------------------------------

// Create and bind multicast socket
static int create_ptp_socket(uint16_t port) {
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %d", errno);
    return -1;
  }

  // Allow address reuse
  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  // Bind to port
  struct sockaddr_in addr = {0};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind to port %d: %d", port, errno);
    close(sock);
    return -1;
  }

  // Join multicast group
  struct ip_mreq mreq = {0};
  mreq.imr_multiaddr.s_addr = inet_addr(PTP_MULTICAST_ADDR);
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);

  if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) <
      0) {
    ESP_LOGE(TAG, "Failed to join multicast group: %d", errno);
    close(sock);
    return -1;
  }

  // Set receive timeout
  struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  return sock;
}

static void receive_one(int sock, bool is_event_port, uint8_t *buffer,
                        size_t size) {
  struct sockaddr_in src = {0};
  socklen_t src_len = sizeof(src);
  const ssize_t len =
      recvfrom(sock, buffer, size, 0, (struct sockaddr *)&src, &src_len);
  const int64_t task_now = get_local_time_ns();
  if (len <= 0) {
    return;
  }
  const uint32_t ip = src.sin_addr.s_addr;
  int64_t rx_ns = task_now;
  if (is_event_port) {
    bool hook = false;
    rx_ns = rx_timestamp_ns(buffer, (size_t)len, ip, task_now, &hook);
    if ((buffer[0] & 0x0F) == PTP_MSG_SYNC) {
      portENTER_CRITICAL(&ptp_state_mux);
      if (hook) {
        ptp.rx_hook_stamps++;
      } else {
        ptp.rx_task_stamps++;
      }
      portEXIT_CRITICAL(&ptp_state_mux);
    }
  }
  handle_packet(buffer, (size_t)len, is_event_port, ip, rx_ns);
}

// PTP task - listens for messages on both ports
static void ptp_task(void *pvParameters) {
  uint8_t buffer[256];

  while (ptp.running) {
    fd_set read_fds;
    FD_ZERO(&read_fds);

    int max_fd = -1;
    if (ptp.event_socket >= 0) {
      FD_SET(ptp.event_socket, &read_fds);
      if (ptp.event_socket > max_fd) {
        max_fd = ptp.event_socket;
      }
    }
    if (ptp.general_socket >= 0) {
      FD_SET(ptp.general_socket, &read_fds);
      if (ptp.general_socket > max_fd) {
        max_fd = ptp.general_socket;
      }
    }

    if (max_fd < 0) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
    int ret = select(max_fd + 1, &read_fds, NULL, NULL, &tv);

    if (ret < 0) {
      if (!ptp.running) {
        break; // Sockets closed during shutdown
      }
      if (errno != EINTR) {
        ESP_LOGE(TAG, "select error: %d", errno);
      }
      continue;
    }

    if (ret > 0) {
      // Event port first: a SYNC's Delay_Req should leave before anything
      // else is processed.
      if (ptp.event_socket >= 0 && FD_ISSET(ptp.event_socket, &read_fds)) {
        receive_one(ptp.event_socket, true, buffer, sizeof(buffer));
      }
      if (ptp.general_socket >= 0 && FD_ISSET(ptp.general_socket, &read_fds)) {
        receive_one(ptp.general_socket, false, buffer, sizeof(buffer));
      }
    }

    const uint32_t now = now_ms();
    if (now - ptp.last_status_ms >= PTP_STATUS_INTERVAL_MS) {
      ptp.last_status_ms = now;
      log_status();
    }
  }

  // Cleanup
  if (ptp.event_socket >= 0) {
    close(ptp.event_socket);
    ptp.event_socket = -1;
  }
  if (ptp.general_socket >= 0) {
    close(ptp.general_socket);
    ptp.general_socket = -1;
  }

  ptp.task_handle = NULL;
  vTaskDelete(NULL);
}

// clockIdentity: EUI-64 from the Wi-Fi station MAC (IEEE 1588 7.5.2.2.2),
// stable across boots.
static void init_identity(void) {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  ptp.local_identity[0] = mac[0];
  ptp.local_identity[1] = mac[1];
  ptp.local_identity[2] = mac[2];
  ptp.local_identity[3] = 0xFF;
  ptp.local_identity[4] = 0xFE;
  ptp.local_identity[5] = mac[3];
  ptp.local_identity[6] = mac[4];
  ptp.local_identity[7] = mac[5];
  ptp.have_identity = true;
}

// Caller holds ptp_state_mux.
static void clear_clock_state(void) {
  reset_timeline();
  ptp.gm_id = 0;
  ptp.model_l = 0;
  ptp.model_r = 0;
  ptp.model_dev_q = 0;
  ptp.model_inv_dev_q = 0;
  ptp.raw_offset_ns = 0;
  ptp.selected = -1;
  ptp.mode = PTP_MODE_PROBING;
  ptp.unanswered = 0;
  ptp.last_probe_ms = 0;
  for (int i = 0; i < PTP_MAX_PORTS; i++) {
    ptp.ports[i].pending = false;
  }
  ptp.sync_count = 0;
  ptp.followup_count = 0;
  // Drop the master filter so the next session can lock to whatever master
  // its anchor packet names (which may differ from the previous session).
  ptp.expected_clock_id = 0;
  ptp.session_ended = false;
  ptp.peer_count = 0;
}

static void reset_all_state(void) {
  memset(&ptp, 0, sizeof(ptp));
  ptp.event_socket = -1;
  ptp.general_socket = -1;
  ptp.selected = -1;
  ptp.delay_limit_ns = PTP_DELAY_LIMIT_NS;
  ptp.lucky_limit_ns = INT64_MAX;
  reset_passive_window();
}

esp_err_t ptp_clock_init(void) {
  if (ptp.running) {
    return ESP_ERR_INVALID_STATE;
  }

  portENTER_CRITICAL(&ptp_state_mux);
  reset_all_state();
  portEXIT_CRITICAL(&ptp_state_mux);
  init_identity();

  // Create sockets
  ptp.event_socket = create_ptp_socket(PTP_EVENT_PORT);
  if (ptp.event_socket < 0) {
    return ESP_FAIL;
  }

  ptp.general_socket = create_ptp_socket(PTP_GENERAL_PORT);
  if (ptp.general_socket < 0) {
    close(ptp.event_socket);
    ptp.event_socket = -1;
    return ESP_FAIL;
  }

  // Start task.  Above the decoder, so a SYNC is answered and t3 taken
  // without waiting out a decode; everything it does per packet is short.
  ptp.running = true;
  BaseType_t ret =
      task_create_spiram(ptp_task, "ptp_clock", PTP_TASK_STACK, NULL,
                         PTP_TASK_PRIORITY, &ptp.task_handle, &ptp.task_mem);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create PTP task");
    close(ptp.event_socket);
    close(ptp.general_socket);
    ptp.event_socket = -1;
    ptp.general_socket = -1;
    ptp.running = false;
    return ESP_FAIL;
  }

  return ESP_OK;
}

void ptp_clock_stop(void) {
  if (!ptp.running) {
    return;
  }

  ptp.running = false;

  // Close sockets to unblock select
  if (ptp.event_socket >= 0) {
    close(ptp.event_socket);
    ptp.event_socket = -1;
  }
  if (ptp.general_socket >= 0) {
    close(ptp.general_socket);
    ptp.general_socket = -1;
  }

  // Wait for task to exit (task sets task_handle = NULL before vTaskDelete)
  for (int i = 0; i < 20 && ptp.task_handle != NULL; i++) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  if (ptp.task_handle != NULL) {
    ESP_LOGW(TAG, "PTP task did not exit in time");
  }
  task_free_spiram(&ptp.task_mem);
  tx_done_unregister();
}

void ptp_clock_clear(void) {
  portENTER_CRITICAL(&ptp_state_mux);
  clear_clock_state();
  portEXIT_CRITICAL(&ptp_state_mux);
}

// Caller holds ptp_state_mux.
// True when the samples still describe the master: it sent one recently and
// that one was accepted.  An iPhone's PTP clock stops while it sleeps, so after
// a pause the first samples can land seconds away from the model -- a lock
// carried across that would place the next anchor that far out.
static bool samples_current(void) {
  const ptp_servo_t *s = output_servo();
  if (s == NULL || !s->seeded || s->drops != 0 || ptp.run_syncs != 0) {
    return false;
  }
  return now_ms() - s->last_accept_ms < SESSION_KEEP_MS;
}

void ptp_clock_end_session(void) {
  portENTER_CRITICAL(&ptp_state_mux);
  if (!samples_current()) {
    clear_clock_state();
    portEXIT_CRITICAL(&ptp_state_mux);
    return;
  }
  // The master is still sending, so the samples are current. Keep its filter:
  // with it dropped, any other master on the network (an Apple TV, a HomePod)
  // could be chosen until the next session names one. The next session may
  // still name a different master; naming this one again keeps the lock (see
  // ptp_clock_set_master_clock_id()).
  ptp.session_ended = true;
  portEXIT_CRITICAL(&ptp_state_mux);
}

void ptp_clock_notify_resume(uint32_t pause_duration_ms) {
  if (pause_duration_ms < PTP_LONG_PAUSE_THRESHOLD_MS) {
    return; // the model kept running
  }
  // The clock measured through the pause unless the master went quiet or
  // moved; then the next sample seeds the filter instead of being gated
  // against the old line (the lock and last model stay until it comes).
  portENTER_CRITICAL(&ptp_state_mux);
  const bool current = samples_current();
  if (!current) {
    soft_reset();
  }
  portEXIT_CRITICAL(&ptp_state_mux);
  ESP_LOGI(TAG, "notify_resume: pause=%lu ms, %s",
           (unsigned long)pause_duration_ms,
           current ? "clock current" : "re-seeding on the next sample");
}

bool ptp_clock_is_locked(void) {
  portENTER_CRITICAL(&ptp_state_mux);
  const bool locked = check_locked();
  portEXIT_CRITICAL(&ptp_state_mux);
  return locked;
}

uint64_t ptp_clock_get_time_ns(void) {
  const int64_t local_ns = get_local_time_ns();
  portENTER_CRITICAL(&ptp_state_mux);
  const int64_t t = ptp.model_valid ? model_predict(local_ns) : local_ns;
  portEXIT_CRITICAL(&ptp_state_mux);
  return (uint64_t)t;
}

int64_t ptp_clock_get_offset_ns(void) {
  const int64_t local_ns = get_local_time_ns();
  portENTER_CRITICAL(&ptp_state_mux);
  const int64_t offset =
      ptp.model_valid ? model_predict(local_ns) - local_ns : 0;
  portEXIT_CRITICAL(&ptp_state_mux);
  return offset;
}

bool ptp_clock_local_to_network(int64_t local_ns, uint64_t *network_ns,
                                uint64_t *gm_id) {
  portENTER_CRITICAL(&ptp_state_mux);
  const bool locked = check_locked();
  const int64_t t = model_predict(local_ns);
  const uint64_t gm = ptp.gm_id;
  portEXIT_CRITICAL(&ptp_state_mux);
  if (!locked) {
    return false;
  }
  if (network_ns) {
    *network_ns = (uint64_t)t;
  }
  if (gm_id) {
    *gm_id = gm;
  }
  return true;
}

bool ptp_clock_network_to_local(uint64_t network_ns, uint64_t gm_id,
                                int64_t *local_ns) {
  portENTER_CRITICAL(&ptp_state_mux);
  const bool ok = check_locked() && gm_id == ptp.gm_id;
  const int64_t dr = (int64_t)network_ns - ptp.model_r;
  const int64_t l = ptp.model_l + dr + mul_q40(dr, ptp.model_inv_dev_q);
  portEXIT_CRITICAL(&ptp_state_mux);
  if (ok && local_ns) {
    *local_ns = l;
  }
  return ok;
}

uint64_t ptp_clock_get_grandmaster_id(void) {
  portENTER_CRITICAL(&ptp_state_mux);
  const uint64_t gm = ptp.gm_id;
  portEXIT_CRITICAL(&ptp_state_mux);
  return gm;
}

void ptp_clock_set_peers(const uint32_t *ipv4_addrs_net_order, size_t count) {
  if (count > PTP_MAX_PEERS) {
    count = PTP_MAX_PEERS;
  }
  portENTER_CRITICAL(&ptp_state_mux);
  ptp.peer_count = ipv4_addrs_net_order ? count : 0;
  for (size_t i = 0; i < ptp.peer_count; i++) {
    ptp.peers[i] = ipv4_addrs_net_order[i];
  }
  portEXIT_CRITICAL(&ptp_state_mux);
  ESP_LOGI(TAG, "%u timing peer(s)", (unsigned)count);
}

void ptp_clock_set_master_clock_id(uint64_t clock_id) {
  portENTER_CRITICAL(&ptp_state_mux);
  if (clock_id == ptp.expected_clock_id && !ptp.session_ended) {
    portEXIT_CRITICAL(&ptp_state_mux);
    return;
  }
  ptp.session_ended = false;

  // A session naming the GM the clock is on, with current samples -- the
  // next track from the same phone -- keeps the lock. Re-acquiring it takes
  // ~0.5 s, and a start waiting on it joins the track that far in.
  // Not locked yet on that GM: nothing stale to drop, keep measuring.
  const bool keep = clock_id != 0 && ptp.gm_id == clock_id &&
                    (samples_current() || !ptp.locked);
  ptp.expected_clock_id = clock_id;
  ptp.expected_gm_seen_ms = now_ms();
  ptp.expected_released = false;
  if (!keep) {
    // The model may describe another timeline.
    reset_timeline();
    ptp.gm_id = 0;
    ptp.selected = -1;
    ptp.mode = PTP_MODE_PROBING;
    ptp.unanswered = 0;
    for (int i = 0; i < PTP_MAX_PORTS; i++) {
      ptp.ports[i].pending = false;
    }
  }
  portEXIT_CRITICAL(&ptp_state_mux);

  ESP_LOGI(TAG, "PTP master clock_id %s: %016llx%s",
           clock_id ? "set" : "cleared", (unsigned long long)clock_id,
           keep ? " (already measuring it, lock kept)" : "");
}

uint64_t ptp_clock_get_master_clock_id(void) {
  portENTER_CRITICAL(&ptp_state_mux);
  uint64_t clock_id = ptp.expected_clock_id;
  portEXIT_CRITICAL(&ptp_state_mux);
  return clock_id;
}

void ptp_clock_get_stats(ptp_stats_t *stats) {
  const int64_t local_ns = get_local_time_ns();
  portENTER_CRITICAL(&ptp_state_mux);
  stats->sync_count = ptp.sync_count;
  stats->followup_count = ptp.followup_count;
  stats->last_offset_ns = ptp.raw_offset_ns;
  stats->filtered_offset_ns =
      ptp.model_valid ? model_predict(local_ns) - local_ns : 0;
  stats->outlier_count = ptp.ppm_drops + ptp.delay_drops + ptp.passive_drops;
  stats->delay_req_sent = ptp.dreq_sent;
  stats->delay_resp_received = ptp.dresp_received;
  const ptp_servo_t *s = output_servo();
  stats->two_way = s == &ptp.tw;
  stats->rate_ppb = s ? (int32_t)mul_q40(1000000000LL, s->dev_q) : 0;
  stats->grandmaster_id = ptp.gm_id;
  if (check_locked() && ptp.lock_start_ms > 0) {
    stats->lock_time_ms = now_ms() - ptp.lock_start_ms;
  } else {
    stats->lock_time_ms = 0;
  }
  portEXIT_CRITICAL(&ptp_state_mux);
}

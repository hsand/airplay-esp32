// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host simulation of the AirPlay 2 buffered playout chain, driving the real
 * engine, scheduler, timeline and clock map (the current ones, or the v0.3.0
 * copies in baseline/ when built with -DBASELINE):
 *
 *   sender (anchor, AAC-sized packets, send-ahead) -> TCP with jitter and a
 *   window -> reader -> decode queue -> "decode" -> timeline -> scheduler ->
 *   a DMA ring drained by a DAC on the local crystal -> the ear.
 *
 * Past the ring the sound still has the I2S FIFO, the DAC's filter and
 * anything downstream to get through.  Unless a scenario models that path
 * (.phys), it takes exactly as long as each receiver assumes -- 5 ms for
 * v0.3.0, the latency breakdown (audio_latency.c) for this tree -- so the
 * other scenarios measure everything but the latency model.
 *
 * Every PCM frame carries its own RTP position and track in its two channels
 * (two offset ramps), so what reaches the DAC can be decoded back and its
 * true emission time compared with when the sender meant it to play.  The
 * clock is a mock: for the new code it implements the PTP contract API
 * (offset + rate, noise, glitches, grandmaster changes); for the baseline it
 * is the offset-only API the old receiver read, with the old receiver's
 * behaviour around it.
 *
 * Usage: sim <scenario> [-v]   -- prints one result line. */

#include "sim_platform.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "audio_engine_v2.h"

#ifndef BASELINE
#include "audio_flush_queue.h"
#include "audio_playout_clock.h"
#include "ntp_clock.h"
#include "ptp_clock.h"

#include "audio_latency.h"
#endif

#define FS           44100
#define FRAME        1024U
#define BLOCK        353U
#define RING_FRAMES  2048U
#define EXTRA_US     5000 /* v0.3.0's fixed post-ring latency */
/* The DAC this tree's receiver is told about: the PCM5102A preset (22
 * samples, Q4) behind a 32-frame I2S FIFO. */
#define RX_DAC_Q4      (22U * 16U)
#define RX_FIFO_FRAMES 32U
#define CHANNELS     2
#define BLOCKS       192
#define RTP_ORIGIN   1000000U
#define DQ_CAPACITY  64U   /* decode queue, packets */
#define TCP_WINDOW   24U   /* packets the sender may have unread */
#define SEND_AHEAD_S 6.0   /* how far ahead of playout the sender streams */
#define LINK_PPS     110.0 /* packets per second the link carries */
#define MAX_PACKETS  8192U

int64_t sim_now_us;
static int verbose;

void sim_log(char level, const char *tag, const char *fmt, ...) {
  if (!verbose) {
    return;
  }
  va_list ap;
  va_start(ap, fmt);
  printf("%c %8.3f %s: ", level, sim_now_us / 1e6, tag);
  vprintf(fmt, ap);
  printf("\n");
  va_end(ap);
}

/* ---------- random ---------- */

static uint64_t rng_state = 88172645463325252ULL;
static double urand(void) {
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 7;
  rng_state ^= rng_state << 17;
  return (double)(rng_state >> 11) / 9007199254740992.0;
}
static double gauss(void) {
  double u = urand(), v = urand();
  if (u < 1e-12) {
    u = 1e-12;
  }
  return sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
}

/* ---------- scenario ---------- */

typedef struct {
  const char *name;
  double duration_s;
  double local_ppm; /* local crystal (and DAC) against true time */
  double stall_at_s, stall_ms;
  double glitch_at_s, glitch_len_s, glitch_ms;
  double gm_change_at_s, gm_relock_s;
  double pause_at_s, resume_at_s;
  double skip_at_s;
  double lose_at_s;
  double reanchor_every_s, reanchor_shift_ms;
  double clock_noise_us;
  /* The real path after the ring, when the scenario models it: the FIFO,
   * the DAC's group delay and whatever is downstream (an AV receiver). */
  bool phys;
  double phys_fifo_frames, phys_dac_samples, phys_downstream_us;
  /* This tree's output latency trim: from the start, then changed at
   * trim_at_s to trim_to_us. */
  double trim_us, trim_at_s, trim_to_us;
} scenario_t;

static const scenario_t scenarios[] = {
    {.name = "steady", .duration_s = 40},
    {.name = "drift+50", .duration_s = 60, .local_ppm = 50},
    {.name = "drift-50", .duration_s = 60, .local_ppm = -50},
    {.name = "stall40", .duration_s = 40, .stall_at_s = 20, .stall_ms = 40},
    {.name = "stall80", .duration_s = 40, .stall_at_s = 20, .stall_ms = 80},
    {.name = "glitch8ms",
     .duration_s = 40,
     .glitch_at_s = 20,
     .glitch_len_s = 1.5,
     .glitch_ms = 8},
    {.name = "glitch20ms",
     .duration_s = 40,
     .glitch_at_s = 20,
     .glitch_len_s = 1.5,
     .glitch_ms = 20},
    {.name = "clockstep20",
     .duration_s = 40,
     .glitch_at_s = 20,
     .glitch_len_s = 100,
     .glitch_ms = 20},
    {.name = "gm_change",
     .duration_s = 40,
     .local_ppm = 20,
     .gm_change_at_s = 20,
     .gm_relock_s = 1.0},
    {.name = "pause_resume",
     .duration_s = 40,
     .pause_at_s = 15,
     .resume_at_s = 22},
    {.name = "skip_deferred", .duration_s = 40, .skip_at_s = 20},
    {.name = "lost_packet", .duration_s = 40, .lose_at_s = 20},
    {.name = "reanchor_same",
     .duration_s = 40,
     .reanchor_every_s = 5,
     .reanchor_shift_ms = 0.3},
    {.name = "reanchor_moved",
     .duration_s = 40,
     .reanchor_every_s = 15,
     .reanchor_shift_ms = 30},
    /* A PCM5102A board as it really is: 32 frames of FIFO and 22 samples of
     * filter after the ring. */
    {.name = "dac_pcm5102a",
     .duration_s = 40,
     .phys = true,
     .phys_fifo_frames = 32,
     .phys_dac_samples = 22},
    /* The same into an AV receiver adding 20 ms, calibrated with the trim. */
    {.name = "avr20_trimmed",
     .duration_s = 40,
     .phys = true,
     .phys_fifo_frames = 32,
     .phys_dac_samples = 22,
     .phys_downstream_us = 20000,
     .trim_us = 20000},
    /* The trim changed mid-stream to what a calibration found. */
    {.name = "trim_to+2ms",
     .duration_s = 40,
     .phys = true,
     .phys_fifo_frames = 32,
     .phys_dac_samples = 22,
     .phys_downstream_us = 2000,
     .trim_at_s = 20,
     .trim_to_us = 2000},
    {.name = "trim_to-3ms",
     .duration_s = 40,
     .phys = true,
     .phys_fifo_frames = 32,
     .phys_dac_samples = 22,
     .phys_downstream_us = -3000,
     .trim_at_s = 20,
     .trim_to_us = -3000},
    {.name = "trim_to+20ms",
     .duration_s = 40,
     .phys = true,
     .phys_fifo_frames = 32,
     .phys_dac_samples = 22,
     .phys_downstream_us = 20000,
     .trim_at_s = 20,
     .trim_to_us = 20000},
};

static scenario_t sc;

/* ---------- time ---------- */

/* True time T (the grandmaster's rate), local time L = T * (1 + ppm).  The
 * DAC is clocked from the local crystal. */
static double true_of_local(double local_ns) {
  return local_ns / (1.0 + sc.local_ppm * 1e-6);
}
__attribute__((unused)) static double local_of_true(double true_ns) {
  return true_ns * (1.0 + sc.local_ppm * 1e-6);
}

/* ---------- clock mock ---------- */

#define GM1 0x1111222233334444ULL
#define GM2 0x5555666677778888ULL

static struct {
  double gm1_offset_ns; /* network - true on each grandmaster */
  double gm2_offset_ns;
  double noise_ns; /* the filter's slowly wandering error */
  double next_sync_true_ns;
} clk;

static bool gm_changed(double true_ns) {
  return sc.gm_change_at_s > 0 && true_ns >= sc.gm_change_at_s * 1e9;
}

/* Error of the clock's estimate of network time at true time T. */
static double clock_error_ns(double true_ns) {
  double e = clk.noise_ns;
  if (sc.glitch_ms > 0 && true_ns >= sc.glitch_at_s * 1e9 &&
      true_ns < (sc.glitch_at_s + sc.glitch_len_s) * 1e9) {
    e += sc.glitch_ms * 1e6;
  }
  return e;
}

static void clock_tick(double true_ns) {
  /* A Sync every 125 ms moves the filter's error a little: AR(1) with a
   * stationary deviation of clock_noise_us. */
  while (true_ns >= clk.next_sync_true_ns) {
    clk.next_sync_true_ns += 125e6;
    const double sigma = sc.clock_noise_us * 1e3;
    clk.noise_ns =
        0.95 * clk.noise_ns + sqrt(1 - 0.95 * 0.95) * sigma * gauss();
  }
}

static double network_of_true(double true_ns, bool second_gm) {
  return true_ns + (second_gm ? clk.gm2_offset_ns : clk.gm1_offset_ns);
}

#ifndef BASELINE
/* The PTP contract, as WP1 provides it: offset and rate tracked (the rate
 * exactly, here), tagged with the grandmaster.  After a grandmaster change it
 * is unlocked for gm_relock_s, then follows the new one. */
static bool contract_state(double true_ns, uint64_t *gm, bool *second) {
  if (!gm_changed(true_ns)) {
    *gm = GM1;
    *second = false;
    return true;
  }
  if (true_ns < (sc.gm_change_at_s + sc.gm_relock_s) * 1e9) {
    return false;
  }
  *gm = GM2;
  *second = true;
  return true;
}

bool ptp_clock_local_to_network(int64_t local_ns, uint64_t *network_ns,
                                uint64_t *gm_id) {
  const double now_true = true_of_local((double)sim_now_us * 1000.0);
  uint64_t gm = 0;
  bool second = false;
  if (!contract_state(now_true, &gm, &second)) {
    return false;
  }
  const double t = true_of_local((double)local_ns);
  *network_ns =
      (uint64_t)llround(network_of_true(t, second) + clock_error_ns(now_true));
  *gm_id = gm;
  return true;
}

bool ptp_clock_network_to_local(uint64_t network_ns, uint64_t gm_id,
                                int64_t *local_ns) {
  const double now_true = true_of_local((double)sim_now_us * 1000.0);
  uint64_t gm = 0;
  bool second = false;
  if (!contract_state(now_true, &gm, &second) || gm != gm_id) {
    return false;
  }
  const double t = (double)network_ns - clock_error_ns(now_true) -
                   (second ? clk.gm2_offset_ns : clk.gm1_offset_ns);
  *local_ns = (int64_t)llround(local_of_true(t));
  return true;
}

uint64_t ptp_clock_get_grandmaster_id(void) {
  uint64_t gm = 0;
  bool second = false;
  return contract_state(true_of_local((double)sim_now_us * 1000.0), &gm,
                        &second)
             ? gm
             : 0;
}

void ptp_clock_set_peers(const uint32_t *addrs, size_t count) {
  (void)addrs;
  (void)count;
}

bool ntp_clock_is_locked(void) {
  return false;
}
int64_t ntp_clock_get_offset_ns(void) {
  return 0;
}
#else
/* What the v0.3.0 receiver saw: one filtered offset, 0 when unlocked.  The
 * anchor pinned the first grandmaster, so after a change its Syncs stop, the
 * offset freezes, and the lock times out 5 s later (ptp_clock.c). */
static double frozen_offset_ns;
static bool frozen;

static bool old_locked(double now_true) {
  return !gm_changed(now_true) || now_true < (sc.gm_change_at_s + 5.0) * 1e9;
}

static int64_t old_offset_ns(double now_local) {
  const double now_true = true_of_local(now_local);
  if (!old_locked(now_true)) {
    return 0;
  }
  if (gm_changed(now_true)) {
    if (!frozen) {
      frozen = true;
      frozen_offset_ns = network_of_true(now_true, false) +
                         clock_error_ns(now_true) - now_local;
    }
    return (int64_t)llround(frozen_offset_ns);
  }
  return (int64_t)llround(network_of_true(now_true, false) +
                          clock_error_ns(now_true) - now_local);
}
#endif

/* ---------- content ---------- */

/* Two ramps a quarter-turn apart per track: whichever is away from its wrap
 * gives the RTP position, and their difference names the track.  A linear
 * ramp passes through the trim interpolator unchanged, so trims do not
 * disturb the decoding; fades and silence fail it and are counted apart. */
#define RAMP 16384
static void encode(uint32_t rtp, int track, int16_t *frame) {
  frame[0] = (int16_t)((int32_t)(rtp % RAMP) - RAMP / 2);
  frame[1] =
      (int16_t)((int32_t)((rtp + RAMP / 2 + (uint32_t)track * 4096U) % RAMP) -
                RAMP / 2);
}

typedef enum { FRAME_SOUND, FRAME_SILENT, FRAME_UNKNOWN } frame_kind_t;

static frame_kind_t decode(const int16_t *frame, uint32_t near, uint32_t *rtp,
                           int *track) {
  if (frame[0] == 0 && frame[1] == 0) {
    return FRAME_SILENT;
  }
  const int a = frame[0] + RAMP / 2;
  for (int t = 0; t < 2; t++) {
    int b = (frame[1] + RAMP / 2 - RAMP / 2 - t * 4096) % RAMP;
    if (b < 0) {
      b += RAMP;
    }
    int d = a - b;
    d = ((d % RAMP) + RAMP + RAMP / 2) % RAMP - RAMP / 2;
    if (abs(d) > 3) {
      continue;
    }
    const int pos = (a > 64 && a < RAMP - 64) ? a : b;
    int32_t delta = (int32_t)((uint32_t)pos - near % RAMP);
    delta = ((delta % RAMP) + RAMP + RAMP / 2) % RAMP - RAMP / 2;
    *rtp = near + (uint32_t)delta;
    *track = t;
    return FRAME_SOUND;
  }
  return FRAME_UNKNOWN;
}

/* ---------- sender ---------- */

typedef struct {
  uint32_t seq;
  uint32_t rtp;
  int track;
  double arrive_true_ns;
  uint32_t epoch;
} packet_t;

static packet_t wire[MAX_PACKETS]; /* sent, not yet read */
static size_t wire_head, wire_count;
static packet_t dq[DQ_CAPACITY]; /* read, not yet decoded */
static size_t dq_head, dq_count;

static struct {
  uint32_t next_seq;
  uint32_t next_rtp;
  int track;
  double last_send_true_ns;
  double last_arrive_true_ns;
  /* The sender's intent: rtp `anchor_rtp` plays at true time anchor_true. */
  uint32_t anchor_rtp;
  double anchor_true_ns;
  bool playing;
  uint32_t paused_rtp;
  bool lose_pending;
} snd;

static double ideal_true_ns(uint32_t rtp) {
  return snd.anchor_true_ns +
         (double)(int32_t)(rtp - snd.anchor_rtp) * 1e9 / FS;
}

static uint32_t sender_play_rtp(double true_ns) {
  if (!snd.playing) {
    return snd.paused_rtp;
  }
  return snd.anchor_rtp +
         (uint32_t)(int64_t)llround((true_ns - snd.anchor_true_ns) * FS / 1e9);
}

static void sender_tick(double true_ns) {
  const uint32_t ahead_limit =
      sender_play_rtp(true_ns) + (uint32_t)(SEND_AHEAD_S * FS);
  while ((int32_t)(snd.next_rtp - ahead_limit) < 0 &&
         wire_count + dq_count < TCP_WINDOW + DQ_CAPACITY &&
         wire_count < TCP_WINDOW && wire_count < MAX_PACKETS) {
    const double send = fmax(true_ns, snd.last_send_true_ns + 1e9 / LINK_PPS);
    if (send > true_ns) {
      break;
    }
    snd.last_send_true_ns = send;
    double delay = 2e6 + -log(1.0 - urand()) * 3e6;
    if (urand() < 0.002) {
      delay += 150e6; /* a Wi-Fi hiccup */
    }
    const double arrive = fmax(snd.last_arrive_true_ns, send + delay);
    snd.last_arrive_true_ns = arrive;
    packet_t p = {.seq = snd.next_seq & 0xFFFFFFU,
                  .rtp = snd.next_rtp,
                  .track = snd.track,
                  .arrive_true_ns = arrive};
    snd.next_seq++;
    snd.next_rtp += FRAME;
    if (snd.lose_pending) {
      snd.lose_pending = false;
      continue; /* never arrives */
    }
    wire[(wire_head + wire_count) % MAX_PACKETS] = p;
    wire_count++;
  }
}

/* ---------- the receiver under test ---------- */

static audio_engine_v2_t engine;
static const audio_format_t format = {.codec = "AAC",
                                      .sample_rate = FS,
                                      .channels = 2,
                                      .bits_per_sample = 16,
                                      .frame_size = FRAME};

static struct {
  uint32_t steps, trims, last_steps, last_trims;
  uint32_t epoch_resets;
} rx_stats;

static void count_scheduler(void) {
  const audio_scheduler_t *s = &engine.scheduler;
  rx_stats.steps += s->drift_steps >= rx_stats.last_steps
                        ? s->drift_steps - rx_stats.last_steps
                        : s->drift_steps;
  rx_stats.trims += s->drift_servo_trims >= rx_stats.last_trims
                        ? s->drift_servo_trims - rx_stats.last_trims
                        : s->drift_servo_trims;
  rx_stats.last_steps = s->drift_steps;
  rx_stats.last_trims = s->drift_servo_trims;
}

#ifndef BASELINE
static audio_playout_clock_t playout_clock;
static audio_flush_queue_t flush_queue;
static bool anchor_pending;
static audio_engine_anchor_t pending_anchor;

static void rx_arm(void) {
  int64_t local = 0;
  if (anchor_pending &&
      ptp_clock_network_to_local(pending_anchor.network_ns,
                                 pending_anchor.timeline_id, &local)) {
    (void)audio_engine_v2_set_rate_anchor(&engine, &pending_anchor);
    anchor_pending = false;
  }
}

static void rx_anchor(uint32_t rtp, uint64_t network_ns, uint64_t gm,
                      bool fas_valid, uint32_t fas) {
  pending_anchor = (audio_engine_anchor_t){.rtp = rtp,
                                           .network_ns = network_ns,
                                           .timeline_id = gm,
                                           .fas_valid = fas_valid,
                                           .fas_rtp = fas};
  anchor_pending = true;
  rx_arm();
  audio_engine_v2_set_playing(&engine, true);
}

static void rx_pause(void) {
  anchor_pending = false;
  audio_engine_v2_stop(&engine);
}

static void rx_flush(bool has_from, uint32_t from_seq, uint32_t until_seq,
                     uint32_t from_ts, uint32_t until_ts) {
  const audio_flush_record_t r = {.has_from = has_from,
                                  .from_seq = from_seq,
                                  .until_seq = until_seq,
                                  .from_ts = from_ts,
                                  .until_ts = until_ts};
  (void)audio_flush_queue_push(&flush_queue, &r);
  if (!has_from) {
    anchor_pending = false;
    audio_engine_v2_stop(&engine);
  }
  (void)audio_engine_v2_flush_seq_range(&engine, has_from, from_seq, until_seq);
}

/* One packet off the decode queue; false when it has to wait. */
static bool rx_decode(const packet_t *p, const int16_t *pcm) {
  uint32_t gen = audio_engine_v2_flush_generation(&engine);
  if (audio_flush_queue_should_drop(&flush_queue, p->seq)) {
    return true;
  }
  for (;;) {
    const audio_engine_push_result_t r = audio_engine_v2_push_pcm_seq_wait(
        &engine, p->epoch, p->rtp, p->seq, gen, pcm, FRAME, CHANNELS, 0);
    if (r == AUDIO_ENGINE_PUSH_OK) {
      return true;
    }
    if (r == AUDIO_ENGINE_PUSH_FAILED) {
      return !audio_epoch_matches(&engine.epoch, p->epoch);
    }
    gen = audio_engine_v2_flush_generation(&engine);
    if (audio_flush_queue_should_drop(&flush_queue, p->seq)) {
      return true;
    }
  }
}

static size_t rx_render(int64_t output_local_ns, int16_t *out) {
  rx_arm();
  return audio_engine_v2_render_local(&engine, output_local_ns, &playout_clock,
                                      out, BLOCK);
}

static void rx_init(void) {
  audio_playout_clock_reset(&playout_clock, AUDIO_PLAYOUT_CLOCK_PTP);
  audio_flush_queue_reset(&flush_queue);
  (void)audio_latency_set_trim_us((int32_t)sc.trim_us);
}

/* What dac.c reports for the board the receiver runs on. */
uint32_t dac_get_output_delay_q4(uint32_t sample_rate_hz,
                                 dac_delay_source_t *source) {
  (void)sample_rate_hz;
  if (source) {
    *source = DAC_DELAY_FROM_PRESET;
  }
  return RX_DAC_Q4;
}

/* The latency the receiver adds past the ring, as audio_output_common.c
 * does: the breakdown with the trim in it. */
static int64_t rx_post_ring_ns(void) {
  audio_latency_t latency;
  audio_latency_compute(FS, RX_FIFO_FRAMES, 0, 0, &latency);
  return (int64_t)latency.post_ring_us * 1000LL;
}
#else
/* The v0.3.0 receiver glue around the engine (audio_receiver.c). */
static struct {
  bool pending;
  uint32_t rtp;
  uint64_t network_ns;
  bool anchor_valid;
  uint32_t anchor_rtp;
  int64_t anchor_local_ns;
  bool paused_valid;
  uint32_t paused_rtp;
  bool deferred;
  uint32_t until_ts;
} old;

static bool old_is_locked(void) {
  return old_locked(true_of_local((double)sim_now_us * 1000.0));
}

static void rx_arm(void) {
  if (old.pending && old_is_locked() &&
      audio_engine_v2_set_anchor(&engine, old.rtp, old.network_ns, 0)) {
    old.pending = false;
  }
}

static void rx_anchor(uint32_t rtp, uint64_t network_ns, uint64_t gm,
                      bool fas_valid, uint32_t fas) {
  (void)gm;
  (void)fas_valid;
  (void)fas;
  /* Path B: a new anchor more than 5 s from where the old one predicts the
   * play position flushes everything. */
  if (old.anchor_valid) {
    uint32_t reference;
    if (old.paused_valid) {
      reference = old.paused_rtp;
      old.paused_valid = false;
    } else {
      const int64_t elapsed_us = sim_now_us - old.anchor_local_ns / 1000;
      reference = old.anchor_rtp + (uint32_t)(elapsed_us * FS / 1000000);
    }
    const int32_t delta = (int32_t)(rtp - reference);
    if (abs(delta) > 5 * FS) {
      (void)audio_engine_v2_begin_epoch(&engine, sim_now_us);
      old.deferred = false;
      rx_stats.epoch_resets++;
    }
  }
  old.anchor_valid = true;
  old.anchor_rtp = rtp;
  old.anchor_local_ns = sim_now_us * 1000;
  old.rtp = rtp;
  old.network_ns = network_ns;
  old.pending = true;
  rx_arm();
  if (old.pending) {
    audio_engine_v2_wait_for_anchor(&engine, sim_now_us);
  }
  audio_engine_v2_set_playing(&engine, true);
}

static void rx_pause(void) {
  if (engine.scheduler.state == AUDIO_SCHED_PLAYING) {
    old.paused_rtp = engine.scheduler.cursor_rtp;
    old.paused_valid = true;
  }
  audio_engine_v2_set_playing(&engine, false);
}

static void rx_flush(bool has_from, uint32_t from_seq, uint32_t until_seq,
                     uint32_t from_ts, uint32_t until_ts) {
  (void)from_seq;
  (void)until_seq;
  (void)from_ts;
  if (has_from) {
    old.until_ts = until_ts;
    old.deferred = true;
  }
}

static bool rx_decode(const packet_t *p, const int16_t *pcm) {
  if (!audio_epoch_matches(&engine.epoch, p->epoch)) {
    return true;
  }
  if (old.deferred && (int32_t)(p->rtp - old.until_ts) >= 0) {
    old.deferred = false;
    (void)audio_engine_v2_deferred_flush(&engine, p->epoch, old.until_ts);
  }
  if (audio_timeline_free_slots(&engine.timeline) == 0U ||
      audio_timeline_phase_blocked(&engine.timeline, p->epoch, p->rtp)) {
    return false;
  }
  (void)audio_engine_v2_push_pcm(&engine, p->epoch, p->rtp, pcm, FRAME,
                                 CHANNELS);
  return true;
}

static size_t rx_render(int64_t output_local_ns, int16_t *out) {
  rx_arm();
  const int64_t network_ns =
      output_local_ns + old_offset_ns((double)sim_now_us * 1000.0);
  return audio_engine_v2_render(&engine, network_ns, out, BLOCK);
}

static void rx_init(void) {
  memset(&old, 0, sizeof(old));
}

static int64_t rx_post_ring_ns(void) {
  return EXTRA_US * 1000LL;
}
#endif

/* ---------- output ---------- */

static int16_t ring[RING_FRAMES][CHANNELS];
static size_t ring_head, ring_count;
static int16_t pending_block[BLOCK + 1][CHANNELS];
static size_t pending_frames, pending_done;

/* ---------- measurement ---------- */

#define MAX_ERRS (44100U * 120U)
static float errs[MAX_ERRS];
static size_t n_errs;
static struct {
  bool heard;
  double first_sound_true_ns;
  uint64_t silent_unplanned;
  uint64_t unknown;
  uint64_t far;         /* more than 100 ms off */
  uint64_t wrong_track; /* the old track where the new one belongs */
  uint64_t new_track;
  uint32_t last_rtp;
  bool last_valid;
  double last_off_true_ns; /* after a trim change: last error over 300 us */
} m;

static bool silence_planned(double true_ns) {
  /* Before the start, and through a pause until the resume has played in. */
  if (!m.heard) {
    return true;
  }
  if (sc.pause_at_s > 0 && true_ns >= sc.pause_at_s * 1e9 &&
      true_ns < (sc.resume_at_s + 1.0) * 1e9) {
    return true;
  }
  return false;
}

static uint32_t skip_from_rtp;
static bool skip_done;

static void measure(const int16_t *frame, double emit_true_ns) {
  const uint32_t near =
      m.last_valid ? m.last_rtp + 1U
                   : snd.anchor_rtp +
                         (uint32_t)(int64_t)llround(
                             (emit_true_ns - snd.anchor_true_ns) * FS / 1e9);
  uint32_t rtp = 0;
  int track = 0;
  const frame_kind_t kind = decode(frame, near, &rtp, &track);
  static frame_kind_t last_kind = FRAME_SILENT;
  if (verbose && kind != last_kind) {
    printf("  %8.3f output %s\n", emit_true_ns / 1e9,
           kind == FRAME_SILENT    ? "silent"
           : kind == FRAME_UNKNOWN ? "unknown"
                                   : "sound");
  }
  last_kind = kind;
  if (kind == FRAME_SILENT) {
    if (!silence_planned(emit_true_ns)) {
      m.silent_unplanned++;
    }
    m.last_valid = false;
    return;
  }
  if (kind == FRAME_UNKNOWN) {
    m.unknown++;
    m.last_valid = false;
    return;
  }
  if (!m.heard) {
    m.heard = true;
    m.first_sound_true_ns = emit_true_ns;
  }
  m.last_rtp = rtp;
  m.last_valid = true;
  if (skip_done && (int32_t)(rtp - skip_from_rtp) >= 0) {
    if (track == 0) {
      m.wrong_track++;
    } else {
      m.new_track++;
    }
  }
  const double err = emit_true_ns - ideal_true_ns(rtp);
  if (fabs(err) > 100e6) {
    m.far++;
  }
  if (sc.trim_at_s > 0 && emit_true_ns >= sc.trim_at_s * 1e9 &&
      fabs(err) > 300e3) {
    m.last_off_true_ns = emit_true_ns;
  }
  /* Settled playout only: from 3 s after the first sound. */
  if (emit_true_ns - m.first_sound_true_ns > 3e9 && n_errs < MAX_ERRS) {
    errs[n_errs++] = (float)err;
  }
}

/* How long after a frame leaves the ring it is heard. */
static double post_ring_ns(void) {
  if (!sc.phys) {
    return (double)rx_post_ring_ns();
  }
  return (sc.phys_fifo_frames + sc.phys_dac_samples) * 1e9 / FS +
         sc.phys_downstream_us * 1e3;
}

/* After a trim change: how long until playout stays within 300 us.  v0.3.0
 * has no trim, so -1 there. */
static double settle_s(void) {
#ifdef BASELINE
  return sc.trim_at_s > 0 ? -1.0 : 0.0;
#else
  return sc.trim_at_s > 0 ? fmax(0.0, m.last_off_true_ns / 1e9 - sc.trim_at_s)
                          : 0.0;
#endif
}

static int cmp_float(const void *a, const void *b) {
  const float x = fabsf(*(const float *)a), y = fabsf(*(const float *)b);
  return (x > y) - (x < y);
}

/* ---------- main loop ---------- */

static void make_pcm(const packet_t *p, int16_t *pcm) {
  for (uint32_t i = 0; i < FRAME; i++) {
    encode(p->rtp + i, p->track, &pcm[i * CHANNELS]);
  }
}

/* SETRATEANCHORTIME rate 1: `rtp` plays at play_true_ns.  The sender's
 * intent moves with it unless this is only a restatement (move_intent false),
 * which carries the sender's own clock reading error. */
static void sender_anchor(uint32_t rtp, double play_true_ns, bool second_gm,
                          bool move_intent) {
  if (move_intent) {
    snd.anchor_rtp = rtp;
    snd.anchor_true_ns = play_true_ns;
  }
  const double network = network_of_true(play_true_ns, second_gm);
  rx_anchor(rtp, (uint64_t)llround(network), second_gm ? GM2 : GM1, true, rtp);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: sim <scenario> [-v]\n");
    return 2;
  }
  verbose = argc > 2 && strcmp(argv[2], "-v") == 0;
  bool found = false;
  for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
    if (strcmp(scenarios[i].name, argv[1]) == 0) {
      sc = scenarios[i];
      found = true;
    }
  }
  if (!found) {
    fprintf(stderr, "unknown scenario %s\n", argv[1]);
    return 2;
  }
  if (sc.clock_noise_us == 0) {
    sc.clock_noise_us = 50;
  }
  clk.gm1_offset_ns = 1.7e18;
  clk.gm2_offset_ns = 1.7e18 + 3600e9 + 123456789.0;

  if (audio_engine_v2_init(&engine, &format, FRAME, BLOCKS) != ESP_OK) {
    return 3;
  }
  rx_init();
  audio_engine_v2_set_playing(&engine, true);

  snd.next_rtp = RTP_ORIGIN;
  snd.playing = true;
  snd.anchor_rtp = RTP_ORIGIN;
  snd.anchor_true_ns = 2.0e9;

  const uint64_t total_frames = (uint64_t)(sc.duration_s * FS);
  bool anchored = false, paused = false, resumed = false, gm_note = false;
#ifndef BASELINE
  bool trim_changed = false;
#endif
  double next_reanchor_s = sc.reanchor_every_s > 0 ? 10.0 : 1e9;
  double next_net_ms = 0;
  int16_t pcm[FRAME * CHANNELS];

  for (uint64_t k = 0; k < total_frames; k++) {
    const double local_ns = (double)k * 1e9 / FS;
    sim_now_us = (int64_t)(local_ns / 1000.0);
    const double true_ns = true_of_local(local_ns);
    clock_tick(true_ns);
#ifndef BASELINE
    if (sc.trim_at_s > 0 && !trim_changed && true_ns >= sc.trim_at_s * 1e9) {
      trim_changed = true;
      (void)audio_latency_set_trim_us((int32_t)sc.trim_to_us);
    }
#endif

    /* ----- the sender and the network, every millisecond ----- */
    if (true_ns >= next_net_ms * 1e6) {
      next_net_ms += 1.0;
      if (!anchored && true_ns >= 0.3e9) {
        anchored = true;
        sender_anchor(RTP_ORIGIN, 2.0e9, false, true);
      }
      if (sc.pause_at_s > 0 && !paused && true_ns >= sc.pause_at_s * 1e9) {
        paused = true;
        snd.paused_rtp = sender_play_rtp(true_ns);
        snd.playing = false;
        rx_pause();
      }
      if (paused && !resumed && true_ns >= sc.resume_at_s * 1e9) {
        resumed = true;
        snd.playing = true;
        /* Resume where it paused, 300 ms from now. */
        sender_anchor(snd.paused_rtp, true_ns + 0.3e9, false, true);
        m.last_valid = false;
      }
      if (sc.reanchor_every_s > 0 && true_ns >= next_reanchor_s * 1e9) {
        next_reanchor_s += sc.reanchor_every_s;
        /* Below a millisecond the sender only restates its timeline, off by
         * its own clock reading; beyond, the timeline really moves. */
        const uint32_t rtp = sender_play_rtp(true_ns + 0.5e9);
        const bool moved = sc.reanchor_shift_ms >= 1.0;
        const double shift_ns =
            sc.reanchor_shift_ms * 1e6 * (moved ? 1.0 : 2.0 * urand() - 1.0);
        sender_anchor(rtp, ideal_true_ns(rtp) + shift_ns, gm_changed(true_ns),
                      moved);
        m.last_valid = false;
      }
      if (gm_changed(true_ns) && !gm_note) {
        gm_note = true;
      }
      if (sc.skip_at_s > 0 && !skip_done && true_ns >= sc.skip_at_s * 1e9) {
        /* Next track from what plays 1 s from now: the sender has streamed
         * past it, so [from, until) is in the timeline, in the decode queue
         * and still on the wire. */
        skip_done = true;
        const uint32_t cut = sender_play_rtp(true_ns + 1.0e9);
        const uint32_t from_index = (cut - RTP_ORIGIN + FRAME - 1) / FRAME;
        const uint32_t from_seq = from_index & 0xFFFFFFU;
        skip_from_rtp = RTP_ORIGIN + from_index * FRAME;
        const uint32_t until_seq = snd.next_seq & 0xFFFFFFU;
        rx_flush(true, from_seq, until_seq, skip_from_rtp, skip_from_rtp);
        snd.track = 1;
        snd.next_rtp = skip_from_rtp;
      }
      if (sc.lose_at_s > 0 && true_ns >= sc.lose_at_s * 1e9) {
        sc.lose_at_s = 0;
        snd.lose_pending = true;
      }
      sender_tick(true_ns);
      /* The reader: off the wire into the decode queue while it has room
       * (no room: the TCP window closes and the sender waits). */
      while (wire_count > 0 && dq_count < DQ_CAPACITY &&
             wire[wire_head].arrive_true_ns <= true_ns) {
        packet_t p = wire[wire_head];
        wire_head = (wire_head + 1) % MAX_PACKETS;
        wire_count--;
        p.epoch = audio_epoch_get(&engine.epoch);
        dq[(dq_head + dq_count) % DQ_CAPACITY] = p;
        dq_count++;
      }
      /* The decode worker: as many as the timeline takes. */
      while (dq_count > 0) {
        const packet_t *p = &dq[dq_head];
        make_pcm(p, pcm);
        if (!rx_decode(p, pcm)) {
          break;
        }
        dq_head = (dq_head + 1) % DQ_CAPACITY;
        dq_count--;
      }
    }

    /* ----- the playback task ----- */
    const bool stalled = sc.stall_ms > 0 && true_ns >= sc.stall_at_s * 1e9 &&
                         true_ns < sc.stall_at_s * 1e9 + sc.stall_ms * 1e6;
    if (!stalled) {
      if (pending_done == pending_frames) {
        /* The write returned: render the next block for the instant its
         * first frame will be heard. */
        const int64_t output_local_ns =
            (int64_t)llround(local_ns + (double)ring_count * 1e9 / FS) +
            rx_post_ring_ns();
        const size_t produced =
            rx_render(output_local_ns, &pending_block[0][0]);
        pending_frames = produced > 0 ? produced : BLOCK;
        if (produced == 0) {
          memset(pending_block, 0, sizeof(pending_block));
        }
        pending_done = 0;
        count_scheduler();
      }
      while (pending_done < pending_frames && ring_count < RING_FRAMES) {
        memcpy(ring[(ring_head + ring_count) % RING_FRAMES],
               pending_block[pending_done], sizeof(ring[0]));
        ring_count++;
        pending_done++;
      }
    }

    /* ----- the DAC: one frame per local sample period ----- */
    int16_t frame[CHANNELS] = {0, 0};
    if (ring_count > 0) {
      memcpy(frame, ring[ring_head], sizeof(frame));
      ring_head = (ring_head + 1) % RING_FRAMES;
      ring_count--;
    }
    measure(frame, true_of_local(local_ns + post_ring_ns()));
  }

  double mean = 0, p95 = 0, max = 0, bias = 0;
  if (n_errs > 0) {
    for (size_t i = 0; i < n_errs; i++) {
      mean += fabs(errs[i]);
      bias += errs[i];
    }
    mean /= (double)n_errs;
    bias /= (double)n_errs;
    qsort(errs, n_errs, sizeof(errs[0]), cmp_float);
    p95 = fabs(errs[(size_t)(0.95 * (double)(n_errs - 1))]);
    max = fabs(errs[n_errs - 1]);
  }
  const double heard_s =
      m.heard ? sc.duration_s - m.first_sound_true_ns / 1e9 : 0;
  printf("RESULT %s mean_us=%.1f p95_us=%.1f max_us=%.1f bias_us=%.1f "
         "silent_ms=%.1f unknown=%" PRIu64 " far=%" PRIu64
         " steps=%u trims_per_s=%.2f wrong_track_ms=%.1f new_track_ms=%.1f "
         "first_sound_s=%.3f flushes=%u settle_s=%.2f\n",
         sc.name, mean / 1e3, p95 / 1e3, max / 1e3, bias / 1e3,
         (double)m.silent_unplanned * 1e3 / FS, m.unknown, m.far,
         rx_stats.steps, heard_s > 0 ? rx_stats.trims / heard_s : 0.0,
         (double)m.wrong_track * 1e3 / FS, (double)m.new_track * 1e3 / FS,
         m.heard ? m.first_sound_true_ns / 1e9 : -1.0, rx_stats.epoch_resets,
         settle_s());
  audio_engine_v2_deinit(&engine);
  return 0;
}

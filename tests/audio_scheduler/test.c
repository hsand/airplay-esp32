// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for the drift servo's trims in main/audio/audio_scheduler.c.  A
 * trim moves the source one frame against the output, and on a sine tone that
 * must not click.  The file is included whole so the tests can reach the
 * resampler as well as audio_scheduler_render(). */

#include "audio_scheduler.c"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define RATE 44100
/* What every playback task asks for. */
#define BLOCK (AUDIO_V2_BLOCK_SAMPLES + 1)
#define AMP   20000.0
/* 20 s of playback. */
#define RENDERS 2500

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

/* The timeline holds a tone on each channel, addressed by RTP: 1600 Hz, which
 * popped at every trim on the board, on the left and 1000 Hz on the right. */
static const double tone_hz[AUDIO_V2_MAX_CHANNELS] = {1600.0, 1000.0};
static bool mock_conceal;

static double tone(double rtp, uint8_t ch) {
  return AMP * sin(2.0 * M_PI * tone_hz[ch] * rtp / RATE + 0.3 * ch);
}

static int16_t tone_sample(uint32_t rtp, uint8_t ch) {
  return (int16_t)lround(tone((double)rtp, ch));
}

size_t audio_timeline_read(audio_timeline_t *timeline, uint32_t epoch,
                           uint32_t start_rtp, int16_t *out,
                           size_t requested_samples, uint8_t channels,
                           bool conceal_missing, size_t *concealed_samples) {
  for (size_t i = 0; i < requested_samples; i++) {
    for (uint8_t ch = 0; ch < channels; ch++) {
      out[i * channels + ch] = tone_sample(start_rtp + (uint32_t)i, ch);
    }
  }
  if (concealed_samples) {
    *concealed_samples = mock_conceal ? requested_samples : 0U;
  }
  return requested_samples;
}

bool audio_timeline_has_playable_from(audio_timeline_t *timeline,
                                      uint32_t epoch, uint32_t target_rtp) {
  return true;
}

bool audio_timeline_find_contiguous_from(audio_timeline_t *timeline,
                                         uint32_t epoch, uint32_t target_rtp,
                                         uint32_t required_samples,
                                         uint32_t max_future_samples,
                                         uint32_t *start_rtp) {
  *start_rtp = target_rtp;
  return true;
}

void audio_timeline_set_playback_floor(audio_timeline_t *timeline,
                                       uint32_t epoch, uint32_t floor_rtp) {
}

bool audio_timeline_is_nearly_full(audio_timeline_t *timeline) {
  return false;
}

size_t audio_timeline_trim_before(audio_timeline_t *timeline, uint32_t epoch,
                                  uint32_t rtp) {
  return 0;
}

/* The sender's clock runs drift_ppm faster than the DAC's. */
static double drift_ppm;

bool audio_clock_map_network_to_rtp(const audio_clock_map_t *map,
                                    int64_t network_ns, uint32_t *rtp) {
  const double elapsed_s =
      (double)(network_ns - (int64_t)map->anchor_network_ns) * 1e-9;
  *rtp = map->anchor_rtp + (uint32_t)llround(elapsed_s * map->sample_rate *
                                             (1.0 + drift_ppm * 1e-6));
  return true;
}

/* ---------- helpers ---------- */

static const audio_clock_map_t clock_map = {
    .valid = true,
    .sample_rate = RATE,
    .anchor_rtp = 5000,
    .anchor_network_ns = 2000000000ULL,
};

static audio_timeline_t timeline;

/* Start playing, as the first render after an anchor does. */
static void start(audio_scheduler_t *scheduler) {
  int16_t block[BLOCK * AUDIO_V2_MAX_CHANNELS];
  audio_scheduler_init(scheduler, RATE / 4, 0);
  audio_scheduler_begin_epoch(scheduler, 1, mock_now_us);
  (void)audio_scheduler_render(scheduler, &timeline, &clock_map,
                               (int64_t)clock_map.anchor_network_ns, block,
                               BLOCK, AUDIO_V2_MAX_CHANNELS, NULL);
}

/* Arm the servo so the next render trims: warmed up, rate limit expired, and
 * a full queue of correction behind it. */
static void arm_trim(audio_scheduler_t *scheduler, int direction) {
  scheduler->drift_servo_warmup = DRIFT_SERVO_WARMUP_RENDERS;
  scheduler->drift_servo_phase = DRIFT_SERVO_MIN_TRIM_INTERVAL;
  scheduler->drift_servo_accum = -direction * DRIFT_SERVO_ACCUM_LIMIT;
}

/* ---------- tests ---------- */

/* Every set of weights sums to exactly 1 and reads a tone at the position it
 * was asked for, within the -78 dB the comment on trim_kernel() claims. */
static void test_kernel(void) {
  printf("kernel\n");
  static const double freqs[] = {400, 1600, 5000, 8000, 10000, 12000};
  double worst = 0.0;
  for (int32_t mu = 1; mu < 65536; mu += 13) {
    int32_t weight[TRIM_TAPS];
    trim_kernel(mu, weight);
    int64_t sum = 0;
    for (int t = 0; t < TRIM_TAPS; t++) {
      sum += weight[t];
    }
    CHECK(sum == 1 << 20, "mu %d: weights sum to %lld", (int)mu,
          (long long)sum);
    for (size_t f = 0; f < sizeof(freqs) / sizeof(freqs[0]); f++) {
      const double w = 2.0 * M_PI * freqs[f] / RATE;
      double re = 0.0;
      double im = 0.0;
      for (int t = 0; t < TRIM_TAPS; t++) {
        re += weight[t] / 1048576.0 * cos(w * (t + 1 - TRIM_REACH));
        im += weight[t] / 1048576.0 * sin(w * (t + 1 - TRIM_REACH));
      }
      const double error =
          hypot(re - cos(w * mu / 65536.0), im - sin(w * mu / 65536.0));
      if (error > worst) {
        worst = error;
      }
    }
  }
  printf("  worst %.1f dB\n", 20.0 * log10(worst));
  CHECK(20.0 * log10(worst) < -78.0, "kernel off by %.1f dB",
        20.0 * log10(worst));
}

/* A trimmed block of the tone is the tone read at the positions the ramp
 * asks for.  Outside the ramp it is the source exactly as read, one frame
 * apart by the end of the block. */
static void test_block(int direction) {
  printf("%s block\n", direction < 0 ? "stretch" : "shrink");
  const uint32_t first = 123456;
  const size_t sources = direction < 0 ? BLOCK - 1U : BLOCK;
  int16_t out[BLOCK * AUDIO_V2_MAX_CHANNELS];
  int16_t spare[AUDIO_V2_MAX_CHANNELS];
  for (size_t k = 0; k < BLOCK; k++) {
    for (uint8_t ch = 0; ch < AUDIO_V2_MAX_CHANNELS; ch++) {
      /* A stretch leaves the last frame unread; make reading it show. */
      out[k * AUDIO_V2_MAX_CHANNELS + ch] =
          k < sources ? tone_sample(first + (uint32_t)k, ch) : INT16_MAX;
    }
  }
  for (uint8_t ch = 0; ch < AUDIO_V2_MAX_CHANNELS; ch++) {
    spare[ch] = tone_sample(first + BLOCK, ch);
  }
  trim_resample(out, BLOCK, AUDIO_V2_MAX_CHANNELS, direction,
                direction > 0 ? spare : NULL);

  double worst = 0.0;
  for (size_t k = 0; k < BLOCK; k++) {
    const double pos =
        (double)k + direction * trim_progress(k, BLOCK) / 65536.0;
    for (uint8_t ch = 0; ch < AUDIO_V2_MAX_CHANNELS; ch++) {
      const int16_t got = out[k * AUDIO_V2_MAX_CHANNELS + ch];
      const double error = fabs(got - tone(first + pos, ch));
      if (error > worst) {
        worst = error;
      }
      if (k <= TRIM_REACH) {
        CHECK(got == tone_sample(first + (uint32_t)k, ch),
              "frame %zu changed before the ramp", k);
      }
      if (k >= BLOCK - TRIM_REACH) {
        CHECK(got == tone_sample(first + (uint32_t)(k + direction), ch),
              "frame %zu not the source frame %+d", k, direction);
      }
    }
  }
  printf("  worst %.2f LSB\n", worst);
  /* Rounding of the source and of the result, and the kernel's error. */
  CHECK(worst < 2.0, "%.2f LSB from the tone", worst);
}

/* Through audio_scheduler_render(), with the sender's clock off by `ppm`: the
 * servo trims, playout stays put, and the trims cannot be found in the second
 * difference of either tone.  A frame repeated or dropped near a zero crossing
 * of the 1600 Hz tone makes it about four times the tone's own. */
static void test_render(double ppm) {
  printf("render, sender %+.0f ppm\n", ppm);
  drift_ppm = ppm;
  audio_scheduler_t scheduler;
  start(&scheduler);

  double limit[AUDIO_V2_MAX_CHANNELS];
  for (uint8_t ch = 0; ch < AUDIO_V2_MAX_CHANNELS; ch++) {
    /* The tone's own, played up to 0.56 % fast, plus rounding. */
    const double s = sin(M_PI * tone_hz[ch] / RATE);
    limit[ch] = 4.0 * s * s * AMP * 1.012 + 8.0;
  }
  double worst[AUDIO_V2_MAX_CHANNELS] = {0};
  int32_t last[2][AUDIO_V2_MAX_CHANNELS] = {{0}};
  size_t frames = 0;
  int16_t block[BLOCK * AUDIO_V2_MAX_CHANNELS];
  for (int64_t i = 1; i <= RENDERS; i++) {
    const int64_t network_ns =
        (int64_t)clock_map.anchor_network_ns + i * BLOCK * 1000000000LL / RATE;
    const size_t produced =
        audio_scheduler_render(&scheduler, &timeline, &clock_map, network_ns,
                               block, BLOCK, AUDIO_V2_MAX_CHANNELS, NULL);
    CHECK(produced == BLOCK, "render %lld produced %zu", (long long)i,
          produced);
    for (size_t k = 0; k < produced; k++, frames++) {
      for (uint8_t ch = 0; ch < AUDIO_V2_MAX_CHANNELS; ch++) {
        const int32_t now = block[k * AUDIO_V2_MAX_CHANNELS + ch];
        const double d2 = fabs((double)now - 2.0 * last[1][ch] + last[0][ch]);
        if (frames >= 2 && d2 > worst[ch]) {
          worst[ch] = d2;
        }
        last[0][ch] = last[1][ch];
        last[1][ch] = now;
      }
    }
  }
  printf("  %u trims, error %d samples, second difference %.0f/%.0f and "
         "%.0f/%.0f\n",
         (unsigned)scheduler.drift_servo_trims,
         (int)scheduler.playout_error_samples, worst[0], limit[0], worst[1],
         limit[1]);
  /* Every trim makes up one frame of the drift, and what is left of it is the
   * error the servo settles at: ~46 samples at 200 ppm. */
  const double drift = RENDERS * BLOCK * fabs(ppm) * 1e-6;
  const double made_up = drift - abs(scheduler.playout_error_samples);
  CHECK(fabs(scheduler.drift_servo_trims - made_up) < 3.0,
        "%u trims for %.1f frames of drift",
        (unsigned)scheduler.drift_servo_trims, made_up);
  CHECK(abs(scheduler.playout_error_samples) < 80, "playout %d samples off",
        (int)scheduler.playout_error_samples);
  for (uint8_t ch = 0; ch < AUDIO_V2_MAX_CHANNELS; ch++) {
    CHECK(worst[ch] <= limit[ch], "channel %u: second difference %.0f > %.0f",
          ch, worst[ch], limit[ch]);
  }
}

/* A shrink reads a frame past the block.  Concealment in either read is
 * counted; the second read used to overwrite the first's count. */
static void test_concealed_count(void) {
  printf("concealed count\n");
  drift_ppm = 0.0;
  audio_scheduler_t scheduler;
  start(&scheduler);
  arm_trim(&scheduler, 1);
  mock_conceal = true;
  int16_t block[BLOCK * AUDIO_V2_MAX_CHANNELS];
  size_t concealed = 0;
  const uint32_t trims = scheduler.drift_servo_trims;
  (void)audio_scheduler_render(&scheduler, &timeline, &clock_map,
                               (int64_t)clock_map.anchor_network_ns, block,
                               BLOCK, AUDIO_V2_MAX_CHANNELS, &concealed);
  mock_conceal = false;
  CHECK(scheduler.drift_servo_trims == trims + 1U, "no trim");
  CHECK(concealed == BLOCK + 1U, "%zu concealed, %u read", concealed,
        (unsigned)BLOCK + 1U);
}

/* A block too short to carry a ramp is not trimmed; the correction waits. */
static void test_short_block(void) {
  printf("short block\n");
  drift_ppm = 0.0;
  audio_scheduler_t scheduler;
  start(&scheduler);
  arm_trim(&scheduler, 1);
  int16_t block[BLOCK * AUDIO_V2_MAX_CHANNELS];
  const uint32_t cursor = scheduler.cursor_rtp;
  (void)audio_scheduler_render(
      &scheduler, &timeline, &clock_map, (int64_t)clock_map.anchor_network_ns,
      block, TRIM_MIN_FRAMES - 1U, AUDIO_V2_MAX_CHANNELS, NULL);
  CHECK(scheduler.drift_servo_trims == 0, "trimmed a short block");
  CHECK(scheduler.cursor_rtp - cursor == TRIM_MIN_FRAMES - 1U,
        "cursor moved %u", (unsigned)(scheduler.cursor_rtp - cursor));
}

/* Playout knocked `jump_ms` off at once, as by a stall of the playback task
 * (late, positive) or the mirror (early, negative): one step makes it up,
 * fading out and back in, where the servo would trim at its limit for over a
 * minute.  The step must not click: a hard jump on the 1600 Hz tone makes a
 * second difference of up to twice its amplitude. */
typedef struct {
  uint32_t steps;
  uint32_t trims_after_step;
  int32_t error;
  double worst; /* the largest second difference on channel 0 */
} step_run_t;

static step_run_t run_step(int64_t jump_ns) {
  drift_ppm = 0.0;
  audio_scheduler_t scheduler;
  start(&scheduler);
  const int64_t at = DRIFT_SERVO_WARMUP_RENDERS + 100;
  step_run_t run = {0};
  int32_t last[2] = {0};
  size_t frames = 0;
  int16_t block[BLOCK * AUDIO_V2_MAX_CHANNELS];
  for (int64_t i = 1; i <= at + 500; i++) {
    const int64_t network_ns = (int64_t)clock_map.anchor_network_ns +
                               i * BLOCK * 1000000000LL / RATE +
                               (i >= at ? jump_ns : 0);
    const uint32_t trims = scheduler.drift_servo_trims;
    const size_t produced =
        audio_scheduler_render(&scheduler, &timeline, &clock_map, network_ns,
                               block, BLOCK, AUDIO_V2_MAX_CHANNELS, NULL);
    CHECK(produced == BLOCK, "render %lld produced %zu", (long long)i,
          produced);
    if (scheduler.drift_steps > 0U) {
      run.trims_after_step += scheduler.drift_servo_trims - trims;
    }
    for (size_t k = 0; k < produced; k++, frames++) {
      const int32_t now = block[k * AUDIO_V2_MAX_CHANNELS];
      const double d2 = fabs((double)now - 2.0 * last[1] + last[0]);
      if (frames >= 2 && d2 > run.worst) {
        run.worst = d2;
      }
      last[0] = last[1];
      last[1] = now;
    }
  }
  run.steps = scheduler.drift_steps;
  run.error = scheduler.playout_error_samples;
  return run;
}

/* Playout knocked `jump_ms` off at once, as by a stall of the playback task
 * (late, positive) or the mirror (early, negative): one step makes it up,
 * fading out and back in, where the servo would trim at its limit for over a
 * minute.  The step must not click: a hard jump on the 1600 Hz tone makes a
 * second difference of up to twice its amplitude. */
static void test_step(double jump_ms) {
  printf("step, playout %+.0f ms\n", jump_ms);
  const step_run_t run = run_step((int64_t)(jump_ms * 1e6));
  printf("  %u step(s), %u trims after, error %d samples, second difference "
         "%.0f\n",
         (unsigned)run.steps, (unsigned)run.trims_after_step, (int)run.error,
         run.worst);
  CHECK(run.steps == 1U, "%u steps", (unsigned)run.steps);
  CHECK(abs(run.error) <= 2, "playout %d samples off", (int)run.error);
  CHECK(run.trims_after_step <= 2U, "%u trims after the step",
        (unsigned)run.trims_after_step);
  CHECK(run.worst < AMP / 4.0, "second difference %.0f: it clicks", run.worst);
}

/* An early step holds the cursor behind silence for as long as the error,
 * which ends anywhere in a block: at its very end, the fade back in has no
 * frames left in it, and a few frames short of it, almost none.  Step by one
 * frame through a whole block's worth of holds. */
static void test_step_hold_ends(void) {
  printf("early steps, the hold ending at every point in a block\n");
  double worst = 0.0;
  int clicks = 0, bad_steps = 0;
  for (int offset = 0; offset < BLOCK; offset++) {
    const int64_t jump_ns = -40000000LL - (int64_t)offset * 1000000000LL / RATE;
    const step_run_t run = run_step(jump_ns);
    if (run.worst > worst) {
      worst = run.worst;
    }
    clicks += run.worst >= AMP / 4.0;
    bad_steps += run.steps != 1U;
  }
  printf("  %d of %d click, %d without exactly one step, worst second "
         "difference %.0f\n",
         clicks, BLOCK, bad_steps, worst);
  CHECK(clicks == 0, "%d of %d holds click", clicks, BLOCK);
  CHECK(bad_steps == 0, "%d holds without exactly one step", bad_steps);
}

/* Render `renders` blocks from render `from` on, with playout knocked off by
 * jump_ns and the clock offset the caller reports moved by offset_ns from
 * render `at` on.  Returns the render at which the first step began, or -1. */
static int64_t run_until_step(audio_scheduler_t *scheduler, int64_t from,
                              int64_t renders, int64_t at, int64_t jump_ns,
                              int64_t offset_ns) {
  int16_t block[BLOCK * AUDIO_V2_MAX_CHANNELS];
  for (int64_t i = from; i < from + renders; i++) {
    const int64_t network_ns = (int64_t)clock_map.anchor_network_ns +
                               i * BLOCK * 1000000000LL / RATE +
                               (i >= at ? jump_ns : 0);
    audio_scheduler_set_clock_offset(scheduler, true, i >= at ? offset_ns : 0);
    const uint32_t steps = scheduler->drift_steps;
    (void)audio_scheduler_render(scheduler, &timeline, &clock_map, network_ns,
                                 block, BLOCK, AUDIO_V2_MAX_CHANNELS, NULL);
    if (scheduler->drift_steps != steps) {
      return i;
    }
  }
  return -1;
}

/* The step's premise changed in this tree: it used to fire after 4 renders
 * (32 ms) past 5 ms.  Now an error of the audio path's own -- the clock
 * offset steady -- is stepped once it has held DRIFT_STEP_CONFIRM_MS past
 * DRIFT_STEP_MS, and not before. */
static void test_step_persistence(void) {
  printf("step persistence: 15 ms off with the clock steady\n");
  drift_ppm = 0.0;
  audio_scheduler_t scheduler;
  start(&scheduler);
  const int64_t at = DRIFT_SERVO_WARMUP_RENDERS + 50;
  const int64_t step =
      run_until_step(&scheduler, 1, at + 400, at, 15000000LL, 0);
  const double after_ms = (double)(step - at) * BLOCK * 1000.0 / RATE;
  printf("  stepped %.0f ms after the error appeared\n", after_ms);
  CHECK(step > 0, "no step");
  /* Plus the few renders the 1/8 filter takes to cross DRIFT_STEP_MS. */
  CHECK(after_ms >= DRIFT_STEP_CONFIRM_MS &&
            after_ms <= DRIFT_STEP_CONFIRM_MS + 80,
        "stepped after %.0f ms", after_ms);
}

/* The same error, made by the clock: the offset jumped with it.  Nothing is
 * stepped while the clock may jump back (CLOCK_JUMP_HOLD_MS), the servo does
 * not chase it, and an excursion that ends inside that leaves no trace. */
static void test_step_clock_excursion(void) {
  printf("clock excursion: 15 ms for 1.5 s, then back\n");
  drift_ppm = 0.0;
  audio_scheduler_t scheduler;
  start(&scheduler);
  const int64_t at = DRIFT_SERVO_WARMUP_RENDERS + 50;
  const int64_t excursion = (int64_t)(1.5 * RATE / BLOCK);
  const uint32_t trims = scheduler.drift_servo_trims;
  int64_t step = run_until_step(&scheduler, 1, at + excursion - 1, at,
                                15000000LL, 15000000LL);
  CHECK(step < 0, "stepped at render %lld during the excursion",
        (long long)step);
  step = run_until_step(&scheduler, at + excursion, 600, at + excursion + 600,
                        0, 0);
  CHECK(step < 0, "stepped at render %lld after it", (long long)step);
  CHECK(scheduler.drift_servo_trims - trims <= 1, "%u trims chasing the clock",
        (unsigned)(scheduler.drift_servo_trims - trims));
  CHECK(scheduler.servo_holds == 2, "%u holds",
        (unsigned)scheduler.servo_holds);

  printf("clock step: 15 ms for good\n");
  start(&scheduler);
  step = run_until_step(&scheduler, 1, at + 800, at, 15000000LL, 15000000LL);
  const double after_ms = (double)(step - at) * BLOCK * 1000.0 / RATE;
  printf("  stepped %.0f ms after the clock moved\n", after_ms);
  CHECK(step > 0 && after_ms >= CLOCK_JUMP_HOLD_MS + DRIFT_STEP_CONFIRM_MS - 20,
        "stepped after %.0f ms", after_ms);
}

/* A new anchor that moved the timeline under a playing stream: the first
 * measurement goes there at once, as Apple re-sets its offset on the first
 * measurement after an anchor.  Within ANCHOR_RESET_MIN_US it is left to the
 * servo. */
static void test_anchor_reset(void) {
  printf("anchor reset: the first measurement after a moving anchor\n");
  drift_ppm = 0.0;
  audio_scheduler_t scheduler;
  start(&scheduler);
  const int64_t at = DRIFT_SERVO_WARMUP_RENDERS + 50;
  (void)run_until_step(&scheduler, 1, at - 1, at, 0, 0);
  audio_scheduler_new_anchor(&scheduler, true);
  int64_t step = run_until_step(&scheduler, at, 3, at, 30000000LL, 0);
  CHECK(step == at, "reset at render %lld, not %lld", (long long)step,
        (long long)at);
  CHECK(scheduler.anchor_resets == 1, "%u resets",
        (unsigned)scheduler.anchor_resets);

  start(&scheduler);
  (void)run_until_step(&scheduler, 1, at - 1, at, 0, 0);
  audio_scheduler_new_anchor(&scheduler, true);
  step = run_until_step(&scheduler, at, 100, at, 1000000LL, 0);
  CHECK(step < 0 && scheduler.anchor_resets == 0, "a 1 ms move is the servo's");
}

/* First audible sample: silence up to it, the tone from exactly it. */
static void test_first_audible(void) {
  printf("first audible sample\n");
  drift_ppm = 0.0;
  audio_scheduler_t scheduler;
  audio_scheduler_init(&scheduler, RATE / 4, 0);
  audio_scheduler_begin_epoch(&scheduler, 1, mock_now_us);
  const uint32_t fas = clock_map.anchor_rtp + 1000U;
  audio_scheduler_set_first_audible(&scheduler, true, fas);
  int16_t block[BLOCK * AUDIO_V2_MAX_CHANNELS];
  int64_t first_sound = -1;
  for (int64_t i = 0; i < 5 && first_sound < 0; i++) {
    const int64_t network_ns =
        (int64_t)clock_map.anchor_network_ns + i * BLOCK * 1000000000LL / RATE;
    (void)audio_scheduler_render(&scheduler, &timeline, &clock_map, network_ns,
                                 block, BLOCK, AUDIO_V2_MAX_CHANNELS, NULL);
    for (size_t k = 0; k < BLOCK; k++) {
      if (block[k * AUDIO_V2_MAX_CHANNELS] != 0 ||
          block[k * AUDIO_V2_MAX_CHANNELS + 1] != 0) {
        first_sound = i * BLOCK + (int64_t)k;
        /* The start's fade-in begins on it: its first step. */
        const uint32_t len = RATE * AUDIO_SCHED_START_FADE_MS / 1000U;
        const int32_t gain = (int32_t)((1ULL << 15) / (len + 1U));
        CHECK(block[k * AUDIO_V2_MAX_CHANNELS] ==
                  (int16_t)((int32_t)tone_sample(fas, 0) * gain / 32768),
              "first sound is not the first audible sample");
        break;
      }
    }
  }
  CHECK(first_sound == 1000, "first sound at frame %lld, wanted 1000",
        (long long)first_sound);
  CHECK(!scheduler.fas_valid, "gate still armed");
}

/* The ramp every fade uses, sample for sample as audio_scheduler.c has it. */
static int16_t ramp_sample(int16_t x, uint32_t num, uint32_t len) {
  const int32_t gain = (int32_t)(((uint64_t)num << 15) / (len + 1U));
  return (int16_t)((int32_t)x * gain / 32768);
}

/* The largest step between neighbouring samples of the unfaded left tone. */
static int max_tone_step(void) {
  return (int)ceil(AMP * 2.0 * M_PI * tone_hz[0] / RATE) + 1;
}

/* Every start fades in over AUDIO_SCHED_START_FADE_MS, straight from 0, and
 * plays the tone untouched after it.  Then a stop: the tail continues the
 * waveform from the cursor and fades it to nothing, with no step at the join
 * or at the end. */
static void test_start_and_stop_fades(void) {
  printf("start fade-in and stop fade-out\n");
  drift_ppm = 0.0;
  audio_scheduler_t scheduler;
  audio_scheduler_init(&scheduler, RATE / 4, 0);
  audio_scheduler_begin_epoch(&scheduler, 1, mock_now_us);
  const uint32_t len = RATE * AUDIO_SCHED_START_FADE_MS / 1000U;
  enum { BLOCKS = 8 };
  static int16_t out[BLOCKS * BLOCK * AUDIO_V2_MAX_CHANNELS];
  uint32_t first_rtp = 0;
  for (int b = 0; b < BLOCKS; b++) {
    const int64_t network_ns = (int64_t)clock_map.anchor_network_ns +
                               (int64_t)b * BLOCK * 1000000000LL / RATE;
    (void)audio_scheduler_render(&scheduler, &timeline, &clock_map, network_ns,
                                 &out[(size_t)b * BLOCK * AUDIO_V2_MAX_CHANNELS],
                                 BLOCK, AUDIO_V2_MAX_CHANNELS, NULL);
    if (b == 0) {
      first_rtp = scheduler.cursor_rtp - BLOCK;
    }
  }
  CHECK(scheduler.cursor_rtp == first_rtp + BLOCKS * BLOCK,
        "playout not contiguous");
  CHECK(scheduler.start_fades == 1, "%u fades", (unsigned)scheduler.start_fades);
  int wrong = 0;
  int worst_step = 0;
  for (uint32_t k = 0; k < BLOCKS * BLOCK; k++) {
    for (uint8_t ch = 0; ch < AUDIO_V2_MAX_CHANNELS; ch++) {
      const int16_t x = tone_sample(first_rtp + k, ch);
      const int16_t want = k < len ? ramp_sample(x, k + 1U, len) : x;
      wrong += out[k * AUDIO_V2_MAX_CHANNELS + ch] != want;
    }
    const int prev = k ? out[(k - 1) * AUDIO_V2_MAX_CHANNELS] : 0;
    const int step = abs(out[k * AUDIO_V2_MAX_CHANNELS] - prev);
    worst_step = step > worst_step ? step : worst_step;
  }
  CHECK(wrong == 0, "%d samples off the fade-in", wrong);
  CHECK(abs(out[0]) <= (int)(AMP / (len + 1)) + 1, "starts at %d", out[0]);
  CHECK(worst_step <= max_tone_step(), "step of %d from silence", worst_step);
  printf("  fade-in %u frames, first sample %d, largest step %d (tone %d)\n",
         (unsigned)len, out[0], worst_step, max_tone_step());

  /* Stop. */
  const uint32_t tail_len = audio_scheduler_fade_out_frames(RATE);
  static int16_t tail[96 * AUDIO_SCHED_FADE_OUT_MS * AUDIO_V2_MAX_CHANNELS];
  const uint32_t cursor = scheduler.cursor_rtp;
  const size_t taken = audio_scheduler_take_tail(
      &scheduler, &timeline, tail, tail_len, AUDIO_V2_MAX_CHANNELS);
  CHECK(taken == tail_len, "tail of %zu frames, wanted %u", taken,
        (unsigned)tail_len);
  wrong = 0;
  worst_step = abs(tail[0] - out[(BLOCKS * BLOCK - 1) * AUDIO_V2_MAX_CHANNELS]);
  for (uint32_t k = 0; k < tail_len; k++) {
    for (uint8_t ch = 0; ch < AUDIO_V2_MAX_CHANNELS; ch++) {
      const int16_t want =
          ramp_sample(tone_sample(cursor + k, ch), tail_len - k, tail_len);
      wrong += tail[k * AUDIO_V2_MAX_CHANNELS + ch] != want;
    }
    if (k) {
      const int step = abs(tail[k * AUDIO_V2_MAX_CHANNELS] -
                           tail[(k - 1) * AUDIO_V2_MAX_CHANNELS]);
      worst_step = step > worst_step ? step : worst_step;
    }
  }
  const int last = tail[(tail_len - 1) * AUDIO_V2_MAX_CHANNELS];
  CHECK(wrong == 0, "%d samples off the fade-out", wrong);
  CHECK(abs(last) <= (int)(AMP / (tail_len + 1)) + 1, "ends at %d", last);
  CHECK(worst_step <= max_tone_step(), "step of %d into the tail", worst_step);
  CHECK(scheduler.cursor_rtp == cursor + tail_len, "cursor not moved on");
  printf("  fade-out %u frames, last sample %d, largest step %d\n",
         (unsigned)tail_len, last, worst_step);

  /* A stop inside the fade-in fades from where the fade-in had got to. */
  audio_scheduler_begin_epoch(&scheduler, 2, mock_now_us);
  (void)audio_scheduler_render(&scheduler, &timeline, &clock_map,
                               (int64_t)clock_map.anchor_network_ns, out,
                               BLOCK, AUDIO_V2_MAX_CHANNELS, NULL);
  const int16_t before = out[(BLOCK - 1) * AUDIO_V2_MAX_CHANNELS];
  (void)audio_scheduler_take_tail(&scheduler, &timeline, tail, tail_len,
                                  AUDIO_V2_MAX_CHANNELS);
  const int join = abs(tail[0] - before);
  CHECK(join <= max_tone_step() * (int)BLOCK / (int)len + 2,
        "a stop during the fade-in jumps by %d", join);
  CHECK(scheduler.start_fade_len == 0, "fade-in still armed after a stop");

  /* Nothing playing yet: no tail. */
  audio_scheduler_begin_epoch(&scheduler, 3, mock_now_us);
  CHECK(audio_scheduler_take_tail(&scheduler, &timeline, tail, tail_len,
                                  AUDIO_V2_MAX_CHANNELS) == 0,
        "a tail before playout started");
}

int main(void) {
  test_kernel();
  test_block(-1);
  test_block(1);
  test_render(200.0);
  test_render(-200.0);
  test_concealed_count();
  test_short_block();
  test_step(40.0);
  test_step(-40.0);
  test_step_hold_ends();
  test_step_persistence();
  test_step_clock_excursion();
  test_anchor_reset();
  test_first_audible();
  test_start_and_stop_fades();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}

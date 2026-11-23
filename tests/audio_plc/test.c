// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for main/audio/audio_plc.c, the concealment of holes in the
 * timeline: a lost packet in a tone, against the silence v0.3.0 played, and
 * the fade, the crossfades and the resets. */

#include "audio_plc.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RATE  44100.0
#define CH    2
#define BLOCK 352
#define AMP   16000.0

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

static double tone_hz = 440.0;

static int16_t tone(uint32_t frame, int ch) {
  return (int16_t)lround(AMP *
                         sin(2 * M_PI * tone_hz * frame / RATE + 0.5 * ch));
}

static void fill(int16_t *pcm, uint32_t first, size_t frames) {
  for (size_t k = 0; k < frames; k++) {
    for (int ch = 0; ch < CH; ch++) {
      pcm[k * CH + ch] = tone(first + (uint32_t)k, ch);
    }
  }
}

/* Plays `frames` frames of the tone with a hole of `hole` frames at
 * `hole_at`, through the concealer (or as silence), reading in BLOCK-sized
 * pieces as the timeline does.  Returns the output. */
static int16_t *play(audio_plc_t *plc, size_t frames, size_t hole_at,
                     size_t hole, bool conceal) {
  int16_t *out = calloc(frames * CH, sizeof(int16_t));
  for (size_t at = 0; at < frames;) {
    size_t n = BLOCK;
    if (at < hole_at && at + n > hole_at) {
      n = hole_at - at;
    } else if (at >= hole_at && at < hole_at + hole) {
      n = hole_at + hole - at < n ? hole_at + hole - at : n;
    }
    if (at + n > frames) {
      n = frames - at;
    }
    int16_t *p = &out[at * CH];
    if (at >= hole_at && at < hole_at + hole) {
      if (conceal) {
        audio_plc_conceal(plc, p, n, CH);
      } else {
        memset(p, 0, n * CH * sizeof(int16_t));
      }
    } else {
      fill(p, (uint32_t)at, n);
      if (conceal) {
        audio_plc_good(plc, p, n, CH);
      }
    }
    at += n;
  }
  return out;
}

// The largest sample-to-sample step: a click shows up as one.
static double max_step(const int16_t *pcm, size_t from, size_t to) {
  double worst = 0;
  for (size_t k = from + 1; k < to; k++) {
    for (int ch = 0; ch < CH; ch++) {
      const double d = fabs((double)pcm[k * CH + ch] - pcm[(k - 1) * CH + ch]);
      worst = fmax(worst, d);
    }
  }
  return worst;
}

// Error energy against the true signal over [from, to), dB below the tone.
static double error_db(const int16_t *pcm, size_t from, size_t to) {
  double e = 0, sig = 0;
  for (size_t k = from; k < to; k++) {
    for (int ch = 0; ch < CH; ch++) {
      const double t = tone((uint32_t)k, ch);
      const double d = pcm[k * CH + ch] - t;
      e += d * d;
      sig += t * t;
    }
  }
  return 10 * log10(e / sig);
}

static void test_one_lost_packet(void) {
  printf("one lost packet in a tone: concealed against silence\n");
  const double tones[] = {440.0, 1000.0, 3000.0};
  for (size_t t = 0; t < 3; t++) {
    tone_hz = tones[t];
    // The steepest a clean tone ever steps, for scale.
    const double natural = 2 * M_PI * tone_hz / RATE * AMP;
    audio_plc_t plc;
    CHECK(audio_plc_init(&plc), "no memory");
    const size_t at = 10 * BLOCK, hole = BLOCK, total = 20 * BLOCK;
    int16_t *silence = play(&plc, total, at, hole, false);
    int16_t *concealed = play(&plc, total, at, hole, true);
    const double s_step = max_step(silence, at - 10, at + hole + 10);
    const double c_step = max_step(concealed, at - 10, at + hole + 10);
    const double s_err = error_db(silence, at, at + hole);
    const double c_err = error_db(concealed, at, at + hole);
    printf("  %4.0f Hz: largest step %5.0f (silence) / %5.0f (concealed), "
           "tone's own %4.0f; error in the hole %+5.1f / %+5.1f dB\n",
           tone_hz, s_step, c_step, natural, s_err, c_err);
    CHECK(c_step < s_step / 2 || c_step <= 1.05 * natural,
          "concealed steps %f vs %f", c_step, s_step);
    CHECK(c_step < 3 * natural + 600, "a click: %f", c_step);
    CHECK(plc.events == 1, "events %u", plc.events);
    free(silence);
    free(concealed);
    audio_plc_deinit(&plc);
  }
  tone_hz = 440.0;
}

static void test_long_hole_fades(void) {
  printf("a 200 ms hole fades to silence within 50 ms\n");
  audio_plc_t plc;
  CHECK(audio_plc_init(&plc), "no memory");
  const size_t at = 10 * BLOCK, hole = 8820, total = at + hole + 4 * BLOCK;
  int16_t *out = play(&plc, total, at, hole, true);
  double peak_after = 0, peak_start = 0;
  for (size_t k = at; k < at + hole; k++) {
    for (int ch = 0; ch < CH; ch++) {
      const double v = fabs((double)out[k * CH + ch]);
      if (k >= at + AUDIO_PLC_FADE_FRAMES) {
        peak_after = fmax(peak_after, v);
      } else if (k < at + 200) {
        peak_start = fmax(peak_start, v);
      }
    }
  }
  CHECK(peak_after == 0, "%f after the fade", peak_after);
  CHECK(peak_start > AMP * 0.7, "start too quiet: %f", peak_start);
  // Coming back from silence: faded in, not switched on.
  const double resume = max_step(out, at + hole - 1, at + hole + 64);
  CHECK(resume < AMP / 8, "resume step %f", resume);
  CHECK(fabs((double)out[(at + hole + AUDIO_PLC_XFADE_FRAMES) * CH] -
             tone((uint32_t)(at + hole + AUDIO_PLC_XFADE_FRAMES), 0)) < 1,
        "real audio altered after the crossfade");
  free(out);
  audio_plc_deinit(&plc);
}

static void test_no_history(void) {
  printf("without history, or after a reset, a hole is silence\n");
  audio_plc_t plc;
  CHECK(audio_plc_init(&plc), "no memory");
  int16_t buf[BLOCK * CH];
  fill(buf, 0, 64);
  audio_plc_good(&plc, buf, 64, CH); // under AUDIO_PLC_MIN_HISTORY
  audio_plc_conceal(&plc, buf, BLOCK, CH);
  int nonzero = 0;
  for (size_t k = 0; k < BLOCK * CH; k++) {
    nonzero += buf[k] != 0;
  }
  CHECK(nonzero == 0, "%d samples from too little history", nonzero);

  fill(buf, 0, BLOCK);
  audio_plc_good(&plc, buf, BLOCK, CH);
  audio_plc_good(&plc, buf, BLOCK, CH);
  audio_plc_reset(&plc);
  audio_plc_conceal(&plc, buf, BLOCK, CH);
  nonzero = 0;
  for (size_t k = 0; k < BLOCK * CH; k++) {
    nonzero += buf[k] != 0;
  }
  CHECK(nonzero == 0, "%d samples after a reset", nonzero);
  audio_plc_deinit(&plc);

  // No memory at all: silence, and good audio is left alone.
  audio_plc_t none;
  memset(&none, 0, sizeof(none));
  fill(buf, 0, BLOCK);
  const int16_t first = buf[0];
  audio_plc_good(&none, buf, BLOCK, CH);
  CHECK(buf[0] == first, "good audio changed");
  audio_plc_conceal(&none, buf, BLOCK, CH);
  CHECK(buf[0] == 0 && buf[BLOCK * CH - 1] == 0, "not silent");
}

static void test_split_hole(void) {
  printf("a hole filled in pieces is the same as in one go\n");
  audio_plc_t a, b;
  CHECK(audio_plc_init(&a) && audio_plc_init(&b), "no memory");
  int16_t buf[4 * BLOCK * CH], one[1000 * CH], pieces[1000 * CH];
  fill(buf, 0, 4 * BLOCK);
  audio_plc_good(&a, buf, 4 * BLOCK, CH);
  audio_plc_good(&b, buf, 4 * BLOCK, CH);
  audio_plc_conceal(&a, one, 1000, CH);
  for (size_t at = 0; at < 1000; at += 37) {
    const size_t n = at + 37 > 1000 ? 1000 - at : 37;
    audio_plc_conceal(&b, &pieces[at * CH], n, CH);
  }
  CHECK(memcmp(one, pieces, sizeof(one)) == 0, "pieces differ");
  audio_plc_deinit(&a);
  audio_plc_deinit(&b);
}

int main(void) {
  test_one_lost_packet();
  test_long_hole_fades();
  test_no_history();
  test_split_hole();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}

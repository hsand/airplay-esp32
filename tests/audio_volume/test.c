// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for main/audio/audio_volume.c: Apple's volume law from a table,
 * the earlier curve kept as an option, and the 0.3 dB change gate.  The file
 * is included whole so the table itself can be checked. */

#include "audio_volume.c"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int failures;
#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      failures++;                                                              \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__);                            \
      printf(__VA_ARGS__);                                                     \
      printf("\n");                                                            \
    }                                                                          \
  } while (0)

static double apple_exact(double db) {
  return 32768.0 * pow(10.0, db / 20.0);
}

static void test_table(void) {
  printf("table: round(32768 * 10^(-i/200))\n");
  for (int i = 0; i <= 300; i++) {
    const long want = lround(apple_exact(-i / 10.0));
    CHECK(DB_TENTHS_Q15[i] == want, "entry %d is %u, wanted %ld", i,
          (unsigned)DB_TENTHS_Q15[i], want);
  }
}

/* Against 10^(dB/20) over the whole slider in 0.01 dB steps: within 0.01 dB
 * (or 1 LSB near the bottom) and never decreasing as the slider goes up. */
static void test_apple_law(void) {
  printf("Apple's law\n");
  double worst_db = 0.0;
  int32_t prev = -1;
  for (int c = -2999; c <= 0; c++) {
    const float db = (float)c / 100.0f;
    const int32_t q15 = audio_volume_apple_q15(db);
    const double exact = apple_exact(db);
    const double err_db = fabs(20.0 * log10((double)q15 / exact));
    if (fabs((double)q15 - exact) > 1.0 && err_db > worst_db) {
      worst_db = err_db;
    }
    CHECK(q15 >= prev, "falls from %d to %d at %.2f dB", (int)prev, (int)q15,
          db);
    prev = q15;
  }
  printf("  worst error %.4f dB\n", worst_db);
  CHECK(worst_db < 0.01, "worst error %.4f dB", worst_db);

  CHECK(audio_volume_apple_q15(0.0f) == AUDIO_VOLUME_UNITY_Q15, "0 dB");
  CHECK(audio_volume_apple_q15(6.0f) == AUDIO_VOLUME_UNITY_Q15, "above 0 dB");
  CHECK(audio_volume_apple_q15(INFINITY) == AUDIO_VOLUME_UNITY_Q15, "+inf");
  CHECK(abs(audio_volume_apple_q15(-6.0f) - 16423) <= 1, "-6 dB is %d",
        (int)audio_volume_apple_q15(-6.0f));
  CHECK(abs(audio_volume_apple_q15(-20.0f) - 3277) <= 1, "-20 dB is %d",
        (int)audio_volume_apple_q15(-20.0f));
  /* Apple's step at the bottom: -29.9 dB still plays at about -30 dB, the
   * bottom of the slider is silence, and so is the sender's -144. */
  CHECK(abs(audio_volume_apple_q15(-29.9f) - 1048) <= 1, "-29.9 dB is %d",
        (int)audio_volume_apple_q15(-29.9f));
  CHECK(audio_volume_apple_q15(-29.999f) > 1000, "just above -30 dB");
  CHECK(audio_volume_apple_q15(-30.0f) == 0, "-30 dB not silent");
  CHECK(audio_volume_apple_q15(-144.0f) == 0, "-144 dB not silent");
  CHECK(audio_volume_apple_q15(-INFINITY) == 0, "-inf not silent");
  CHECK(audio_volume_apple_q15(NAN) == 0, "NaN not silent");
}

/* The earlier curve, unchanged: ((dB + 30) / 30)^2. */
static void test_legacy_law(void) {
  printf("legacy law\n");
  for (int c = -2999; c < 0; c += 7) {
    const float db = (float)c / 100.0f;
    const float n = (db + 30.0f) / 30.0f;
    const int32_t want = (int32_t)(n * n * 32768.0f);
    CHECK(audio_volume_legacy_q15(db) == want, "%.2f dB", db);
  }
  CHECK(audio_volume_legacy_q15(-30.0f) == 0 &&
            audio_volume_legacy_q15(-144.0f) == 0 &&
            audio_volume_legacy_q15(NAN) == 0,
        "mute");
  CHECK(audio_volume_legacy_q15(0.0f) == AUDIO_VOLUME_UNITY_Q15, "unity");
#ifdef CONFIG_DAC_VOLUME_LAW_LEGACY
  CHECK(audio_volume_q15(-10.0f) == audio_volume_legacy_q15(-10.0f),
        "configured law");
#else
  CHECK(audio_volume_q15(-10.0f) == audio_volume_apple_q15(-10.0f),
        "configured law");
#endif
}

/* Apple drops a change under 1 % of its 30 dB slider. */
static void test_gate(void) {
  printf("0.3 dB gate\n");
  CHECK(!audio_volume_change_noticeable(-10.0f, -10.2f), "0.2 dB acted on");
  CHECK(!audio_volume_change_noticeable(-10.0f, -9.75f), "0.25 dB acted on");
  CHECK(audio_volume_change_noticeable(-10.0f, -10.3f), "0.3 dB dropped");
  CHECK(audio_volume_change_noticeable(-10.0f, -9.7f), "0.3 dB up dropped");
  CHECK(audio_volume_change_noticeable(-15.0f, -5.0f), "10 dB dropped");
  /* Into and out of mute always counts, however small. */
  CHECK(audio_volume_change_noticeable(-29.9f, -30.0f), "into mute dropped");
  CHECK(audio_volume_change_noticeable(-144.0f, -29.95f), "out of mute");
  CHECK(audio_volume_change_noticeable(-12.0f, -144.0f), "-144 dropped");
  /* -144 and -30 are the same slider position. */
  CHECK(!audio_volume_change_noticeable(-30.0f, -144.0f), "-144 vs -30");
  /* Above 0 dB is the top of the slider. */
  CHECK(!audio_volume_change_noticeable(0.0f, 3.0f), "above the top");
  CHECK(!audio_volume_change_noticeable(NAN, -144.0f), "NaN is mute");
}

int main(void) {
  test_table();
  test_apple_law();
  test_legacy_law();
  test_gate();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}

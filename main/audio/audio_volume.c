// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "audio_volume.h"

#include "sdkconfig.h"

#define VOLUME_MIN_DB (-30.0f)

/* round(32768 * 10^(-i / 200)) for i = 0..300: the gain at 0, -0.1, ...,
 * -30.0 dB in Q15.  Generated with
 *   [round(32768 * 10 ** (-i / 200)) for i in range(301)]
 * The last entry only anchors the interpolation just above -30 dB, which is
 * itself silent. */
static const uint16_t DB_TENTHS_Q15[301] = {
    32768, 32393, 32022, 31656, 31293, 30935, 30581, 30231, 29885, 29543, 29205,
    28870, 28540, 28213, 27890, 27571, 27255, 26943, 26635, 26330, 26029, 25731,
    25436, 25145, 24857, 24573, 24291, 24013, 23738, 23467, 23198, 22932, 22670,
    22410, 22154, 21900, 21650, 21402, 21157, 20915, 20675, 20439, 20205, 19973,
    19745, 19519, 19295, 19074, 18856, 18640, 18427, 18216, 18007, 17801, 17597,
    17396, 17197, 17000, 16805, 16613, 16423, 16235, 16049, 15865, 15684, 15504,
    15327, 15151, 14978, 14806, 14637, 14469, 14304, 14140, 13978, 13818, 13660,
    13504, 13349, 13196, 13045, 12896, 12748, 12602, 12458, 12315, 12174, 12035,
    11897, 11761, 11627, 11493, 11362, 11232, 11103, 10976, 10851, 10726, 10604,
    10482, 10362, 10244, 10126, 10010, 9896,  9783,  9671,  9560,  9450,  9342,
    9235,  9130,  9025,  8922,  8820,  8719,  8619,  8520,  8423,  8326,  8231,
    8137,  8044,  7952,  7860,  7771,  7682,  7594,  7507,  7421,  7336,  7252,
    7169,  7087,  7006,  6925,  6846,  6768,  6690,  6614,  6538,  6463,  6389,
    6316,  6244,  6172,  6102,  6032,  5963,  5895,  5827,  5760,  5694,  5629,
    5565,  5501,  5438,  5376,  5314,  5254,  5193,  5134,  5075,  5017,  4960,
    4903,  4847,  4791,  4736,  4682,  4629,  4576,  4523,  4471,  4420,  4370,
    4320,  4270,  4221,  4173,  4125,  4078,  4031,  3985,  3940,  3894,  3850,
    3806,  3762,  3719,  3677,  3635,  3593,  3552,  3511,  3471,  3431,  3392,
    3353,  3315,  3277,  3239,  3202,  3166,  3129,  3093,  3058,  3023,  2988,
    2954,  2920,  2887,  2854,  2821,  2789,  2757,  2726,  2694,  2663,  2633,
    2603,  2573,  2544,  2514,  2486,  2457,  2429,  2401,  2374,  2347,  2320,
    2293,  2267,  2241,  2215,  2190,  2165,  2140,  2116,  2091,  2068,  2044,
    2020,  1997,  1974,  1952,  1930,  1907,  1886,  1864,  1843,  1822,  1801,
    1780,  1760,  1740,  1720,  1700,  1681,  1661,  1642,  1623,  1605,  1587,
    1568,  1550,  1533,  1515,  1498,  1481,  1464,  1447,  1430,  1414,  1398,
    1382,  1366,  1350,  1335,  1320,  1305,  1290,  1275,  1260,  1246,  1232,
    1217,  1204,  1190,  1176,  1163,  1149,  1136,  1123,  1110,  1098,  1085,
    1073,  1060,  1048,  1036,
};

int32_t audio_volume_apple_q15(float db) {
  if (!(db > VOLUME_MIN_DB)) {
    return 0; /* -30 dB, -144 and NaN all mean mute */
  }
  if (db >= 0.0f) {
    return AUDIO_VOLUME_UNITY_Q15;
  }
  /* Tenths of a dB below 0 in Q8: at most 300 * 256. */
  const int32_t x = (int32_t)(-db * 2560.0f + 0.5f);
  const int32_t i = x >> 8;
  if (i >= 300) {
    return DB_TENTHS_Q15[300];
  }
  const int32_t frac = x & 0xFF;
  const int32_t a = DB_TENTHS_Q15[i];
  const int32_t b = DB_TENTHS_Q15[i + 1];
  return a + (((b - a) * frac + 128) >> 8);
}

int32_t audio_volume_legacy_q15(float db) {
  if (!(db > VOLUME_MIN_DB)) {
    return 0;
  }
  if (db >= 0.0f) {
    return AUDIO_VOLUME_UNITY_Q15;
  }
  const float normalized = (db - VOLUME_MIN_DB) / -VOLUME_MIN_DB;
  return (int32_t)(normalized * normalized * (float)AUDIO_VOLUME_UNITY_Q15);
}

int32_t audio_volume_q15(float db) {
#ifdef CONFIG_DAC_VOLUME_LAW_LEGACY
  return audio_volume_legacy_q15(db);
#else
  return audio_volume_apple_q15(db);
#endif
}

bool audio_volume_change_noticeable(float from_db, float to_db) {
  const bool from_mute = !(from_db > VOLUME_MIN_DB);
  const bool to_mute = !(to_db > VOLUME_MIN_DB);
  if (from_mute != to_mute) {
    return true;
  }
  if (from_mute) {
    return false;
  }
  /* Slider positions: nothing above 0 dB. */
  from_db = from_db > 0.0f ? 0.0f : from_db;
  to_db = to_db > 0.0f ? 0.0f : to_db;
  const float diff = to_db > from_db ? to_db - from_db : from_db - to_db;
  /* 0.01 of the 30 dB slider, with a hair of slack for the float rounding
   * of values a sender sends as 0.3 dB steps. */
  return diff >= 0.3f - 1e-4f;
}

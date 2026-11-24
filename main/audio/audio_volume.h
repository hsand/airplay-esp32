// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/**
 * AirPlay volume to software gain.
 *
 * Apple's receivers turn the sender's dB into a gain with
 * APSVolumeConvertDBToLinearGain: 10^(dB/20) above -30 dB, silence at or
 * below it (the sender's -144 "mute" included), never above unity.  That is
 * the default here (CONFIG_DAC_VOLUME_LAW_APPLE), so a speaker in a group
 * with HomePods plays at their level for the same slider position.  The
 * earlier squared-slider curve stays as CONFIG_DAC_VOLUME_LAW_LEGACY.
 *
 * The gain is read from a table in 0.1 dB steps with linear interpolation
 * between them: no pow(), which the ESP32-C5 would run in soft float.  It is
 * worked out once per volume change, never per sample.
 *
 * Host-testable: no platform dependencies.
 */

#include <stdbool.h>
#include <stdint.h>

/** Unity in Q15. */
#define AUDIO_VOLUME_UNITY_Q15 32768

/** The gain for @p db by the configured law, Q15 (0..32768). */
int32_t audio_volume_q15(float db);

/** Apple's law: 10^(db/20), 0 at or below -30 dB and for NaN. */
int32_t audio_volume_apple_q15(float db);

/** The earlier curve: ((db + 30) / 30)^2, 0 at or below -30 dB. */
int32_t audio_volume_legacy_q15(float db);

/**
 * Whether a volume change is worth acting on.  Apple drops a change of under
 * 0.01 of its slider (-30..0 dB), i.e. 0.3 dB, after clamping -144 to -30.
 * Here a change into or out of mute always counts, whatever its size: a
 * slider pulled from -29.9 dB to the bottom has to go quiet.
 */
bool audio_volume_change_noticeable(float from_db, float to_db);

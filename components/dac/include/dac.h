// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  DAC_POWER_ON = 0,
  DAC_POWER_STANDBY,
  DAC_POWER_OFF,
} dac_power_mode_t;

/**
 * DAC driver operations — each DAC driver provides one of these.
 * NULL function pointers are treated as no-ops.
 */
typedef struct {
  esp_err_t (*init)(void *i2c_bus);
  esp_err_t (*deinit)(void);
  void (*set_volume)(float volume_db);
  void (*set_power_mode)(dac_power_mode_t mode);
  void (*on_i2s_stopping)(void);
  void (*on_i2s_started)(uint32_t sample_rate_hz);
  void (*enable_speaker)(bool enable);
  void (*enable_line_out)(bool enable);
  /**
   * The DAC's own output delay at @p sample_rate_hz: its interpolation
   * filter's group delay, plus any resampling or DSP ahead of it, from the
   * I2S input to the analog output, in 1/16 sample periods (Q4).  Return
   * DAC_OUTPUT_DELAY_UNKNOWN when the configured mode has no published
   * figure.  NULL is the same as unknown, and lets the plain-DAC preset
   * (CONFIG_DAC_PLAIN_DELAY_*) stand in.  Called from the playback task on
   * every block, so it must read cached state only: no bus traffic, no
   * locks, no logging.
   */
  uint32_t (*output_delay_q4)(uint32_t sample_rate_hz);
} dac_ops_t;

/** output_delay_q4(): no figure for this DAC in its current mode. */
#define DAC_OUTPUT_DELAY_UNKNOWN UINT32_MAX

/** Where dac_get_output_delay_q4() took its figure from. */
typedef enum {
  DAC_DELAY_FROM_DRIVER = 0, /**< the registered driver reported it */
  DAC_DELAY_FROM_PRESET,     /**< the plain I2S DAC preset in Kconfig */
  DAC_DELAY_UNKNOWN,         /**< neither: taken as 0 */
} dac_delay_source_t;

/**
 * AirPlay's volume floor: at or below it the output is muted, as on Apple's
 * receivers, which also read the sender's -144 ("muted") as this.
 */
#define DAC_VOLUME_MIN_DB (-30.0f)

/**
 * Apple's volume law for a DAC with its own volume control: the level, in dB
 * relative to the driver's full-volume ceiling, for an AirPlay volume.  The
 * law is 1:1 in dB (gain = 10^(dB/20)), so a DAC sits where an Apple speaker
 * at the same slider position does.  Returns false for mute: at or below
 * DAC_VOLUME_MIN_DB, and for NaN.
 */
static inline bool dac_volume_apple_db(float airplay_db, float *level_db) {
  if (!(airplay_db > DAC_VOLUME_MIN_DB)) {
    return false;
  }
  *level_db = airplay_db > 0.0f ? 0.0f : airplay_db;
  return true;
}

/**
 * Register a DAC driver. Must be called before dac_init().
 */
void dac_register(const dac_ops_t *ops);

/**
 * Initialize the registered DAC driver.
 *
 * @param i2c_bus  I2C master bus handle (may be NULL if the driver
 *                 does not use an externally-provided bus)
 */
esp_err_t dac_init(void *i2c_bus);

/**
 * Deinitialize the DAC
 */
esp_err_t dac_deinit(void);

/**
 * Set the DAC output volume (AirPlay dB scale: -30 to 0)
 */
void dac_set_volume(float volume_db);

/**
 * Set the DAC/amplifier power mode.
 *
 * DAC_POWER_ON takes effect at once and cancels a pending quieting.  A
 * STANDBY or OFF that follows ON is deferred by DAC_QUIET_DELAY_MS, so the
 * receiver's fade-out still in the DMA ring is heard before the amplifier or
 * its mute line goes quiet (see dac.c).
 */
void dac_set_power_mode(dac_power_mode_t mode);

/**
 * How long a quieting power mode waits: the DMA ring (2048 frames, 46 ms at
 * 44.1 kHz and 64 ms at 32 kHz), the receiver's fade-out (8 ms) and the DAC's
 * filter, with room to spare.
 */
#define DAC_QUIET_DELAY_MS 80

/**
 * Notify the DAC driver that the I2S clocks are about to stop for a retune,
 * so it can quiet its outputs first. dac_on_i2s_started() follows once they
 * run again.
 */
void dac_on_i2s_stopping(void);

/**
 * Notify the DAC driver that the I2S master clock and bit clocks are active,
 * and at what rate they are running.
 */
void dac_on_i2s_started(uint32_t sample_rate_hz);

/**
 * The output delay of the DAC, in 1/16 sample periods at @p sample_rate_hz:
 * the registered driver's figure, else the plain I2S DAC preset from Kconfig
 * (a board with no driver, or one whose driver has no figure), else 0.
 * @p source, if not NULL, says which.  Cheap enough for the playback task.
 */
uint32_t dac_get_output_delay_q4(uint32_t sample_rate_hz,
                                 dac_delay_source_t *source);

/** A short name for the plain I2S DAC preset, for logs. */
const char *dac_output_delay_preset_name(void);

/** "driver", "preset" or "unknown". */
const char *dac_delay_source_name(dac_delay_source_t source);

/**
 * Enable or disable the speaker output
 */
void dac_enable_speaker(bool enable);

/**
 * Enable or disable the line output
 */
void dac_enable_line_out(bool enable);

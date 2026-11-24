// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * DAC dispatch layer
 *
 * Routes abstract DAC API calls to the registered DAC driver.
 * When no driver is registered, all functions are no-ops.
 */

#include <stdbool.h>
#include <stddef.h>
#include "dac.h"

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const dac_ops_t *s_ops = NULL;

static bool power_lock_ready(void);

void dac_register(const dac_ops_t *ops) {
  /* Boards register from app_main, before any playback event can race to
   * create the lock. */
  (void)power_lock_ready();
  s_ops = ops;
}

esp_err_t dac_init(void *i2c_bus) {
  return (s_ops && s_ops->init) ? s_ops->init(i2c_bus) : ESP_OK;
}

void dac_set_volume(float volume_db) {
  if (s_ops && s_ops->set_volume) {
    s_ops->set_volume(volume_db);
  }
}

/* ---- Deferred quieting ----
 *
 * A pause, a teardown and a disconnect reach the board as playback events,
 * and the RTSP task emits them *before* it stops the audio.  Applied at once,
 * they muted the amplifier (or dropped XSMT, or SD_MODE) while ~46 ms of
 * queued audio and the receiver's fade-out were still in the DMA ring: the
 * end of the music was cut off, and an amplifier without a soft mute popped.
 * So a STANDBY or OFF that follows ON waits DAC_QUIET_DELAY_MS on a small
 * task of its own, and an ON in the meantime cancels it.  Waking stays
 * immediate: it runs at SETRATEANCHORTIME, ahead of the first audible sample.
 * Only the quieting of a running DAC is deferred; the heavy transitions out
 * of OFF (a TAS57xx flow download, say) still run in the caller.  A driver's
 * set_power_mode() is serialised by s_pm_lock, whichever task calls it. */

static SemaphoreHandle_t s_pm_lock;
static TaskHandle_t s_pm_task;
static bool s_pm_task_failed;
static portMUX_TYPE s_pm_mux = portMUX_INITIALIZER_UNLOCKED;
/* Under s_pm_mux: the deferred mode, when it is due, and a generation that
 * every request bumps, so the task can tell its request was superseded. */
static bool s_pm_pending;
static dac_power_mode_t s_pm_pending_mode;
static TickType_t s_pm_due;
static uint32_t s_pm_generation;
/* Under s_pm_lock: what the driver was last asked for.  Starts as OFF, which
 * makes the first quieting after boot immediate. */
static dac_power_mode_t s_pm_applied = DAC_POWER_OFF;

static void apply_power_mode_locked(dac_power_mode_t mode) {
  s_ops->set_power_mode(mode);
  s_pm_applied = mode;
}

/* One pass of the deferral task: applies a quieting that has come due and
 * returns how long to sleep before the next look (portMAX_DELAY: until a
 * request notifies).  A request superseded while this waited for s_pm_lock
 * is dropped: the generation it read has moved on. */
static TickType_t power_service(void) {
  taskENTER_CRITICAL(&s_pm_mux);
  const bool pending = s_pm_pending;
  const uint32_t generation = s_pm_generation;
  const dac_power_mode_t mode = s_pm_pending_mode;
  const TickType_t wait = (TickType_t)(s_pm_due - xTaskGetTickCount());
  taskEXIT_CRITICAL(&s_pm_mux);
  if (!pending) {
    return portMAX_DELAY;
  }
  /* Wrapped subtraction: a due time already passed reads as huge. */
  if (wait != 0 && wait <= pdMS_TO_TICKS(DAC_QUIET_DELAY_MS)) {
    return wait;
  }
  xSemaphoreTake(s_pm_lock, portMAX_DELAY);
  taskENTER_CRITICAL(&s_pm_mux);
  const bool current = s_pm_pending && s_pm_generation == generation;
  if (current) {
    s_pm_pending = false;
  }
  taskEXIT_CRITICAL(&s_pm_mux);
  if (current) {
    apply_power_mode_locked(mode);
  }
  xSemaphoreGive(s_pm_lock);
  return 0;
}

static void power_task(void *arg) {
  (void)arg;
  for (;;) {
    /* A newer request notifies, and is looked at straight away. */
    (void)ulTaskNotifyTake(pdTRUE, power_service());
  }
}

static bool power_lock_ready(void) {
  if (!s_pm_lock) {
    s_pm_lock = xSemaphoreCreateMutex();
  }
  return s_pm_lock != NULL;
}

/* The deferral task, created on first need.  Without it (out of memory) every
 * mode applies at once, as before. */
static bool power_task_ready(void) {
  if (s_pm_task || s_pm_task_failed) {
    return s_pm_task != NULL;
  }
  if (xTaskCreate(power_task, "dac_pwr", 3072, NULL, 5, &s_pm_task) != pdPASS) {
    s_pm_task = NULL;
    s_pm_task_failed = true;
  }
  return s_pm_task != NULL;
}

void dac_set_power_mode(dac_power_mode_t mode) {
  if (!s_ops || !s_ops->set_power_mode || !power_lock_ready()) {
    return;
  }
  xSemaphoreTake(s_pm_lock, portMAX_DELAY);
  const bool quieting = mode != DAC_POWER_ON && s_pm_applied == DAC_POWER_ON;
  const bool defer = quieting && power_task_ready();
  taskENTER_CRITICAL(&s_pm_mux);
  const bool cancelled = s_pm_pending;
  s_pm_generation++;
  s_pm_pending = defer;
  if (defer) {
    s_pm_pending_mode = mode;
    s_pm_due = xTaskGetTickCount() + pdMS_TO_TICKS(DAC_QUIET_DELAY_MS);
  }
  taskEXIT_CRITICAL(&s_pm_mux);
  if (defer) {
    xSemaphoreGive(s_pm_lock);
    xTaskNotifyGive(s_pm_task);
    return;
  }
  /* A resume inside the delay: the DAC never left ON, and waking it again
   * would only mute it for its PLL wait. */
  if (!(cancelled && mode == DAC_POWER_ON && s_pm_applied == DAC_POWER_ON)) {
    apply_power_mode_locked(mode);
  }
  xSemaphoreGive(s_pm_lock);
}

esp_err_t dac_deinit(void) {
  /* A quieting still waiting is applied now: the driver is about to go. */
  if (s_ops && s_ops->set_power_mode && s_pm_lock) {
    xSemaphoreTake(s_pm_lock, portMAX_DELAY);
    taskENTER_CRITICAL(&s_pm_mux);
    const bool pending = s_pm_pending;
    const dac_power_mode_t mode = s_pm_pending_mode;
    s_pm_pending = false;
    s_pm_generation++;
    taskEXIT_CRITICAL(&s_pm_mux);
    if (pending) {
      apply_power_mode_locked(mode);
    }
    /* The driver starts again from OFF after an init. */
    s_pm_applied = DAC_POWER_OFF;
    xSemaphoreGive(s_pm_lock);
  }
  return (s_ops && s_ops->deinit) ? s_ops->deinit() : ESP_OK;
}

void dac_on_i2s_stopping(void) {
  if (s_ops && s_ops->on_i2s_stopping) {
    s_ops->on_i2s_stopping();
  }
}

void dac_on_i2s_started(uint32_t sample_rate_hz) {
  if (s_ops && s_ops->on_i2s_started) {
    s_ops->on_i2s_started(sample_rate_hz);
  }
}

void dac_enable_speaker(bool enable) {
  if (s_ops && s_ops->enable_speaker) {
    s_ops->enable_speaker(enable);
  }
}

void dac_enable_line_out(bool enable) {
  if (s_ops && s_ops->enable_line_out) {
    s_ops->enable_line_out(enable);
  }
}

/* ---- Output delay ----
 *
 * The plain I2S DAC preset, for a board whose DAC has no driver (or a driver
 * with no figure).  Every value is a datasheet group delay, in 1/16 input
 * sample periods; the sources are in components/dac/Kconfig. */
static uint32_t preset_delay_q4(uint32_t sample_rate_hz) {
#if defined(CONFIG_DAC_PLAIN_DELAY_PCM510X_NORMAL)
  /* TI SLAS859C §9.3.4.1 Tables 4-6 (normal x8/x4/x2): "Filter group delay
   * 22 tS".  §6 and §8.5 of the same revision say 20 tS. */
  (void)sample_rate_hz;
  return 22U * 16U;
#elif defined(CONFIG_DAC_PLAIN_DELAY_PCM510X_LOW_LATENCY)
  /* TI SLAS859C Tables 7-9 and §8.5: 3.5 tS. */
  (void)sample_rate_hz;
  return 56U;
#elif defined(CONFIG_DAC_PLAIN_DELAY_CS434X)
  /* Cirrus DS613F2 p.6, combined interpolation and on-chip analog filter:
   * 10/Fs single speed, 5/Fs double, 2.5/Fs quad.  The part picks the mode
   * from the MCLK/LRCK ratio, which the ESP32 sets so that it follows the
   * rate: up to 50 kHz single, up to 100 kHz double. */
  if (sample_rate_hz <= 50000U) {
    return 160U;
  }
  return sample_rate_hz <= 100000U ? 80U : 40U;
#elif defined(CONFIG_DAC_PLAIN_DELAY_CUSTOM)
  (void)sample_rate_hz;
  return (uint32_t)CONFIG_DAC_PLAIN_DELAY_CUSTOM_Q4;
#else
  (void)sample_rate_hz;
  return DAC_OUTPUT_DELAY_UNKNOWN;
#endif
}

const char *dac_output_delay_preset_name(void) {
#if defined(CONFIG_DAC_PLAIN_DELAY_PCM510X_NORMAL)
  return "PCM510xA normal filter";
#elif defined(CONFIG_DAC_PLAIN_DELAY_PCM510X_LOW_LATENCY)
  return "PCM510xA low-latency filter";
#elif defined(CONFIG_DAC_PLAIN_DELAY_CS434X)
  return "CS4344/5/8";
#elif defined(CONFIG_DAC_PLAIN_DELAY_CUSTOM)
  return "custom";
#else
  return "unknown";
#endif
}

uint32_t dac_get_output_delay_q4(uint32_t sample_rate_hz,
                                 dac_delay_source_t *source) {
  dac_delay_source_t from = DAC_DELAY_UNKNOWN;
  uint32_t q4 = DAC_OUTPUT_DELAY_UNKNOWN;
  if (s_ops && s_ops->output_delay_q4) {
    q4 = s_ops->output_delay_q4(sample_rate_hz);
    from = DAC_DELAY_FROM_DRIVER;
  }
  if (q4 == DAC_OUTPUT_DELAY_UNKNOWN) {
    q4 = preset_delay_q4(sample_rate_hz);
    from = DAC_DELAY_FROM_PRESET;
  }
  if (q4 == DAC_OUTPUT_DELAY_UNKNOWN) {
    q4 = 0;
    from = DAC_DELAY_UNKNOWN;
  }
  if (source) {
    *source = from;
  }
  return q4;
}

const char *dac_delay_source_name(dac_delay_source_t source) {
  switch (source) {
  case DAC_DELAY_FROM_DRIVER:
    return "driver";
  case DAC_DELAY_FROM_PRESET:
    return "preset";
  default:
    return "unknown";
  }
}

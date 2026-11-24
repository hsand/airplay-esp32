// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

// The DAC dispatch layer on the host: the deferred quieting of the amplifier
// after a stop, the output delay a driver or the plain-DAC preset reports,
// and the latency breakdown built on it.  dac.c is included so the test can
// run its deferral task's body, power_service(), at the ticks it chooses.

#include "mocks.h"

#include "../../components/dac/dac.c"
#include "../../main/audio/audio_latency.c"

#include <stdio.h>
#include <stdlib.h>

static int failures;
#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__);                            \
      printf(__VA_ARGS__);                                                     \
      printf("\n");                                                            \
      failures++;                                                              \
    }                                                                          \
  } while (0)

/* ---- FreeRTOS mocks ---- */

static int lock_token;
static bool lock_held;
static TickType_t ticks;
static int notifies;
static bool task_created;

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
  return &lock_token;
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t timeout) {
  CHECK(mutex == &lock_token && !lock_held && timeout == portMAX_DELAY,
        "lock taken twice");
  lock_held = true;
  return 1;
}

int xSemaphoreGive(SemaphoreHandle_t mutex) {
  CHECK(mutex == &lock_token && lock_held, "lock given while free");
  lock_held = false;
  return 1;
}

BaseType_t xTaskCreate(void (*fn)(void *), const char *name, uint32_t stack,
                       void *arg, int priority, TaskHandle_t *handle) {
  (void)fn;
  (void)name;
  (void)stack;
  (void)arg;
  (void)priority;
  task_created = true;
  *handle = &task_created;
  return pdPASS;
}

TickType_t xTaskGetTickCount(void) {
  return ticks;
}

uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait) {
  (void)clear;
  (void)wait;
  return 0;
}

void xTaskNotifyGive(TaskHandle_t task) {
  CHECK(task == &task_created, "notified an unknown task");
  notifies++;
}

/* ---- A driver that records what it is asked ---- */

static dac_power_mode_t applied[32];
static int applied_count;
static uint32_t driver_delay_q4 = DAC_OUTPUT_DELAY_UNKNOWN;

static void fake_set_power_mode(dac_power_mode_t mode) {
  CHECK(lock_held, "driver called outside the power lock");
  if (applied_count < 32) {
    applied[applied_count] = mode;
  }
  applied_count++;
}

static uint32_t fake_output_delay_q4(uint32_t sample_rate_hz) {
  (void)sample_rate_hz;
  return driver_delay_q4;
}

static const dac_ops_t fake_ops = {
    .set_power_mode = fake_set_power_mode,
    .output_delay_q4 = fake_output_delay_q4,
};

static const dac_ops_t plain_ops = {
    .set_power_mode = fake_set_power_mode,
};

static dac_power_mode_t last_applied(void) {
  return applied_count > 0 ? applied[applied_count - 1] : (dac_power_mode_t)-1;
}

/* Run the deferral task for `ms` of ticks, a tick at a time, as the real one
 * would wake. */
static void run_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    ticks++;
    (void)power_service();
  }
}

static void test_deferral(void) {
  printf("deferred quieting\n");
  dac_register(&plain_ops);
  ticks = UINT32_MAX - 30U; /* every wait below crosses the tick wrap */

  dac_set_power_mode(DAC_POWER_ON);
  CHECK(applied_count == 1 && last_applied() == DAC_POWER_ON,
        "waking is immediate");

  /* A pause: the amplifier stays on while the ring drains. */
  dac_set_power_mode(DAC_POWER_STANDBY);
  CHECK(applied_count == 1, "standby applied at once");
  CHECK(task_created && notifies == 1, "task not told");
  CHECK(power_service() == pdMS_TO_TICKS(DAC_QUIET_DELAY_MS),
        "task would sleep %u ticks", (unsigned)power_service());
  run_ms(DAC_QUIET_DELAY_MS - 1U);
  CHECK(applied_count == 1, "standby %u ms early", 1U);
  run_ms(1);
  CHECK(applied_count == 2 && last_applied() == DAC_POWER_STANDBY,
        "standby not applied when due");
  CHECK(power_service() == portMAX_DELAY, "task still busy");

  /* Already quiet: a further quieting goes straight through. */
  dac_set_power_mode(DAC_POWER_OFF);
  CHECK(applied_count == 3 && last_applied() == DAC_POWER_OFF,
        "quieting a quiet DAC was deferred");

  /* A resume inside the delay: the DAC never leaves ON, and is not woken
   * again (that would mute it for its PLL wait). */
  dac_set_power_mode(DAC_POWER_ON);
  const int before = applied_count;
  dac_set_power_mode(DAC_POWER_STANDBY);
  run_ms(40);
  dac_set_power_mode(DAC_POWER_ON);
  run_ms(2U * DAC_QUIET_DELAY_MS);
  CHECK(applied_count == before, "%d calls for a cancelled pause",
        applied_count - before);

  /* A pause then a disconnect: one quieting, the later one, timed from it. */
  dac_set_power_mode(DAC_POWER_STANDBY);
  run_ms(50);
  dac_set_power_mode(DAC_POWER_OFF);
  run_ms(DAC_QUIET_DELAY_MS - 1U);
  CHECK(applied_count == before, "quieting before the later request was due");
  run_ms(1);
  CHECK(applied_count == before + 1 && last_applied() == DAC_POWER_OFF,
        "disconnect not applied");

  /* Deinit applies a quieting still waiting, at once. */
  dac_set_power_mode(DAC_POWER_ON);
  dac_set_power_mode(DAC_POWER_STANDBY);
  const int pending_at = applied_count;
  (void)dac_deinit();
  CHECK(applied_count == pending_at + 1 && last_applied() == DAC_POWER_STANDBY,
        "deinit dropped the pending quieting");
  run_ms(2U * DAC_QUIET_DELAY_MS);
  CHECK(applied_count == pending_at + 1, "applied twice");
  /* After an init the driver starts from OFF: quieting is immediate. */
  dac_set_power_mode(DAC_POWER_STANDBY);
  CHECK(applied_count == pending_at + 2, "quieting after deinit deferred");
  CHECK(!lock_held, "lock left held");
}

/* What each build's preset gives, at a 44.1 kHz and a 96 kHz output. */
static void test_delay_sources(void) {
  printf("output delay: driver, preset (%s), unknown\n",
         dac_output_delay_preset_name());
  dac_delay_source_t source;
#if defined(CONFIG_DAC_PLAIN_DELAY_PCM510X_NORMAL)
  const uint32_t want44 = 352U, want96 = 352U;
#elif defined(CONFIG_DAC_PLAIN_DELAY_PCM510X_LOW_LATENCY)
  const uint32_t want44 = 56U, want96 = 56U;
#elif defined(CONFIG_DAC_PLAIN_DELAY_CS434X)
  const uint32_t want44 = 160U, want96 = 80U;
#elif defined(CONFIG_DAC_PLAIN_DELAY_CUSTOM)
  const uint32_t want44 = CONFIG_DAC_PLAIN_DELAY_CUSTOM_Q4,
                 want96 = CONFIG_DAC_PLAIN_DELAY_CUSTOM_Q4;
#else
  const uint32_t want44 = 0U, want96 = 0U;
#endif
#if defined(CONFIG_DAC_PLAIN_DELAY_UNKNOWN)
  const dac_delay_source_t preset = DAC_DELAY_UNKNOWN;
#else
  const dac_delay_source_t preset = DAC_DELAY_FROM_PRESET;
#endif

  dac_register(&plain_ops);
  uint32_t q4 = dac_get_output_delay_q4(44100, &source);
  CHECK(q4 == want44 && source == preset, "no driver: %u from %s",
        (unsigned)q4, dac_delay_source_name(source));
  q4 = dac_get_output_delay_q4(96000, &source);
  CHECK(q4 == want96 && source == preset, "no driver, 96 kHz: %u",
        (unsigned)q4);

  /* A driver with no figure for its mode: the preset stands in. */
  dac_register(&fake_ops);
  driver_delay_q4 = DAC_OUTPUT_DELAY_UNKNOWN;
  q4 = dac_get_output_delay_q4(44100, &source);
  CHECK(q4 == want44 && source == preset, "driver unknown: %u from %s",
        (unsigned)q4, dac_delay_source_name(source));

  /* A driver figure always wins, 0 included. */
  driver_delay_q4 = 0;
  q4 = dac_get_output_delay_q4(44100, &source);
  CHECK(q4 == 0 && source == DAC_DELAY_FROM_DRIVER, "driver 0: %u from %s",
        (unsigned)q4, dac_delay_source_name(source));
  driver_delay_q4 = 539;
  q4 = dac_get_output_delay_q4(44100, NULL);
  CHECK(q4 == 539, "driver 33.7 samples: %u", (unsigned)q4);
}

/* The breakdown: each part in microseconds, rounded, and the trim. */
static void test_breakdown(void) {
  printf("latency breakdown\n");
  dac_register(&fake_ops);
  driver_delay_q4 = 22U * 16U; /* 22 samples */
  audio_latency_t l;
  (void)audio_latency_set_trim_us(0);
  audio_latency_compute(44100, 32, 726, 0, &l);
  CHECK(l.fifo_us == 726, "FIFO %u us", (unsigned)l.fifo_us); /* 32/44.1k */
  CHECK(l.dac_us == 499, "DAC %u us", (unsigned)l.dac_us);    /* 22/44.1k */
  CHECK(l.dac_source == DAC_DELAY_FROM_DRIVER, "source");
  CHECK(l.post_ring_us == 726 + 726 + 499, "sum %d", (int)l.post_ring_us);

  audio_latency_compute(48000, 32, 0, 1000, &l);
  CHECK(l.dac_us == 458 && l.fifo_us == 667, "48 kHz: DAC %u FIFO %u",
        (unsigned)l.dac_us, (unsigned)l.fifo_us);
  CHECK(l.post_ring_us == 667 + 1000 + 458, "sum %d", (int)l.post_ring_us);

  CHECK(audio_latency_set_trim_us(-3500) == -3500, "trim");
  audio_latency_compute(48000, 32, 0, 0, &l);
  CHECK(l.trim_us == -3500 && l.post_ring_us == 667 + 458 - 3500,
        "trimmed sum %d", (int)l.post_ring_us);
  CHECK(audio_latency_set_trim_us(80000) == AUDIO_LATENCY_TRIM_MAX_US &&
            audio_latency_set_trim_us(-80000) == -AUDIO_LATENCY_TRIM_MAX_US,
        "trim not clamped");
  audio_latency_compute(0, 32, 0, 0, &l);
  CHECK(l.fifo_us == 0 && l.dac_us == 0, "rate 0 divides");
  (void)audio_latency_set_trim_us(0);
}

int main(void) {
  test_deferral();
  test_delay_sources();
  test_breakdown();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* The platform under the playout simulation: ESP-IDF, FreeRTOS and the clock
 * are all whatever sim.c says they are.  Every shim header run.py generates
 * (esp_timer.h, freertos/semphr.h, ...) includes only this file. */

#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef int esp_err_t;
#define ESP_OK                0
#define ESP_FAIL              (-1)
#define ESP_ERR_NO_MEM        0x101
#define ESP_ERR_INVALID_ARG   0x102
#define ESP_ERR_INVALID_STATE 0x103
static inline const char *esp_err_to_name(esp_err_t err) {
  (void)err;
  return "esp_err";
}

/* Local time: the ESP32's crystal, which also clocks the DAC. */
extern int64_t sim_now_us;
#define esp_timer_get_time() (sim_now_us)

void sim_log(char level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
#define ESP_LOGE(tag, ...) sim_log('E', tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) sim_log('W', tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) sim_log('I', tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) sim_log('D', tag, __VA_ARGS__)
#define ESP_LOGV(tag, ...) sim_log('V', tag, __VA_ARGS__)

/* One task at a time: every lock is free. */
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m)        ((void)(m))
#define portEXIT_CRITICAL(m)         ((void)(m))
#define taskENTER_CRITICAL(m)        ((void)(m))
#define taskEXIT_CRITICAL(m)         ((void)(m))

typedef void *SemaphoreHandle_t;
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdTRUE            1
#define pdFALSE           0
#define pdPASS            1
#define portMAX_DELAY     0xFFFFFFFFU
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
static inline SemaphoreHandle_t sim_semaphore(void) {
  static int token;
  return &token;
}
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) {
  (void)s;
  (void)t;
  return pdTRUE;
}
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) {
  (void)s;
  return pdTRUE;
}
static inline void vSemaphoreDelete(SemaphoreHandle_t s) {
  (void)s;
}
static inline void vTaskDelay(TickType_t t) {
  (void)t;
}
#define xSemaphoreCreateBinary() sim_semaphore()
#define xSemaphoreCreateMutex()  sim_semaphore()

#define MALLOC_CAP_SPIRAM            0
#define MALLOC_CAP_8BIT              0
#define MALLOC_CAP_INTERNAL          0
#define heap_caps_malloc(n, caps)    malloc(n)
#define heap_caps_calloc(n, s, caps) calloc(n, s)


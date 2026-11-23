// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

// Platform stand-ins for the PTP network simulator.  Time is the simulated
// board's local clock (it drifts against the truth); there is one thread.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>

typedef int esp_err_t;
#define ESP_OK                0
#define ESP_FAIL              -1
#define ESP_ERR_INVALID_STATE 0x103

void sim_log(const char *tag, const char *format, ...);
#define ESP_LOGE sim_log
#define ESP_LOGW sim_log
#define ESP_LOGI sim_log
#define ESP_LOGD sim_log

extern int64_t sim_local_us;
#define esp_timer_get_time() (sim_local_us)

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux)      ((void)(mux))
#define portEXIT_CRITICAL(mux)       ((void)(mux))

typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
#define pdPASS              1
#define portTICK_PERIOD_MS  1
#define pdMS_TO_TICKS(ms)   (ms)
#define xTaskGetTickCount() ((TickType_t)(sim_local_us / 1000))
#define vTaskDelay(ticks)   ((void)(ticks))
#define vTaskDelete(handle) ((void)(handle))

typedef struct {
  void *stack;
} spiram_task_mem_t;
static inline BaseType_t task_create_spiram(TaskFunction_t fn, const char *name,
                                            uint32_t depth, void *param,
                                            int prio, TaskHandle_t *handle,
                                            spiram_task_mem_t *mem) {
  (void)fn, (void)name, (void)depth, (void)param, (void)prio, (void)handle;
  (void)mem;
  return 0;
}
static inline void task_free_spiram(spiram_task_mem_t *mem) {
  (void)mem;
}

#define ESP_MAC_WIFI_STA 0
static inline esp_err_t esp_read_mac(uint8_t *mac, int type) {
  (void)type;
  static const uint8_t m[6] = {0x24, 0x0a, 0xc4, 0x12, 0x34, 0x56};
  for (int i = 0; i < 6; i++) {
    mac[i] = m[i];
  }
  return ESP_OK;
}

// The board's Delay_Reqs go to the simulated network.
ssize_t sim_sendto(int fd, const void *buf, size_t len, int flags,
                   const struct sockaddr *dst, socklen_t dst_len);
#define sendto sim_sendto

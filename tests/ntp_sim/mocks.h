// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Platform stand-ins for the NTP clock simulation.  Every blocking call the
 * clock makes (select, recvfrom with a timeout, vTaskDelay) advances the
 * simulated time instead, and every socket call talks to a simulated sender
 * across a simulated Wi-Fi link.  sim.c includes the system headers before
 * this one, so the macros below only rename the calls in ntp_clock.c. */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/select.h>
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

int64_t sim_local_us(void);
#define esp_timer_get_time() sim_local_us()
uint32_t sim_random(void);
#define esp_random() sim_random()

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux)      ((void)(mux))
#define portEXIT_CRITICAL(mux)       ((void)(mux))

typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
#define pdPASS             1
#define pdMS_TO_TICKS(ms)  (ms)
#define portTICK_PERIOD_MS 1
BaseType_t sim_task_create(TaskFunction_t fn, const char *name, uint32_t depth,
                           void *param, int prio, TaskHandle_t *handle);
#define xTaskCreate(fn, name, depth, param, prio, handle) \
  sim_task_create(fn, name, depth, param, prio, handle)
#define vTaskDelete(handle) ((void)(handle))
void sim_task_delay(uint32_t ms);
#define vTaskDelay(ticks) sim_task_delay(ticks)
TickType_t sim_tick_count(void);
#define xTaskGetTickCount() sim_tick_count()

int sim_socket(int domain, int type, int protocol);
int sim_setsockopt(int fd, int level, int name, const void *value,
                   socklen_t len);
int sim_connect(int fd, const struct sockaddr *addr, socklen_t len);
ssize_t sim_sendto(int fd, const void *buf, size_t len, int flags,
                   const struct sockaddr *to, socklen_t tolen);
ssize_t sim_send(int fd, const void *buf, size_t len, int flags);
ssize_t sim_recvfrom(int fd, void *buf, size_t len, int flags,
                     struct sockaddr *from, socklen_t *fromlen);
ssize_t sim_recv(int fd, void *buf, size_t len, int flags);
int sim_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
               struct timeval *timeout);
int sim_close(int fd);
#define socket     sim_socket
#define setsockopt sim_setsockopt
#define connect    sim_connect
#define sendto     sim_sendto
#define send       sim_send
#define recvfrom   sim_recvfrom
#define recv       sim_recv
#define select     sim_select
#define close      sim_close

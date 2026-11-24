// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

// Just enough FreeRTOS for components/dac/dac.c. The deferral task is
// created but never runs: the test calls its body, power_service(), itself,
// with the tick count under its control.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef int esp_err_t;
#define ESP_OK 0

typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portMAX_DELAY                UINT32_MAX
#define pdPASS                       1
#define pdTRUE                       1
#define pdMS_TO_TICKS(ms)            ((TickType_t)(ms))
#define taskENTER_CRITICAL(mux)      ((void)(mux))
#define taskEXIT_CRITICAL(mux)       ((void)(mux))

SemaphoreHandle_t xSemaphoreCreateMutex(void);
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t timeout);
int xSemaphoreGive(SemaphoreHandle_t mutex);
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, uint32_t stack,
                       void *arg, int priority, TaskHandle_t *handle);
TickType_t xTaskGetTickCount(void);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait);
void xTaskNotifyGive(TaskHandle_t task);

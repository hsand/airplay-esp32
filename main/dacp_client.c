// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * DACP client — sends control commands to AirPlay source device.
 *
 * Used for AirPlay 1 and for AirPlay 2 senders older than 354.54.6 (newer
 * ones get MediaRemote commands on the event channel instead and never send
 * DACP headers).  The sender's DACP service is resolved by its exact mDNS
 * instance name, iTunes_Ctrl_<DACP-ID>.
 * Commands are fire-and-forget HTTP GETs dispatched on a dedicated
 * worker task so callers (button task) never block on network I/O.
 *
 * Thread safety: dacp_set_session/clear may be called from the RTSP
 * server task while dacp_send_* are called from the button task.
 * A mutex protects the session state.
 */

#include "dacp_client.h"

#include "esp_http_client.h"
#include "esp_log.h"
#include "mdns.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static const char *TAG = "dacp";

#define DACP_ID_MAX       32
#define ACTIVE_REMOTE_MAX 32
// Apple's receiver resolves the sender's DACP service, and connects to it,
// with a 5 s timeout each.  A phone that has just woken its Wi-Fi can take
// most of that to answer.
#define RESOLVE_TIMEOUT_MS 5000
#define PROBE_TIMEOUT_MS   2000
#define CMD_PATH_MAX       80
#define CMD_QUEUE_LEN      4
#define HTTP_TIMEOUT_MS    5000
#define WORKER_STACK       4096

static SemaphoreHandle_t s_mutex;
static bool s_initialized;
static char s_dacp_id[DACP_ID_MAX];
static char s_active_remote[ACTIVE_REMOTE_MAX];
static uint32_t s_client_ip;
// From the last successful resolve; 0 means resolve before the next command.
// Never a "discovery failed" latch: every command that finds it 0 tries
// again, and any HTTP or connect failure clears it.
static uint16_t s_dacp_port;
static bool s_session_valid;
static QueueHandle_t s_cmd_queue;
static TaskHandle_t s_worker_handle;

// Special sentinel posted to the command queue to resolve ahead of the first
// button press, on the worker task (no extra task per session).
static const char CMD_DISCOVER[] = "\x01";

// The service instance a sender advertises: "iTunes_Ctrl_" and the DACP-ID as
// 16 upper-case hex digits, which is how Apple's receiver names it (the
// header is parsed as a 64-bit hex number and printed with %016llX).
static bool instance_name(const char *dacp_id, char *out, size_t size) {
  char *end = NULL;
  const unsigned long long id = strtoull(dacp_id, &end, 16);
  if (!end || end == dacp_id || *end != '\0') {
    return false;
  }
  snprintf(out, size, "iTunes_Ctrl_%016llX", id);
  return true;
}

// Resolve the session's DACP service to a port: the exact instance by SRV
// query, as Apple does, then a browse matched on the full instance name for a
// responder that ignores direct queries.  Without the mutex: it blocks.
static uint16_t resolve_port(const char *dacp_id, uint32_t timeout_ms) {
  char instance[48];
  if (!instance_name(dacp_id, instance, sizeof(instance))) {
    ESP_LOGW(TAG, "DACP-ID '%s' is not a hex number", dacp_id);
    return 0;
  }
  uint16_t port = 0;
  mdns_result_t *results = NULL;
  if (mdns_query_srv(instance, "_dacp", "_tcp", timeout_ms, &results) ==
          ESP_OK &&
      results) {
    port = results->port;
    mdns_query_results_free(results);
    results = NULL;
  }
  if (port == 0 && mdns_query_ptr("_dacp", "_tcp", PROBE_TIMEOUT_MS, 8,
                                  &results) == ESP_OK) {
    for (mdns_result_t *r = results; r && port == 0; r = r->next) {
      if (r->instance_name && r->port != 0 &&
          strcasecmp(r->instance_name, instance) == 0) {
        port = r->port;
      }
    }
  }
  if (results) {
    mdns_query_results_free(results);
  }
  if (port != 0) {
    ESP_LOGI(TAG, "Resolved %s._dacp._tcp: port %u", instance, port);
  } else {
    ESP_LOGW(TAG, "%s._dacp._tcp did not resolve", instance);
  }
  return port;
}

// Resolve if the port is not known.  Worker task only.
static uint16_t ensure_port(void) {
  char dacp_id_local[DACP_ID_MAX];
  xSemaphoreTake(s_mutex, portMAX_DELAY);
  uint16_t port = s_dacp_port;
  strlcpy(dacp_id_local, s_dacp_id, sizeof(dacp_id_local));
  const bool valid = s_session_valid;
  xSemaphoreGive(s_mutex);
  if (port != 0 || !valid || dacp_id_local[0] == '\0') {
    return port;
  }

  port = resolve_port(dacp_id_local, RESOLVE_TIMEOUT_MS);

  xSemaphoreTake(s_mutex, portMAX_DELAY);
  // Only if the session is still the one resolved for.
  if (s_session_valid && strcmp(s_dacp_id, dacp_id_local) == 0) {
    s_dacp_port = port;
  } else {
    port = 0;
  }
  xSemaphoreGive(s_mutex);
  return port;
}

// Execute a DACP HTTP request synchronously. Only called from the worker task.
static void execute_dacp_request(const char *path) {
  // Waits for the resolve rather than dropping the command, as Apple's
  // receiver does: the press that finds no port is the one that resolves.
  const uint16_t port_copy = ensure_port();

  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGW(TAG, "DACP mutex timeout for '%s'", path);
    return;
  }
  if (!s_session_valid || s_active_remote[0] == '\0') {
    xSemaphoreGive(s_mutex);
    ESP_LOGD(TAG, "DACP: no active session, skipping '%s'", path);
    return;
  }
  // Copy session state under the lock, then release before any I/O
  char active_remote_copy[ACTIVE_REMOTE_MAX];
  uint32_t client_ip_copy = s_client_ip;
  strlcpy(active_remote_copy, s_active_remote, sizeof(active_remote_copy));
  xSemaphoreGive(s_mutex);

  if (port_copy == 0) {
    ESP_LOGW(TAG, "DACP: sender's remote not found, '%s' not sent", path);
    return;
  }

  // Build URL: http://<ip>:<port>/ctrl-int/1/<path>
  char url[128];
  uint8_t *ip = (uint8_t *)&client_ip_copy;
  snprintf(url, sizeof(url), "http://%u.%u.%u.%u:%u/ctrl-int/1/%s", ip[0],
           ip[1], ip[2], ip[3], port_copy, path);

  esp_http_client_config_t config = {
      .url = url,
      .timeout_ms = HTTP_TIMEOUT_MS,
      .disable_auto_redirect = true,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (!client) {
    ESP_LOGW(TAG, "Failed to init HTTP client");
    return;
  }

  esp_http_client_set_header(client, "Active-Remote", active_remote_copy);
  esp_http_client_set_method(client, HTTP_METHOD_GET);

  esp_err_t err = esp_http_client_perform(client);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "DACP request failed: %s (%s)", path, esp_err_to_name(err));
    // The sender's remote moved or went away: resolve again next time.
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_dacp_port = 0;
    xSemaphoreGive(s_mutex);
  } else {
    int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "DACP: %s -> %d", path, status);
    if (status == 403 || status == 401) {
      ESP_LOGW(TAG, "DACP auth rejected — Active-Remote may be wrong");
    }
  }

  esp_http_client_cleanup(client);
}

// Worker task: drains the command queue and executes HTTP requests.
// A sentinel command (CMD_DISCOVER) resolves the port ahead of time.
static void dacp_worker_task(void *pvParameters) {
  (void)pvParameters;
  char path[CMD_PATH_MAX];
  while (1) {
    if (xQueueReceive(s_cmd_queue, path, portMAX_DELAY) == pdTRUE) {
      if (path[0] == CMD_DISCOVER[0]) {
        (void)ensure_port();
      } else {
        execute_dacp_request(path);
      }
    }
  }
}

// Enqueue a command for the worker task. Returns immediately (non-blocking).
static void send_dacp_request(const char *path) {
  if (!s_cmd_queue) {
    return;
  }
  char buf[CMD_PATH_MAX];
  strlcpy(buf, path, sizeof(buf));
  // Non-blocking: drop if queue is full (better than blocking caller)
  if (xQueueSend(s_cmd_queue, buf, 0) != pdTRUE) {
    ESP_LOGW(TAG, "DACP queue full, dropping '%s'", path);
  }
}

// ============================================================================
// Public API
// ============================================================================

void dacp_init(void) {
  if (s_initialized) {
    return;
  }
  s_mutex = xSemaphoreCreateMutex();
  s_cmd_queue = xQueueCreate(CMD_QUEUE_LEN, CMD_PATH_MAX);
  xTaskCreate(dacp_worker_task, "dacp_wk", WORKER_STACK, NULL, 4,
              &s_worker_handle);
  s_initialized = true;
  ESP_LOGI(TAG, "DACP client initialized");
}

void dacp_set_session(const char *dacp_id, const char *active_remote,
                      uint32_t client_ip) {
  xSemaphoreTake(s_mutex, portMAX_DELAY);

  const char *id = dacp_id ? dacp_id : "";
  const char *remote = active_remote ? active_remote : "";

  // If same DACP-ID, just update the active-remote token and IP in place.
  // No need to re-discover — the port doesn't change within a session.
  bool same_session = (strcmp(s_dacp_id, id) == 0 && s_session_valid);
  if (same_session) {
    strlcpy(s_active_remote, remote, sizeof(s_active_remote));
    s_client_ip = client_ip;
    xSemaphoreGive(s_mutex);
    return;
  }

  // New session — reset everything
  strlcpy(s_dacp_id, id, sizeof(s_dacp_id));
  strlcpy(s_active_remote, remote, sizeof(s_active_remote));
  s_client_ip = client_ip;
  s_dacp_port = 0;
  s_session_valid = (s_dacp_id[0] != '\0' && s_active_remote[0] != '\0');
  bool valid = s_session_valid;

  xSemaphoreGive(s_mutex);

  if (valid) {
    ESP_LOGI(TAG, "DACP session: id=%s", id);
    // Kick off discovery on the worker task so the port is ready before
    // the user presses a button.  Posting to front so it runs before any
    // queued commands; non-blocking drop is fine if queue is full.
    char sentinel[CMD_PATH_MAX] = {CMD_DISCOVER[0]};
    xQueueSendToFront(s_cmd_queue, sentinel, 0);
  }
}

void dacp_clear_session(void) {
  if (!s_initialized) {
    return;
  }

  xSemaphoreTake(s_mutex, portMAX_DELAY);
  s_dacp_id[0] = '\0';
  s_active_remote[0] = '\0';
  s_dacp_port = 0;
  s_session_valid = false;
  xSemaphoreGive(s_mutex);

  ESP_LOGD(TAG, "DACP session cleared");
}

void dacp_send_command(const char *command) {
  if (command && command[0]) {
    send_dacp_request(command);
  }
}

void dacp_send_playpause(void) {
  send_dacp_request("playpause");
}

void dacp_send_next(void) {
  send_dacp_request("nextitem");
}

void dacp_send_prev(void) {
  send_dacp_request("previtem");
}

void dacp_send_volume_up(void) {
  send_dacp_request("volumeup");
}

void dacp_send_volume_down(void) {
  send_dacp_request("volumedown");
}

void dacp_send_volume(float volume_percent) {
  if (volume_percent < 0.0f) {
    volume_percent = 0.0f;
  }
  if (volume_percent > 100.0f) {
    volume_percent = 100.0f;
  }

  char path[64];
  snprintf(path, sizeof(path), "setproperty?dmcp.volume=%.0f", volume_percent);
  send_dacp_request(path);
}

bool dacp_is_active(void) {
  if (!s_initialized) {
    return false;
  }
  bool active = false;
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    active = s_session_valid;
    xSemaphoreGive(s_mutex);
  }
  return active;
}

bool dacp_probe_service(void) {
  if (!s_initialized) {
    return false;
  }

  char dacp_id_copy[DACP_ID_MAX];
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  if (s_dacp_id[0] == '\0') {
    xSemaphoreGive(s_mutex);
    return false;
  }
  strlcpy(dacp_id_copy, s_dacp_id, sizeof(dacp_id_copy));
  xSemaphoreGive(s_mutex);

  return resolve_port(dacp_id_copy, PROBE_TIMEOUT_MS) != 0;
}

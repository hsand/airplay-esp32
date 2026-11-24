// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtsp_events.h"

#include <errno.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sodium.h"

#include "hap_internal.h"
#include "playback_events.h"
#include "rtsp_crypto.h"

static const char *TAG = "rtsp_events";

// The task logs replies, so it needs room for a vprintf.
#define EVENT_STACK_SIZE 4096
// A command that cannot go out within this long is dropped rather than
// holding up the button or web task that sent it.
#define EVENT_SEND_TIMEOUT_S 1

static int event_client_socket = -1;
static int event_listen_socket = -1;
static TaskHandle_t event_task_handle = NULL;
static volatile bool event_task_should_stop = false;

// Everything below is shared between the event task and the senders of
// commands, under s_lock.
static SemaphoreHandle_t s_lock;
static uint8_t s_ikm[HAP_IKM_MAX];
static size_t s_ikm_len;
static bool s_encrypted;
static uint8_t s_write_key[32];
static uint8_t s_read_key[32];
static uint64_t s_write_nonce;
static uint64_t s_read_nonce; // event task only
static uint32_t s_cseq;

static void lock(void) {
  xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock(void) {
  xSemaphoreGive(s_lock);
}

void rtsp_events_set_keys(const uint8_t *ikm, size_t ikm_len) {
  if (!s_lock) {
    s_lock = xSemaphoreCreateMutex();
  }
  lock();
  s_ikm_len = (ikm && ikm_len <= sizeof(s_ikm)) ? ikm_len : 0;
  if (s_ikm_len > 0) {
    memcpy(s_ikm, ikm, s_ikm_len);
  }
  unlock();
}

bool rtsp_events_connected(void) {
  return event_client_socket >= 0;
}

// A new event connection: its own keys and nonce counters from zero.
static void attach_client(int client) {
  lock();
  s_encrypted = s_ikm_len > 0;
  if (s_encrypted) {
    hap_derive_events_keys(s_ikm, s_ikm_len, s_write_key, s_read_key);
  }
  s_write_nonce = 0;
  s_read_nonce = 0;
  s_cseq = 0;
  event_client_socket = client;
  unlock();
}

static void detach_client(void) {
  lock();
  if (event_client_socket >= 0) {
    close(event_client_socket);
    event_client_socket = -1;
  }
  sodium_memzero(s_write_key, sizeof(s_write_key));
  sodium_memzero(s_read_key, sizeof(s_read_key));
  unlock();
}

// One read from the sender: an encrypted frame, or whatever plaintext is
// there.  The sender only ever replies to our requests, so the reply's status
// line is all that is looked at.  False when the connection is gone.
static bool read_reply(int client, uint8_t *buf, size_t size) {
  int len;
  if (s_encrypted) {
    len = rtsp_crypto_read_block_key(client, s_read_key, &s_read_nonce, buf,
                                     size - 1);
  } else {
    len = (int)recv(client, buf, size - 1, 0);
  }
  if (len <= 0) {
    return false;
  }
  buf[len] = '\0';
  if (strncmp((const char *)buf, "RTSP/1.0 ", 9) == 0) {
    char *eol = strpbrk((char *)buf, "\r\n");
    if (eol) {
      *eol = '\0';
    }
    const int status = atoi((const char *)buf + 9);
    if (status >= 200 && status < 300) {
      ESP_LOGI(TAG, "Sender replied: %s", (const char *)buf);
    } else {
      ESP_LOGW(TAG, "Sender refused the command: %s", (const char *)buf);
    }
  }
  return true;
}

static void event_port_task(void *pvParameters) {
  int listen_socket = (int)(intptr_t)pvParameters;
  event_listen_socket = listen_socket;
  uint8_t *rx = malloc(RTSP_ENCRYPTED_BLOCK_MAX + 1);

  while (!event_task_should_stop && listen_socket >= 0 && rx) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(listen_socket, &read_fds);

    struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
    int ret = select(listen_socket + 1, &read_fds, NULL, NULL, &tv);

    if (event_task_should_stop) {
      break;
    }
    if (ret < 0) {
      if (errno != EINTR && !event_task_should_stop) {
        ESP_LOGE(TAG, "Event port select error: %d", errno);
      }
      break;
    }
    if (ret == 0 || !FD_ISSET(listen_socket, &read_fds)) {
      continue;
    }

    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int client =
        accept(listen_socket, (struct sockaddr *)&client_addr, &addr_len);
    if (client < 0) {
      if (!event_task_should_stop) {
        ESP_LOGE(TAG, "Event port accept error: %d", errno);
      }
      break; // Don't continue - socket is likely invalid
    }
    struct timeval send_timeout = {.tv_sec = EVENT_SEND_TIMEOUT_S};
    setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &send_timeout,
               sizeof(send_timeout));

    detach_client();
    attach_client(client);
    ESP_LOGI(TAG, "Event client connected (%s)",
             s_encrypted ? "encrypted" : "plaintext");
    playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_CONNECTED,
                         NULL);

    // Read replies until the connection closes.
    while (!event_task_should_stop) {
      fd_set cfds;
      FD_ZERO(&cfds);
      FD_SET(client, &cfds);
      struct timeval ctv = {.tv_sec = 1, .tv_usec = 0};
      ret = select(client + 1, &cfds, NULL, NULL, &ctv);
      if (ret < 0 || event_task_should_stop) {
        break;
      }
      if (ret > 0 && FD_ISSET(client, &cfds) &&
          !read_reply(client, rx, RTSP_ENCRYPTED_BLOCK_MAX + 1)) {
        ESP_LOGI(TAG, "Event client disconnected");
        break;
      }
    }
    detach_client();
  }

  free(rx);
  detach_client();
  event_listen_socket = -1;
  event_task_handle = NULL;
  vTaskDelete(NULL);
}

static bool event_port_wait_for_task_stopped(int timeout_ticks) {
  while (event_task_handle != NULL && timeout_ticks-- > 0) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return event_task_handle == NULL;
}

esp_err_t rtsp_start_event_port_task(int listen_socket) {
  if (!s_lock) {
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
      return ESP_ERR_NO_MEM;
    }
  }
  if (event_task_handle != NULL) {
    rtsp_stop_event_port_task();
    if (!event_port_wait_for_task_stopped(20)) {
      ESP_LOGE(TAG, "Previous event port task did not stop");
      return ESP_ERR_INVALID_STATE;
    }
  }
  event_task_should_stop = false;
  event_listen_socket = -1;
  event_task_handle = NULL;
  BaseType_t ret =
      xTaskCreate(event_port_task, "event_port", EVENT_STACK_SIZE,
                  (void *)(intptr_t)listen_socket, 5, &event_task_handle);
  if (ret != pdPASS) {
    event_task_handle = NULL;
    ESP_LOGE(TAG, "Failed to create event port task");
    return ESP_FAIL;
  }
  return ESP_OK;
}

int rtsp_event_port_listen_socket(void) {
  return event_listen_socket;
}

void rtsp_stop_event_port_task(void) {
  if (event_task_handle == NULL) {
    return;
  }

  event_task_should_stop = true;

  // Shutdown sockets to unblock select() and a read in progress.
  const int client = event_client_socket;
  if (client >= 0) {
    shutdown(client, SHUT_RDWR);
  }
  if (event_listen_socket >= 0) {
    shutdown(event_listen_socket, SHUT_RDWR);
  }

  if (!event_port_wait_for_task_stopped(20)) {
    ESP_LOGW(TAG, "Event port task did not exit within timeout");
  }
}

bool rtsp_events_send_command(const uint8_t *body, size_t body_len) {
  if (!s_lock || !body || body_len == 0) {
    return false;
  }
  char header[160];
  bool sent = false;
  lock();
  if (event_client_socket >= 0) {
    const int header_len =
        snprintf(header, sizeof(header),
                 "POST /command RTSP/1.0\r\n"
                 "CSeq: %" PRIu32 "\r\n"
                 "Content-Type: application/x-apple-binary-plist\r\n"
                 "Content-Length: %u\r\n\r\n",
                 ++s_cseq, (unsigned)body_len);
    const size_t total = (size_t)header_len + body_len;
    uint8_t *request = malloc(total);
    if (request) {
      memcpy(request, header, (size_t)header_len);
      memcpy(request + header_len, body, body_len);
      if (s_encrypted) {
        sent = rtsp_crypto_write_frame_key(event_client_socket, s_write_key,
                                           &s_write_nonce, request, total) == 0;
      } else {
        sent = send(event_client_socket, request, total, 0) == (ssize_t)total;
      }
      free(request);
    }
    if (!sent) {
      // A frame cut short leaves the sender's decryptor out of step for good;
      // close the channel rather than send more into it.
      ESP_LOGW(TAG, "Event channel send failed (errno %d), closing it", errno);
      shutdown(event_client_socket, SHUT_RDWR);
    }
  }
  unlock();
  return sent;
}

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtsp_handlers.h"

#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sodium.h"

#include "audio_output.h"
#include "audio_receiver.h"
#include "audio_stream.h"
#ifdef CONFIG_BT_A2DP_ENABLE
#include "a2dp_sink.h"
#include "dac.h"
#endif
#include "hap.h"
#include "ntp_clock.h"
#include "ptp_clock.h"
#include "plist.h"
#include "rtsp_anchor.h"
#include "rtsp_fairplay.h"
#include "rtsp_rsa.h"
#include "settings.h"
#include "socket_utils.h"
#include "tlv8.h"

#include "playback_events.h"
#include "dacp_client.h"

static const char *TAG = "rtsp_handlers";

uint64_t airplay_features(void) {
  return settings_airplay_v1() ? UINT64_C(0x5C4A00) : UINT64_C(0x1C340405C4A00);
}

// ============================================================================
// Codec Registry
// ============================================================================
// To add a new codec, add an entry to codec_registry[] below.

static void configure_codec(audio_format_t *fmt, const char *name, int64_t sr,
                            int64_t spf) {
  strlcpy(fmt->codec, name, sizeof(fmt->codec));
  fmt->sample_rate = (int)sr;
  fmt->channels = 2;
  fmt->bits_per_sample = 16;
  fmt->frame_size = (int)spf;
  fmt->max_samples_per_frame = (uint32_t)spf;
  fmt->sample_size = 16;
  fmt->num_channels = 2;
  fmt->sample_rate_config = (uint32_t)sr;
}

// Codec registry - add new codecs here
// ct values: 2=ALAC, 4=AAC, 8=AAC-ELD, 64=OPUS (based on AirPlay 2 protocol)
static const rtsp_codec_t codec_registry[] = {
    {"ALAC", 2}, {"AAC", 4}, {"AAC-ELD", 8}, {"OPUS", 64}, {NULL, 0}};

bool rtsp_codec_configure(int64_t type_id, audio_format_t *fmt,
                          int64_t sample_rate, int64_t samples_per_frame) {
  for (const rtsp_codec_t *codec = codec_registry; codec->name; codec++) {
    if (codec->type_id == type_id) {
      configure_codec(fmt, codec->name, sample_rate, samples_per_frame);
      ESP_LOGI(TAG, "Configured codec: %s (ct=%lld, sr=%lld, spf=%lld)",
               codec->name, (long long)type_id, (long long)sample_rate,
               (long long)samples_per_frame);
      return true;
    }
  }
  // Default to ALAC if unknown codec type
  ESP_LOGW(TAG, "Unknown codec type %lld, defaulting to ALAC",
           (long long)type_id);
  configure_codec(fmt, "ALAC", sample_rate, samples_per_frame);
  return false;
}

// Event port task state
#define EVENT_STACK_SIZE 3072

// Default playout latency for realtime (type 96) streams when SETUP does not
// carry a usable latencyMin: 11025 samples = 250 ms at 44.1 kHz.  This is the
// standard AirPlay realtime-stream minimum latency; senders transmit audio
// ~2 s (88200 samples) ahead of this deadline.
#define AIRPLAY_RT_LATENCY_DEFAULT_SAMPLES 11025
// A sender latency beyond this is not believed (and not applied).
#define AIRPLAY_LATENCY_MAX_MS 5000

// Session liveness, as Apple's receiver keeps it: the session ends after this
// long with no request at all (/feedback counts), and TCP keepalive probes a
// silent RTSP connection after a fifth of it, 5 times.  A buffered stream
// stretches both, since a paused phone may send nothing but /feedback.
#define RTSP_IDLE_TIMEOUT_S          30
#define RTSP_IDLE_TIMEOUT_BUFFERED_S 120
#define RTSP_KEEPALIVE_PROBES        5

// The sender's NTP port when a session SETUP asks for NTP without naming one.
#define AIRPLAY_NTP_DEFAULT_PORT 7010

// Timing peers taken from one SETUP or SETPEERS.
#define AIRPLAY_MAX_TIMING_PEERS 16
static int event_client_socket = -1;
static int event_listen_socket = -1;
static TaskHandle_t event_task_handle = NULL;
static volatile bool event_task_should_stop = false;

void rtsp_get_device_id(char *device_id, size_t len) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(device_id, len, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5]);
}

static int rtsp_create_udp_socket(uint16_t *port) {
  uint16_t bound_port = 0;
  int sock = socket_utils_bind_udp(0, 0, 0, &bound_port);
  if (sock < 0) {
    return -1;
  }
  if (port) {
    *port = bound_port;
  }
  return sock;
}

static int rtsp_create_event_socket(uint16_t *port) {
  uint16_t bound_port = 0;
  int sock = socket_utils_bind_tcp_listener(0, 1, false, &bound_port);
  if (sock < 0) {
    return -1;
  }
  if (port) {
    *port = bound_port;
  }
  return sock;
}

static void ensure_stream_ports(rtsp_conn_t *conn, bool buffered) {
  int temp_socket = 0;
  if (!buffered && conn->data_port == 0) {
    temp_socket = rtsp_create_udp_socket(&conn->data_port);
    if (temp_socket > 0) {
      close(temp_socket);
    }
  }
  if (conn->control_port == 0) {
    temp_socket = rtsp_create_udp_socket(&conn->control_port);
    if (temp_socket > 0) {
      close(temp_socket);
    }
  }
  if (conn->timing_port == 0) {
    // Allocate a timing port for RTSP response (required by protocol)
    // Note: For AirPlay 1, we send timing requests TO the client, not receive
    // them
    temp_socket = rtsp_create_udp_socket(&conn->timing_port);
    if (temp_socket > 0) {
      close(temp_socket);
    }
  }
}

static bool start_ntp_timing_or_fail(int socket, rtsp_conn_t *conn,
                                     const rtsp_request_t *req) {
  if (conn->client_timing_port == 0 || conn->client_ip == 0) {
    return true;
  }

  esp_err_t err =
      ntp_clock_start_client(conn->client_ip, conn->client_timing_port);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start NTP timing client: %s",
             esp_err_to_name(err));
    rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                       NULL, 0);
    return false;
  }
  return true;
}

static bool start_audio_receiver_or_fail(int socket, rtsp_conn_t *conn,
                                         const rtsp_request_t *req,
                                         int64_t stream_type) {
#ifdef CONFIG_BT_A2DP_ENABLE
  // Tear the BT radio down here rather than waiting for the coex task's
  // PLAYING event: that lands ~600 ms after RECORD, by which time these tasks
  // have already tried and failed to allocate their stacks.
  if (!bt_a2dp_sink_is_connected()) {
    (void)bt_a2dp_sink_suspend();
  }
#endif
  audio_receiver_set_stream_type((audio_stream_type_t)stream_type);
  esp_err_t err = audio_receiver_start_stream(
      conn->data_port, conn->control_port, conn->buffered_port);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start audio receiver: %s", esp_err_to_name(err));
    rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                       NULL, 0);
    return false;
  }
  return true;
}

int rtsp_session_idle_limit_s(const rtsp_conn_t *conn) {
  return conn && conn->stream_active && conn->stream_type == 103
             ? RTSP_IDLE_TIMEOUT_BUFFERED_S
             : RTSP_IDLE_TIMEOUT_S;
}

void rtsp_set_keepalive(int socket, int idle_s) {
  int on = 1;
  int idle = idle_s;
  // [I] The spec gives the idle time and the probe count; the interval
  // between probes is taken as the idle time too, so a dead peer is noticed
  // within about the session idle timeout.
  int interval = idle_s;
  int probes = RTSP_KEEPALIVE_PROBES;
  if (setsockopt(socket, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on)) != 0 ||
      setsockopt(socket, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle)) != 0 ||
      setsockopt(socket, IPPROTO_TCP, TCP_KEEPINTVL, &interval,
                 sizeof(interval)) != 0 ||
      setsockopt(socket, IPPROTO_TCP, TCP_KEEPCNT, &probes, sizeof(probes)) !=
          0) {
    ESP_LOGW(TAG, "TCP keepalive not set: errno %d", errno);
  }
}

void rtsp_session_end_clock(void) {
  ptp_clock_set_peers(NULL, 0);
  ptp_clock_end_session();
}

// This end of the RTSP connection, for the receiver's own timing peer entry.
static bool rtsp_local_ipv4(int socket, uint32_t *addr) {
  struct sockaddr_in local;
  socklen_t len = sizeof(local);
  if (getsockname(socket, (struct sockaddr *)&local, &len) != 0 ||
      local.sin_family != AF_INET) {
    return false;
  }
  *addr = local.sin_addr.s_addr;
  return true;
}

// Peers named by a SETUP or SETPEERS, without this receiver itself.
static size_t collect_timing_peers(const uint8_t *body, size_t body_len,
                                   const char *key, uint32_t self,
                                   uint32_t *peers, size_t max) {
  size_t count = 0;
  if (!body || !bplist_collect_ipv4(body, body_len, key, peers, max, &count)) {
    return 0;
  }
  size_t kept = 0;
  for (size_t i = 0; i < count; i++) {
    if (peers[i] != self) {
      peers[kept++] = peers[i];
    }
  }
  return kept;
}

static void log_timing_peers(const char *what, const uint32_t *peers,
                             size_t count) {
  char list[AIRPLAY_MAX_TIMING_PEERS * 16 + 1] = "";
  size_t used = 0;
  for (size_t i = 0; i < count && used < sizeof(list); i++) {
    const uint8_t *b = (const uint8_t *)&peers[i];
    used += (size_t)snprintf(list + used, sizeof(list) - used, "%s%u.%u.%u.%u",
                             i ? " " : "", b[0], b[1], b[2], b[3]);
  }
  ESP_LOGI(TAG, "%s: %u timing peer(s) %s", what, (unsigned)count, list);
}

// Event port task - handles AirPlay 2 session persistence
static void event_port_task(void *pvParameters) {
  int listen_socket = (int)(intptr_t)pvParameters;
  event_listen_socket = listen_socket;

  while (!event_task_should_stop && listen_socket >= 0) {
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

    if (ret == 0) {
      continue;
    }

    // Check stop flag after select unblocks (shutdown() makes socket readable)
    if (event_task_should_stop) {
      break;
    }

    if (FD_ISSET(listen_socket, &read_fds)) {
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

      if (event_client_socket >= 0) {
        close(event_client_socket);
      }
      event_client_socket = client;
      ESP_LOGI(TAG, "Event client connected");
      playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_CONNECTED,
                           NULL);

      // Monitor connection for disconnection
      while (event_client_socket >= 0 && !event_task_should_stop) {
        fd_set cfds;
        FD_ZERO(&cfds);
        FD_SET(event_client_socket, &cfds);
        struct timeval ctv = {.tv_sec = 1, .tv_usec = 0};

        ret = select(event_client_socket + 1, &cfds, NULL, NULL, &ctv);
        if (ret < 0) {
          break;
        }
        // Check stop flag after select unblocks
        if (event_task_should_stop) {
          break;
        }
        if (ret > 0 && FD_ISSET(event_client_socket, &cfds)) {
          char buf[16];
          ssize_t n = recv(event_client_socket, buf, sizeof(buf), MSG_PEEK);
          if (n <= 0) {
            close(event_client_socket);
            event_client_socket = -1;
            break;
          }
        }
      }
    }
  }

  if (event_client_socket >= 0) {
    close(event_client_socket);
    event_client_socket = -1;
  }
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

  // Signal task to stop
  event_task_should_stop = true;

  // Shutdown sockets to unblock select()
  if (event_client_socket >= 0) {
    shutdown(event_client_socket, SHUT_RDWR);
  }
  if (event_listen_socket >= 0) {
    shutdown(event_listen_socket, SHUT_RDWR);
  }

  if (!event_port_wait_for_task_stopped(20)) {
    ESP_LOGW(TAG, "Event port task did not exit within timeout");
  }
}

// Forward declarations of handlers
static void handle_options(int socket, rtsp_conn_t *conn,
                           const rtsp_request_t *req, const uint8_t *raw,
                           size_t raw_len);
static void handle_get(int socket, rtsp_conn_t *conn, const rtsp_request_t *req,
                       const uint8_t *raw, size_t raw_len);
static void handle_post(int socket, rtsp_conn_t *conn,
                        const rtsp_request_t *req, const uint8_t *raw,
                        size_t raw_len);
static void handle_announce(int socket, rtsp_conn_t *conn,
                            const rtsp_request_t *req, const uint8_t *raw,
                            size_t raw_len);
static void handle_setup(int socket, rtsp_conn_t *conn,
                         const rtsp_request_t *req, const uint8_t *raw,
                         size_t raw_len);
static void handle_record(int socket, rtsp_conn_t *conn,
                          const rtsp_request_t *req, const uint8_t *raw,
                          size_t raw_len);
static void handle_set_parameter(int socket, rtsp_conn_t *conn,
                                 const rtsp_request_t *req, const uint8_t *raw,
                                 size_t raw_len);
static void handle_get_parameter(int socket, rtsp_conn_t *conn,
                                 const rtsp_request_t *req, const uint8_t *raw,
                                 size_t raw_len);
static void handle_pause(int socket, rtsp_conn_t *conn,
                         const rtsp_request_t *req, const uint8_t *raw,
                         size_t raw_len);
static void handle_flush(int socket, rtsp_conn_t *conn,
                         const rtsp_request_t *req, const uint8_t *raw,
                         size_t raw_len);
static void handle_flushbuffered(int socket, rtsp_conn_t *conn,
                                 const rtsp_request_t *req, const uint8_t *raw,
                                 size_t raw_len);
static void handle_teardown(int socket, rtsp_conn_t *conn,
                            const rtsp_request_t *req, const uint8_t *raw,
                            size_t raw_len);
static void handle_setrateanchortime(int socket, rtsp_conn_t *conn,
                                     const rtsp_request_t *req,
                                     const uint8_t *raw, size_t raw_len);
static void handle_setpeers(int socket, rtsp_conn_t *conn,
                            const rtsp_request_t *req, const uint8_t *raw,
                            size_t raw_len);

// Dispatch table
static const rtsp_method_handler_t method_handlers[] = {
    {"OPTIONS", handle_options},
    {"GET", handle_get},
    {"POST", handle_post},
    {"ANNOUNCE", handle_announce},
    {"SETUP", handle_setup},
    {"RECORD", handle_record},
    {"SET_PARAMETER", handle_set_parameter},
    {"GET_PARAMETER", handle_get_parameter},
    {"PAUSE", handle_pause},
    {"FLUSH", handle_flush},
    {"FLUSHBUFFERED", handle_flushbuffered},
    {"TEARDOWN", handle_teardown},
    {"SETRATEANCHORTIME", handle_setrateanchortime},
    {"SETPEERS", handle_setpeers},
    {"SETPEERSX", handle_setpeers},
    {NULL, NULL}};

// Parse a named header value from raw RTSP request data (case-insensitive).
// Returns pointer to a static buffer with the trimmed value, or NULL.
static const char *parse_raw_header(const uint8_t *raw, size_t raw_len,
                                    const char *name) {
  static char value_buf[64];
  if (!raw || !name) {
    return NULL;
  }

  const uint8_t *header_end = rtsp_find_header_end(raw, raw_len);
  if (!header_end) {
    return NULL;
  }

  const char *line = (const char *)raw;
  const char *end = (const char *)header_end;
  size_t name_len = strlen(name);

  while (line < end) {
    const char *line_end = line;
    while (line_end < end && *line_end != '\r' && *line_end != '\n') {
      line_end++;
    }

    if ((size_t)(line_end - line) >= name_len &&
        strncasecmp(line, name, name_len) == 0) {
      const char *hdr = line + name_len;
      // Skip optional whitespace
      while (hdr < line_end && (*hdr == ' ' || *hdr == '\t')) {
        hdr++;
      }

      size_t i = 0;
      while (i < sizeof(value_buf) - 1 && hdr < line_end) {
        value_buf[i] = *hdr;
        hdr++;
        i++;
      }
      value_buf[i] = '\0';
      return value_buf;
    }

    line = line_end;
    while (line < end && (*line == '\r' || *line == '\n')) {
      line++;
    }
  }

  return NULL;
}

static bool request_uses_rtsp(const rtsp_request_t *req) {
  return req && strncasecmp(req->protocol, "RTSP/", 5) == 0;
}

int rtsp_dispatch(int socket, rtsp_conn_t *conn, const uint8_t *raw_request,
                  size_t raw_len) {
  rtsp_request_t req;
  if (rtsp_request_parse(raw_request, raw_len, &req) < 0) {
    ESP_LOGW(TAG, "Failed to parse RTSP request");
    return -1;
  }

  ESP_LOGD(TAG, "<- %s %s", req.method, req.path);

  // Extract DACP headers if present (AirPlay 1 only — modern iOS AirPlay 2
  // does not send these; it uses MRP for remote control instead).
  // parse_raw_header uses a static buffer — copy before calling again.
  if (conn->dacp_id[0] == '\0') {
    const char *val = parse_raw_header(raw_request, raw_len, "DACP-ID:");
    if (val) {
      strlcpy(conn->dacp_id, val, sizeof(conn->dacp_id));
      ESP_LOGI(TAG, "DACP-ID: %s (from %s)", conn->dacp_id, req.method);
    }
  }
  if (conn->active_remote[0] == '\0') {
    const char *val = parse_raw_header(raw_request, raw_len, "Active-Remote:");
    if (val) {
      strlcpy(conn->active_remote, val, sizeof(conn->active_remote));
      ESP_LOGI(TAG, "Active-Remote: %s (from %s)", conn->active_remote,
               req.method);
    }
  }
  // Update DACP client session when both identifiers are available
  if (conn->dacp_id[0] != '\0' && conn->active_remote[0] != '\0') {
    dacp_set_session(conn->dacp_id, conn->active_remote, conn->client_ip);
  }

  // Find handler in dispatch table
  for (const rtsp_method_handler_t *h = method_handlers; h->method; h++) {
    if (strcasecmp(req.method, h->method) == 0) {
      h->handler(socket, conn, &req, raw_request, raw_len);
      return 0;
    }
  }

  ESP_LOGW(TAG, "Unknown method: %s", req.method);
  if (request_uses_rtsp(&req)) {
    rtsp_send_response(socket, conn, 501, "Not Implemented", req.cseq,
                       "Content-Type: text/plain\r\n", "Not Implemented", 15);
  } else {
    rtsp_send_http_response(socket, conn, 501, "Not Implemented", "text/plain",
                            "Not Implemented", 15);
  }
  return 0;
}

// ============================================================================
// Handler implementations
// ============================================================================

static void handle_options(int socket, rtsp_conn_t *conn,
                           const rtsp_request_t *req, const uint8_t *raw,
                           size_t raw_len) {
  // AirPlay v1: handle Apple-Challenge if present. Triggered by request
  // shape, so safe unconditionally — iOS in AirPlay 2 mode does not send
  // this header.
  const char *challenge = parse_raw_header(raw, raw_len, "Apple-Challenge:");
  if (challenge) {
    conn->protocol_version = 1;
  }

  // A classic-only sender inspects this list and gives up straight after
  // OPTIONS if it sees the AirPlay 2 methods, so answer in the dialect this
  // connection is speaking rather than the one the receiver advertises.
  const char *public_methods =
      (conn->protocol_version == 1 || settings_airplay_v1())
          ? "Public: ANNOUNCE, SETUP, RECORD, PAUSE, FLUSH, TEARDOWN, OPTIONS, "
            "GET_PARAMETER, SET_PARAMETER\r\n"
          : "Public: ANNOUNCE, SETUP, RECORD, PAUSE, FLUSH, FLUSHBUFFERED, "
            "TEARDOWN, OPTIONS, POST, GET, SET_PARAMETER, GET_PARAMETER, "
            "SETPEERS, SETRATEANCHORTIME\r\n";

  if (challenge) {
    // The sender verifies that the response embeds the address it connected
    // to, so take it from the socket: hardcoding the WiFi netif yields 0.0.0.0
    // on an Ethernet-attached board and the sender then walks away silently.
    struct sockaddr_in local = {0};
    socklen_t local_len = sizeof(local);
    if (getsockname(socket, (struct sockaddr *)&local, &local_len) == 0 &&
        local.sin_addr.s_addr != 0) {
      uint8_t mac[6];
      esp_read_mac(mac, ESP_MAC_WIFI_STA);

      char response_b64[512];
      if (rsa_apple_challenge_response(challenge, local.sin_addr.s_addr, mac,
                                       response_b64,
                                       sizeof(response_b64)) == 0) {
        ESP_LOGI(TAG, "Apple-Challenge answered for local IP %s",
                 inet_ntoa(local.sin_addr));
        char headers[768];
        snprintf(headers, sizeof(headers), "%sApple-Response: %s\r\n",
                 public_methods, response_b64);
        rtsp_send_response(socket, conn, 200, "OK", req->cseq, headers, NULL,
                           0);
        return;
      }
    }
    ESP_LOGW(TAG, "Failed to build Apple-Challenge response");
  }

  rtsp_send_response(socket, conn, 200, "OK", req->cseq, public_methods, NULL,
                     0);
}

static void handle_get(int socket, rtsp_conn_t *conn, const rtsp_request_t *req,
                       const uint8_t *raw, size_t raw_len) {
  (void)raw;
  (void)raw_len;

  if (strcmp(req->path, "/info") == 0) {
    // Build info response
    char device_id[18];
    char device_name[65];

    rtsp_get_device_id(device_id, sizeof(device_id));
    settings_get_device_name(device_name, sizeof(device_name));
    const uint8_t *pk = hap_get_public_key();
    uint64_t features = airplay_features();

    int64_t protocol_version = settings_airplay_v1() ? 1 : 2;

    if (request_uses_rtsp(req)) {
      static uint8_t body[1024];
      size_t body_len =
          bplist_build_info_response(body, sizeof(body), device_id, device_name,
                                     pk, 32, features, protocol_version);
      if (body_len == 0) {
        ESP_LOGE(TAG, "Failed to build binary /info response");
        rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                           NULL, 0);
        return;
      }
      rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                         "Content-Type: application/x-apple-binary-plist\r\n",
                         (const char *)body, body_len);
      return;
    }

    static char body[4096];
    plist_t p;

    plist_init(&p, body, sizeof(body));
    plist_begin(&p);
    plist_dict_begin(&p);

    plist_dict_string(&p, "deviceid", device_id);
    plist_dict_uint(&p, "features", features);
    plist_dict_string(&p, "model", "AudioAccessory5,1");
    plist_dict_string(&p, "protovers", "1.1");
    plist_dict_string(&p, "srcvers", "377.40.00");
    plist_dict_int(&p, "vv", protocol_version);
    plist_dict_int(&p, "statusFlags", 4);
    plist_dict_data(&p, "pk", pk, 32);
    plist_dict_string(&p, "pi", "00000000-0000-0000-0000-000000000000");
    plist_dict_string(&p, "name", device_name);

    // Audio formats array
    plist_dict_array_begin(&p, "audioFormats");
    plist_dict_begin(&p);
    plist_dict_int(&p, "type", 96);
    plist_dict_int(&p, "audioInputFormats", 0x01000000);
    plist_dict_int(&p, "audioOutputFormats", 0x01000000);
    plist_dict_end(&p);
    plist_array_end(&p);

    // Audio latencies array
    // Both type 96 (realtime/UDP) and type 103 (buffered/TCP) use PTP-based
    // anchor timing with internal hardware-latency compensation in
    // compute_early_us().  Report 0 so the sender does NOT also adjust its
    // anchor — otherwise the hardware pipeline delay is subtracted twice and
    // the ESP plays ahead of other speakers.  shairport-sync likewise
    // reports no audioLatencies at all.
    plist_dict_array_begin(&p, "audioLatencies");
    plist_dict_begin(&p);
    plist_dict_int(&p, "type", 96);
    plist_dict_int(&p, "audioType", 0x64);
    plist_dict_int(&p, "inputLatencyMicros", 0);
    plist_dict_int(&p, "outputLatencyMicros", 0);
    plist_dict_end(&p);
    plist_dict_begin(&p);
    plist_dict_int(&p, "type", 103);
    plist_dict_int(&p, "audioType", 0x64);
    plist_dict_int(&p, "inputLatencyMicros", 0);
    plist_dict_int(&p, "outputLatencyMicros", 0);
    plist_dict_end(&p);
    plist_array_end(&p);

    plist_dict_end(&p);
    size_t body_len = plist_end(&p);

    rtsp_send_http_response(socket, conn, 200, "OK", "text/x-apple-plist+xml",
                            body, body_len);
  } else {
    ESP_LOGW(TAG, "Unknown GET path: %s", req->path);
    if (request_uses_rtsp(req)) {
      rtsp_send_response(socket, conn, 404, "Not Found", req->cseq,
                         "Content-Type: text/plain\r\n", "Not Found", 9);
    } else {
      rtsp_send_http_response(socket, conn, 404, "Not Found", "text/plain",
                              "Not Found", 9);
    }
  }
}

static void handle_post(int socket, rtsp_conn_t *conn,
                        const rtsp_request_t *req, const uint8_t *raw,
                        size_t raw_len) {
  (void)raw;
  (void)raw_len;

  const uint8_t *body = req->body;
  size_t body_len = req->body_len;

  if (strstr(req->path, "/pair-setup")) {
    conn->protocol_version = 2;
    // Create session if needed
    if (!conn->hap_session) {
      conn->hap_session = hap_session_create();
      if (!conn->hap_session) {
        ESP_LOGE(TAG, "Failed to create HAP session");
        rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                           NULL, 0);
        return;
      }
    }

    uint8_t *response = malloc(2048);
    if (!response) {
      rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                         NULL, 0);
      return;
    }

    size_t response_len = 0;
    esp_err_t err = ESP_FAIL;

    if (body && body_len > 0) {
      size_t state_len;
      const uint8_t *state =
          tlv8_find(body, body_len, TLV_TYPE_STATE, &state_len);

      if (state && state_len == 1) {
        switch (state[0]) {
        default:
          break;
        case 1:
          err = hap_pair_setup_m1(conn->hap_session, body, body_len, response,
                                  2048, &response_len);
          break;
        case 3:
          err = hap_pair_setup_m3(conn->hap_session, body, body_len, response,
                                  2048, &response_len);
          break;
        case 5:
          err = hap_pair_setup_m5(conn->hap_session, body, body_len, response,
                                  2048, &response_len);
          break;
        }
      }
    }

    if (err == ESP_OK && response_len > 0) {
      rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                         "Content-Type: application/octet-stream\r\n",
                         (const char *)response, response_len);

      if (conn->hap_session && conn->hap_session->pair_setup_state == 4 &&
          conn->hap_session->session_established) {
        conn->encrypted_mode = true;
      }
    } else {
      ESP_LOGE(TAG, "Pair-setup failed: err=%d", err);
      static const uint8_t error_response[] = {0x06, 0x01, 0x02,
                                               0x07, 0x01, 0x02};
      rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                         "Content-Type: application/octet-stream\r\n",
                         (const char *)error_response, sizeof(error_response));
    }

    free(response);

  } else if (strstr(req->path, "/pair-verify")) {
    conn->protocol_version = 2;
    if (!conn->hap_session) {
      conn->hap_session = hap_session_create();
      if (!conn->hap_session) {
        rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                           NULL, 0);
        return;
      }
    }

    uint8_t *response = malloc(1024);
    if (!response) {
      rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                         NULL, 0);
      return;
    }

    size_t response_len = 0;
    esp_err_t err = ESP_FAIL;

    if (body && body_len > 0) {
      size_t state_len;
      const uint8_t *state =
          tlv8_find(body, body_len, TLV_TYPE_STATE, &state_len);

      if (state && state_len == 1) {
        if (state[0] == 0x01) {
          err = hap_pair_verify_m1(conn->hap_session, body, body_len, response,
                                   1024, &response_len);
        } else if (state[0] == 0x03) {
          err = hap_pair_verify_m3(conn->hap_session, body, body_len, response,
                                   1024, &response_len);
          // TLV8 pair-verify M3 establishes RTSP channel encryption
          if (err == ESP_OK &&
              conn->hap_session->pair_verify_state == PAIR_VERIFY_STATE_M4) {
            conn->encrypted_mode = true;
            ESP_LOGI(TAG, "RTSP encryption enabled (TLV8 pair-verify)");
          }
        }
      } else {
        // Raw format - used for audio encryption keys, not RTSP encryption
        if (conn->hap_session->pair_verify_state == 0) {
          err = hap_pair_verify_m1_raw(conn->hap_session, body, body_len,
                                       response, 1024, &response_len);
        } else if (conn->hap_session->pair_verify_state ==
                   PAIR_VERIFY_STATE_M2) {
          err = hap_pair_verify_m3_raw(conn->hap_session, body, body_len,
                                       response, 1024, &response_len);
          if (err == ESP_OK) {
            ESP_LOGI(TAG, "Raw pair-verify complete (RTSP unencrypted)");
          }
        }
      }
    }

    if (err == ESP_OK && response_len > 0) {
      rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                         "Content-Type: application/octet-stream\r\n",
                         (const char *)response, response_len);
    } else {
      ESP_LOGE(TAG, "Pair-verify failed, err=%d", err);
      rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                         "Content-Type: application/octet-stream\r\n",
                         "\x06\x01\x04\x07\x01\x02", 6);
    }

    free(response);

  } else if (strstr(req->path, "/fp-setup")) {
    uint8_t *fp_response = NULL;
    size_t fp_response_len = 0;

    if (body && body_len >= 16) {
      if (rtsp_fairplay_handle(body, body_len, &fp_response,
                               &fp_response_len) == 0) {
        rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                           "Content-Type: application/octet-stream\r\n",
                           (const char *)fp_response, fp_response_len);
        free(fp_response);
        return;
      }
    }

    rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                       "Content-Type: application/octet-stream\r\n", "\x00", 1);

  } else if (strstr(req->path, "/command")) {
    if (body && body_len >= 8 && memcmp(body, "bplist00", 8) == 0) {
      int64_t cmd_type = 0;
      if (bplist_find_int(body, body_len, "type", &cmd_type)) {
        ESP_LOGI(TAG, "/command type=%lld", (long long)cmd_type);
      }
    }
    rtsp_send_ok(socket, conn, req->cseq);

  } else if (strstr(req->path, "/feedback")) {
    if (body && body_len >= 8 && memcmp(body, "bplist00", 8) == 0) {
      int64_t value;
      if (bplist_find_int(body, body_len, "networkTimeSecs", &value)) {
        ESP_LOGI(TAG, "/feedback has networkTimeSecs=%lld", (long long)value);
      }
    }

    // For buffered audio streams (type 103), send a proper feedback response
    // with stream status. This acts as a keepalive to prevent iPhone from
    // sending TEARDOWN during extended pause.
    if (conn->stream_type == 103) {
      uint8_t response[128];
      size_t response_len = bplist_build_feedback_response(
          response, sizeof(response), conn->stream_type, 44100.0);

      if (response_len > 0) {
        ESP_LOGD(
            TAG,
            "/feedback responding with stream status (type=%lld, sr=44100)",
            (long long)conn->stream_type);
        rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                           "Content-Type: application/x-apple-binary-plist\r\n",
                           (const char *)response, response_len);
      } else {
        // Fallback to simple OK if response build fails
        rtsp_send_ok(socket, conn, req->cseq);
      }
    } else {
      // For non-buffered streams, simple OK is fine
      rtsp_send_ok(socket, conn, req->cseq);
    }

  } else {
    rtsp_send_ok(socket, conn, req->cseq);
  }
}

static void parse_sdp(rtsp_conn_t *conn, const char *sdp, size_t len) {
  (void)len;

  audio_format_t format = {0};
  audio_encrypt_t encrypt = {0};
  encrypt.type = AUDIO_ENCRYPT_NONE;

  format.sample_rate = 44100;
  format.channels = 2;
  format.bits_per_sample = 16;
  format.frame_size = 352;
  strcpy(format.codec, "AppleLossless");

  const char *rtpmap = strstr(sdp, "a=rtpmap:");
  if (rtpmap) {
    sscanf(rtpmap, "a=rtpmap:%*d %31s", format.codec);
    char *slash = strchr(format.codec, '/');
    if (slash) {
      *slash = '\0';
      int sr = 0;
      int ch = 0;
      // NOLINTNEXTLINE(bugprone-unchecked-string-to-number-conversion)
      if (sscanf(slash + 1, "%d/%d", &sr, &ch) >= 1) {
        if (sr > 0) {
          format.sample_rate = sr;
        }
        if (ch > 0) {
          format.channels = ch;
        }
      }
    }
  }

  const char *fmtp = strstr(sdp, "a=fmtp:");
  if (fmtp) {
    unsigned int frame_len, bit_depth, pb, mb, kb, num_ch, max_run, max_frame,
        avg_rate, rate;
    unsigned int compat;
    // NOLINTNEXTLINE(bugprone-unchecked-string-to-number-conversion)
    int matched = sscanf(fmtp, "a=fmtp:%*d %u %u %u %u %u %u %u %u %u %u %u",
                         &frame_len, &compat, &bit_depth, &pb, &mb, &kb,
                         &num_ch, &max_run, &max_frame, &avg_rate, &rate);
    if (matched >= 7) {
      format.max_samples_per_frame = frame_len;
      format.sample_size = bit_depth;
      format.rice_history_mult = pb;
      format.rice_initial_history = mb;
      format.rice_limit = kb;
      format.num_channels = num_ch;
      format.channels = (int)num_ch;
      format.bits_per_sample = (int)bit_depth;
      if (matched >= 8) {
        format.max_run = max_run;
      }
      if (matched >= 9) {
        format.max_coded_frame_size = max_frame;
      }
      if (matched >= 10) {
        format.avg_bit_rate = avg_rate;
      }
      if (matched >= 11) {
        format.sample_rate_config = rate;
        format.sample_rate = (int)rate;
      }
    }
  }

  if ((strstr(format.codec, "AAC") || strstr(format.codec, "aac") ||
       strstr(format.codec, "mpeg4-generic") ||
       strstr(format.codec, "MPEG4-GENERIC")) &&
      format.max_samples_per_frame == 0) {
    format.frame_size = 1024;
    format.max_samples_per_frame = 1024;
  }

  // AirPlay v1: parse RSA-encrypted AES key and IV from SDP. Triggered by
  // SDP shape so safe unconditionally — AirPlay 2 uses HAP-derived keys
  // and never embeds rsaaeskey in ANNOUNCE.
  const char *rsaaeskey = strcasestr(sdp, "rsaaeskey:");
  const char *aesiv_str = strcasestr(sdp, "aesiv:");
  if (rsaaeskey && aesiv_str) {
    conn->protocol_version = 1;
    // Extract base64 key (may span multiple lines, concatenate until next
    // field or end of SDP). In practice it's a single long base64 line.
    rsaaeskey += strlen("rsaaeskey:");
    while (*rsaaeskey == ' ' || *rsaaeskey == '\t') {
      rsaaeskey++;
    }

    // Collect key characters (skip whitespace/newlines within base64)
    char key_b64[512];
    size_t ki = 0;
    for (const char *p = rsaaeskey; *p && ki < sizeof(key_b64) - 1; p++) {
      if (*p == '\r' || *p == '\n') {
        // Check if next non-space char is start of a new SDP field (e.g. "a=")
        const char *q = p + 1;
        while (*q == '\r' || *q == '\n' || *q == ' ') {
          q++;
        }
        if (*q == 'a' && *(q + 1) == '=') {
          break;
        }
      } else if (*p != ' ' && *p != '\t') {
        key_b64[ki++] = *p;
      }
    }
    key_b64[ki] = '\0';

    // Extract IV
    aesiv_str += strlen("aesiv:");
    while (*aesiv_str == ' ' || *aesiv_str == '\t') {
      aesiv_str++;
    }
    char iv_b64[64];
    size_t ii = 0;
    for (const char *p = aesiv_str;
         *p && *p != '\r' && *p != '\n' && ii < sizeof(iv_b64) - 1; p++) {
      iv_b64[ii++] = *p;
    }
    iv_b64[ii] = '\0';

    // Decrypt AES key using RSA
    uint8_t aes_key[32];
    size_t aes_key_len = 0;
    if (rsa_decrypt_aes_key(key_b64, aes_key, sizeof(aes_key), &aes_key_len) ==
            0 &&
        aes_key_len >= 16) {
      encrypt.type = AUDIO_ENCRYPT_AES_CBC;
      memcpy(encrypt.key, aes_key, aes_key_len);
      encrypt.key_len = aes_key_len;

      // Decode IV
      size_t iv_len = 0;
      if (sodium_base642bin(encrypt.iv, sizeof(encrypt.iv), iv_b64,
                            strlen(iv_b64), "\r\n \t", &iv_len, NULL,
                            sodium_base64_VARIANT_ORIGINAL_NO_PADDING) != 0) {
        sodium_base642bin(encrypt.iv, sizeof(encrypt.iv), iv_b64,
                          strlen(iv_b64), "\r\n \t", &iv_len, NULL,
                          sodium_base64_VARIANT_ORIGINAL);
      }
      ESP_LOGI(TAG,
               "AirPlay v1: AES-CBC encryption configured (key=%zu iv=%zu)",
               aes_key_len, iv_len);
    }
  }

  // Update connection state
  strlcpy(conn->codec, format.codec, sizeof(conn->codec));
  conn->sample_rate = format.sample_rate;
  conn->channels = format.channels;
  conn->bits_per_sample = format.bits_per_sample;

  audio_receiver_set_format(&format);

  if (encrypt.type != AUDIO_ENCRYPT_NONE) {
    audio_receiver_set_encryption(&encrypt);
  }
}

static void handle_announce(int socket, rtsp_conn_t *conn,
                            const rtsp_request_t *req, const uint8_t *raw,
                            size_t raw_len) {
  (void)raw;
  (void)raw_len;

  if (req->body && req->body_len > 0) {
    parse_sdp(conn, (const char *)req->body, req->body_len);
  }

  rtsp_send_ok(socket, conn, req->cseq);
}

// A stable identifier for this receiver's timing peer entry.  Apple's is its
// boot UUID; this one is derived from the MAC, in the same shape.
static void timing_peer_id(char *out, size_t len) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(out, len, "a1b2c3d4-0000-4000-8000-%02X%02X%02X%02X%02X%02X", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Session SETUP timing, as Apple's receiver sets it up: timingProtocol
// decides the clock (absent means NTP), and under PTP the clock is started
// and given its peers here, before any stream, RECORD or anchor, so that it
// can be locked by the time the first anchor arrives.
static void session_setup_timing(rtsp_conn_t *conn, const uint8_t *body,
                                 size_t body_len, uint32_t self_ip) {
  char protocol[16] = "";
  const bool named = bplist_find_string(body, body_len, "timingProtocol",
                                        protocol, sizeof(protocol));
  // "None" is for remote-control and persistent sessions, which borrow
  // another session's clock; there is none here to borrow but PTP.
  conn->timing_ptp =
      named && (strcmp(protocol, "PTP") == 0 || strcmp(protocol, "None") == 0);
  audio_receiver_set_session_timing(conn->timing_ptp);
  ESP_LOGI(TAG, "SETUP: timingProtocol=%s -> %s", named ? protocol : "(absent)",
           conn->timing_ptp ? "PTP" : "NTP");

  if (!conn->timing_ptp) {
    int64_t port = AIRPLAY_NTP_DEFAULT_PORT;
    (void)bplist_find_int(body, body_len, "timingPort", &port);
    if (port > 0 && port <= UINT16_MAX && conn->client_ip != 0) {
      conn->client_timing_port = (uint16_t)port;
      esp_err_t err =
          ntp_clock_start_client(conn->client_ip, conn->client_timing_port);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "NTP timing client failed: %s", esp_err_to_name(err));
      }
    }
    return;
  }

  // Already running unless an AirPlay 1 session stopped it.
  (void)ptp_clock_init();
  uint32_t peers[AIRPLAY_MAX_TIMING_PEERS];
  size_t count = collect_timing_peers(body, body_len, "timingPeerInfo", self_ip,
                                      peers, AIRPLAY_MAX_TIMING_PEERS);
  if (count == 0 && conn->client_ip != 0 && conn->client_ip != self_ip) {
    // No peer info: the sender's own address, as Apple falls back to.
    peers[count++] = conn->client_ip;
  }
  uint32_t list[AIRPLAY_MAX_TIMING_PEERS];
  const size_t listed =
      collect_timing_peers(body, body_len, "timingPeerList", self_ip, list,
                           AIRPLAY_MAX_TIMING_PEERS);
  for (size_t i = 0; i < listed && count < AIRPLAY_MAX_TIMING_PEERS; i++) {
    bool known = false;
    for (size_t j = 0; j < count; j++) {
      known = known || peers[j] == list[i];
    }
    if (!known) {
      peers[count++] = list[i];
    }
  }
  log_timing_peers("SETUP", peers, count);
  ptp_clock_set_peers(peers, count);
}

// One stream dict's keys on one line, for the hardware test: what senders put
// in a buffered SETUP (latencyMin? audioLatencyMs?) is still an open question.
static void log_stream_keys(const bplist_kv_info_t *kv, size_t count,
                            int64_t stream_type) {
  char line[512];
  size_t used = (size_t)snprintf(
      line, sizeof(line), "SETUP stream type %lld:", (long long)stream_type);
  for (size_t k = 0; k < count && used < sizeof(line); k++) {
    if (kv[k].value_type == BPLIST_VALUE_INT) {
      used += (size_t)snprintf(line + used, sizeof(line) - used, " %s=%lld",
                               kv[k].key, (long long)kv[k].int_value);
    } else {
      used += (size_t)snprintf(line + used, sizeof(line) - used, " %s(%u:%u)",
                               kv[k].key, (unsigned)kv[k].value_type,
                               (unsigned)kv[k].value_len);
    }
  }
  ESP_LOGI(TAG, "%s", line);
}

// The sender's latency for a stream, in samples: audioLatencyMs when present
// and non-zero (Apple then uses it for both min and max), else latencyMin.
static bool stream_sender_latency(const bplist_kv_info_t *kv, size_t count,
                                  int64_t sample_rate, int64_t *samples,
                                  const char **source) {
  int64_t latency_ms = 0;
  int64_t latency_min = -1;
  for (size_t k = 0; k < count; k++) {
    if (kv[k].value_type != BPLIST_VALUE_INT) {
      continue;
    }
    if (strcmp(kv[k].key, "audioLatencyMs") == 0) {
      latency_ms = kv[k].int_value;
    } else if (strcmp(kv[k].key, "latencyMin") == 0) {
      latency_min = kv[k].int_value;
    }
  }
  const int64_t max_samples = sample_rate * AIRPLAY_LATENCY_MAX_MS / 1000;
  if (latency_ms > 0 && latency_ms <= AIRPLAY_LATENCY_MAX_MS) {
    *samples = sample_rate * latency_ms / 1000;
    *source = "audioLatencyMs";
    return true;
  }
  if (latency_min >= 0 && latency_min <= max_samples) {
    *samples = latency_min;
    *source = "latencyMin";
    return true;
  }
  return false;
}

static void handle_setup(int socket, rtsp_conn_t *conn,
                         const rtsp_request_t *req, const uint8_t *raw,
                         size_t raw_len) {
  (void)raw_len;

  const uint8_t *body = req->body;
  size_t body_len = req->body_len;

  bool is_bplist =
      strstr(req->content_type, "application/x-apple-binary-plist") != NULL;

  // Check for streams array
  bool request_has_streams = false;
  size_t stream_count = 0;
  if (body && body_len >= 8 && memcmp(body, "bplist00", 8) == 0) {
    if (bplist_get_streams_count(body, body_len, &stream_count)) {
      request_has_streams = true;
    }
  }

  ESP_LOGI(TAG, "SETUP: has_streams=%d, stream_count=%zu", request_has_streams,
           stream_count);

  // AirPlay v1 stream SETUP is identified by a Transport header and no bplist
  // streams array. Classify it before opening AirPlay 2-only resources.
  bool is_v1_transport_setup =
      !request_has_streams &&
      parse_raw_header(raw, raw_len, "Transport:") != NULL;

  if (body && body_len > 0 && is_bplist && request_has_streams) {
    conn->protocol_version = 2;
    for (size_t i = 0; i < stream_count; i++) {
      int64_t stream_type = -1;
      size_t ekey_len = 0, eiv_len = 0, shk_len = 0;
      if (bplist_get_stream_info(body, body_len, i, &stream_type, &ekey_len,
                                 &eiv_len, &shk_len)) {
        if (i == 0) {
          conn->stream_type = stream_type;
          audio_receiver_set_stream_type((audio_stream_type_t)stream_type);
        }

        bplist_kv_info_t kv[16];
        size_t kv_count = 0;
        int64_t codec_type = -1;
        int64_t sample_rate = 44100;
        int64_t samples_per_frame = 352;

        if (bplist_get_stream_kv_info(body, body_len, i, kv, 16, &kv_count)) {
          for (size_t k = 0; k < kv_count; k++) {
            if (kv[k].value_type == BPLIST_VALUE_INT) {
              if (strcmp(kv[k].key, "ct") == 0) {
                codec_type = kv[k].int_value;
              } else if (strcmp(kv[k].key, "sr") == 0) {
                sample_rate = kv[k].int_value;
              } else if (strcmp(kv[k].key, "spf") == 0) {
                samples_per_frame = kv[k].int_value;
              } else if (strcmp(kv[k].key, "controlPort") == 0) {
                conn->client_control_port = (uint16_t)kv[k].int_value;
              }
            }
          }

          // Use codec registry to configure audio format
          audio_format_t format = {0};
          rtsp_codec_configure(codec_type, &format, sample_rate,
                               samples_per_frame);
          audio_receiver_set_format(&format);
          if (i == 0) {
            conn->sample_rate = format.sample_rate;
          }
        }
      }
    }
  }

  // Process encryption keys
  if (body && body_len > 0) {
    uint8_t ekey_encrypted[64];
    size_t ekey_len = 0;
    uint8_t eiv[16];
    size_t eiv_len = 0;
    uint8_t shk[32];
    size_t shk_len = 0;

    int64_t crypto_stream_type = conn->stream_type > 0 ? conn->stream_type : 96;
    bool has_stream_crypto = bplist_find_stream_crypto(
        body, body_len, crypto_stream_type, ekey_encrypted,
        sizeof(ekey_encrypted), &ekey_len, eiv, sizeof(eiv), &eiv_len, shk,
        sizeof(shk), &shk_len);

    if (!has_stream_crypto || (ekey_len == 0 && shk_len == 0)) {
      bplist_find_data_deep(body, body_len, "ekey", ekey_encrypted,
                            sizeof(ekey_encrypted), &ekey_len);
      bplist_find_data_deep(body, body_len, "eiv", eiv, sizeof(eiv), &eiv_len);
      bplist_find_data_deep(body, body_len, "shk", shk, sizeof(shk), &shk_len);
    }

    // iOS 26.5 and later leave the key out when an app picks the receiver
    // itself (Spotify's in-app picker, "AirplayHandoffDevice") for a realtime
    // stream.  Nothing derived from the pairing decrypts it; a HomePod plays
    // it, third-party receivers do not (shairport-sync #2238).  Say so, since
    // the session otherwise looks healthy and simply stays silent.
    if (request_has_streams && shk_len == 0 && ekey_len == 0) {
      ESP_LOGW(TAG, "SETUP: stream has no key (shk); its audio cannot be "
                    "decrypted. Pick this speaker from Control Center instead "
                    "of the app.");
    }

    audio_encrypt_t audio_encrypt = {0};
    bool encryption_set = false;

    if (shk_len >= 16) {
      audio_encrypt.type = AUDIO_ENCRYPT_CHACHA20_POLY1305;
      memcpy(audio_encrypt.key, shk, shk_len > 32 ? 32 : shk_len);
      audio_encrypt.key_len = shk_len > 32 ? 32 : shk_len;
      if (eiv_len >= 16) {
        memcpy(audio_encrypt.iv, eiv, 16);
      }
      audio_receiver_set_encryption(&audio_encrypt);
      encryption_set = true;
    } else if (ekey_len > 16 && conn->hap_session &&
               conn->hap_session->session_established) {
      uint8_t nonce[12] = {0};
      uint8_t decrypted_key[32];
      unsigned long long decrypted_len;

      if (crypto_aead_chacha20poly1305_ietf_decrypt(
              decrypted_key, &decrypted_len, NULL, ekey_encrypted, ekey_len,
              NULL, 0, nonce, conn->hap_session->shared_secret) == 0 &&
          decrypted_len >= 16) {
        audio_encrypt.type = AUDIO_ENCRYPT_CHACHA20_POLY1305;
        memcpy(audio_encrypt.key, decrypted_key,
               decrypted_len > 32 ? 32 : decrypted_len);
        audio_encrypt.key_len = decrypted_len > 32 ? 32 : decrypted_len;
        if (eiv_len >= 16) {
          memcpy(audio_encrypt.iv, eiv, 16);
        }
        audio_receiver_set_encryption(&audio_encrypt);
        encryption_set = true;
      }
    }

    if (!encryption_set && conn->hap_session &&
        conn->hap_session->session_established) {
      audio_encrypt.type = AUDIO_ENCRYPT_CHACHA20_POLY1305;
      if (hap_derive_audio_key(conn->hap_session, audio_encrypt.key,
                               sizeof(audio_encrypt.key)) == ESP_OK) {
        audio_encrypt.key_len = 32;
        if (eiv_len >= 16) {
          memcpy(audio_encrypt.iv, eiv, 16);
        }
        audio_receiver_set_encryption(&audio_encrypt);
      }
    }
  }

  // Create event port if needed
  if (!is_v1_transport_setup && conn->event_port == 0) {
    conn->event_socket = rtsp_create_event_socket(&conn->event_port);
    if (conn->event_socket >= 0) {
      if (rtsp_start_event_port_task(conn->event_socket) == ESP_OK) {
        ESP_LOGI(TAG, "SETUP: Created event port %u", conn->event_port);
      } else {
        close(conn->event_socket);
        conn->event_socket = -1;
        conn->event_port = 0;
        rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                           NULL, 0);
        return;
      }
    }
  }

  // Handle initial SETUP vs stream SETUP
  if (!request_has_streams) {
    // AirPlay v1: SETUP has no bplist body — transport info is in the header.
    // Detected by request shape (Transport: header present); AirPlay 2's
    // initial SETUP has neither streams nor a Transport header.
    if (is_v1_transport_setup) {
      ESP_LOGI(TAG, "SETUP: AirPlay v1 stream setup");
      conn->protocol_version = 1;
      int64_t stream_type = 96; // RTP
      conn->stream_type = stream_type;

      // Parse client's control and timing ports from Transport header
      rtsp_parse_transport((const char *)raw, &conn->client_control_port,
                           &conn->client_timing_port);
      ESP_LOGI(TAG, "Client ports: control=%u timing=%u",
               conn->client_control_port, conn->client_timing_port);

      if (!start_ntp_timing_or_fail(socket, conn, req)) {
        return;
      }

      ensure_stream_ports(conn, false);

      char transport_response[256];
      snprintf(transport_response, sizeof(transport_response),
               "Transport: RTP/AVP/UDP;unicast;mode=record;"
               "server_port=%d;control_port=%d;timing_port=%d\r\n"
               "Session: 1\r\n",
               conn->data_port, conn->control_port, conn->timing_port);
      rtsp_send_response(socket, conn, 200, "OK", req->cseq, transport_response,
                         NULL, 0);

      // Configure audio format — RAOP default is ALAC 44100/352
      audio_format_t format = {0};
      rtsp_codec_configure(2, &format, 44100, 352); // ct=2 is ALAC
      audio_receiver_set_format(&format);
      audio_receiver_set_stream_type((audio_stream_type_t)stream_type);

      // Stop PTP (AirPlay 2 timing) to free socket slots for audio.
      // Audio stream will be started by the subsequent RECORD command.
      ptp_clock_stop();

      conn->stream_active = true;
      return;
    }

    ESP_LOGI(TAG, "SETUP: Initial connection setup (no streams)");

    uint32_t self_ip = 0;
    const bool have_self = rtsp_local_ipv4(socket, &self_ip);
    if (is_bplist && body) {
      session_setup_timing(conn, body, body_len, have_self ? self_ip : 0);
    }

    if (is_bplist && have_self) {
      // The receiver's own timing peer entry, as Apple's receiver returns it,
      // so the sender can put this device in its peer lists.
      char address[16];
      const uint8_t *b = (const uint8_t *)&self_ip;
      snprintf(address, sizeof(address), "%u.%u.%u.%u", b[0], b[1], b[2], b[3]);
      char peer_id[40];
      timing_peer_id(peer_id, sizeof(peer_id));
      uint8_t plist_body[256];
      size_t plist_len = bplist_build_session_setup(
          plist_body, sizeof(plist_body), conn->event_port, address, peer_id);
      if (plist_len == 0) {
        rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                           NULL, 0);
        return;
      }
      rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                         "Content-Type: application/x-apple-binary-plist\r\n",
                         (const char *)plist_body, plist_len);
    } else if (is_bplist) {
      uint8_t plist_body[128];
      size_t plist_len = bplist_build_initial_setup(
          plist_body, sizeof(plist_body), conn->event_port);
      if (plist_len == 0) {
        rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                           NULL, 0);
        return;
      }
      rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                         "Content-Type: application/x-apple-binary-plist\r\n",
                         (const char *)plist_body, plist_len);
    } else {
      rtsp_send_ok(socket, conn, req->cseq);
    }
    return;
  }

  // Stream SETUP
  int64_t stream_type = conn->stream_type;
  if (stream_type == 0) {
    stream_type = 96;
  }

  ESP_LOGI(TAG, "SETUP: Stream setup, stream_type=%lld",
           (long long)stream_type);

  bool buffered = audio_stream_uses_buffer((audio_stream_type_t)stream_type);
  if (buffered) {
    esp_err_t err = audio_receiver_start_buffered(0);
    if (err != ESP_OK) {
      rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                         NULL, 0);
      return;
    }
    conn->buffered_port = audio_receiver_get_buffered_port();
  }

  ensure_stream_ports(conn, buffered);

  uint16_t response_data_port =
      buffered ? conn->buffered_port : conn->data_port;

  if (!start_audio_receiver_or_fail(socket, conn, req, stream_type)) {
    return;
  }

  // Playout latency: the sender's, from the stream dict (audioLatencyMs,
  // else latencyMin), applied to both stream types as Apple does
  // (offset = sender latency - platform latency; the platform half is already
  // in the output instant the playback task schedules against).
  //  - REALTIME (type 96): the anchor maps an RTP timestamp onto the sender's
  //    source timeline and playout is expected latencyMin samples later.
  //    Playing at the anchor directly put this device ~250 ms AHEAD of a
  //    multi-room group (issue #54), so without a value the AirPlay default of
  //    11025 (250 ms) stands.
  //  - BUFFERED (type 103): 0 without a value, as on Apple.
  // The AirPlay 1 path keeps 0 -- no field evidence either way there.
  if (!is_bplist) {
    audio_receiver_set_playout_latency_samples(0);
  } else {
    int64_t sample_rate = conn->sample_rate > 0 ? conn->sample_rate : 44100;
    int64_t latency = 0;
    const char *latency_src = "default";
    bplist_kv_info_t *kv = calloc(32, sizeof(*kv));
    size_t kv_count = 0;
    if (kv && body &&
        bplist_get_stream_kv_info(body, body_len, 0, kv, 32, &kv_count)) {
      for (size_t k = 0; k < kv_count; k++) {
        if (kv[k].value_type == BPLIST_VALUE_INT &&
            strcmp(kv[k].key, "sr") == 0 && kv[k].int_value > 0) {
          sample_rate = kv[k].int_value;
        }
      }
      if (buffered && !conn->stream_keys_logged) {
        conn->stream_keys_logged = true;
        log_stream_keys(kv, kv_count, stream_type);
      }
      // A realtime latencyMin of 0 is read as none: issue #54's evidence is
      // that 250 ms is what the rest of a group plays at.
      if (!stream_sender_latency(kv, kv_count, sample_rate, &latency,
                                 &latency_src) ||
          (!buffered && latency == 0)) {
        latency = buffered ? 0 : AIRPLAY_RT_LATENCY_DEFAULT_SAMPLES;
        latency_src = "default";
      }
    } else {
      latency = buffered ? 0 : AIRPLAY_RT_LATENCY_DEFAULT_SAMPLES;
    }
    free(kv);
    if (buffered) {
      audio_receiver_set_buffered_latency_samples((uint32_t)latency);
    } else {
      audio_receiver_set_playout_latency_samples((uint32_t)latency);
    }
    ESP_LOGI(TAG, "%s playout latency: %lld samples (%lld ms, %s)",
             buffered ? "Buffered" : "Realtime", (long long)latency,
             (long long)(latency * 1000 / sample_rate), latency_src);
  }
  if (buffered) {
    rtsp_set_keepalive(socket, RTSP_IDLE_TIMEOUT_BUFFERED_S / 5);
  }

  if (is_bplist) {
    // Realtime only: how long after arrival a packet is rendered, which the
    // sender uses to decide how early realtime packets must arrive.  Apple
    // answers round((platform latency + I/O buffer) in ms) + 4, the 4 being
    // its default rasUserSpaceDelayMs; here, the modelled DMA ring plus the
    // device latency past it.
    int64_t arrival_to_render_ms = -1;
    if (stream_type == 96) {
      arrival_to_render_ms =
          (int64_t)((audio_output_get_platform_latency_us() + 500U) / 1000U) +
          4;
    }
    uint8_t plist_body[256];
    size_t plist_len = bplist_build_stream_setup(
        plist_body, sizeof(plist_body), stream_type, response_data_port,
        conn->control_port,
        stream_type == 103 ? AP2_AUDIO_BUFFER_SIZE
                           : AP2_REALTIME_AUDIO_BUFFER_SIZE,
        arrival_to_render_ms);
    if (plist_len == 0) {
      audio_receiver_stop();
      rtsp_send_response(socket, conn, 500, "Internal Error", req->cseq, NULL,
                         NULL, 0);
      return;
    }
    ESP_LOGI(TAG,
             "SETUP response: type=%lld dataPort=%u controlPort=%u "
             "arrivalToRenderLatencyMs=%lld",
             (long long)stream_type, response_data_port, conn->control_port,
             (long long)arrival_to_render_ms);
    rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                       "Content-Type: application/x-apple-binary-plist\r\n",
                       (const char *)plist_body, plist_len);
  } else {
    // AirPlay 1: Parse client's ports from Transport header
    rtsp_parse_transport((const char *)raw, &conn->client_control_port,
                         &conn->client_timing_port);
    ESP_LOGI(TAG, "Client ports: control=%u timing=%u",
             conn->client_control_port, conn->client_timing_port);

    if (!start_ntp_timing_or_fail(socket, conn, req)) {
      audio_receiver_stop();
      return;
    }

    char transport_response[256];
    snprintf(transport_response, sizeof(transport_response),
             "Transport: RTP/AVP/UDP;unicast;mode=record;"
             "server_port=%d;control_port=%d;timing_port=%d\r\n"
             "Session: 1\r\n",
             conn->data_port, conn->control_port, conn->timing_port);
    rtsp_send_response(socket, conn, 200, "OK", req->cseq, transport_response,
                       NULL, 0);
  }

  // Enable NACK retransmission if we know the client's control port
  if (conn->client_control_port > 0 && conn->client_ip != 0) {
    audio_receiver_set_client_control(conn->client_ip,
                                      conn->client_control_port);
  }

#ifdef CONFIG_BT_A2DP_ENABLE
  // Apply saved AirPlay volume before playback starts — the DAC may have
  // been left at a different level by Bluetooth A2DP.
  dac_set_volume(conn->volume_db);
#endif

  audio_receiver_set_playing(true);
  conn->stream_paused = false;
  conn->stream_active = true;
  // RECORD emits PLAYING on the initial connection only; a resume after a
  // stream TEARDOWN sends just a new SETUP, so emit it here too or the DAC
  // stays in the standby it entered on pause and the stream plays silent.
  playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_PLAYING, NULL);
}

static void handle_record(int socket, rtsp_conn_t *conn,
                          const rtsp_request_t *req, const uint8_t *raw,
                          size_t raw_len) {
  (void)raw;
  (void)raw_len;

  ESP_LOGI(TAG, "RECORD received - starting playback, stream_paused was %d",
           conn->stream_paused);

#ifdef CONFIG_BT_A2DP_ENABLE
  // Ensure DAC is at the saved AirPlay volume before any audio plays —
  // Bluetooth A2DP may have left it at a different level.
  dac_set_volume(conn->volume_db);
#endif

  if (conn->stream_paused) {
    // Resuming from PAUSE: the stream listener is still running and the
    // timing anchor has been preserved.  Just re-enable playout; the
    // pause-duration offset in audio_timing will re-align the timestamps.
    ESP_LOGI(TAG, "RECORD: resuming from pause, skipping stream restart");
    audio_receiver_set_playing(true);
  } else {
    // Fresh start or post-teardown reconnect: full stream restart.
    if (!start_audio_receiver_or_fail(socket, conn, req, conn->stream_type)) {
      return;
    }
    if (conn->client_control_port > 0 && conn->client_ip != 0) {
      audio_receiver_set_client_control(conn->client_ip,
                                        conn->client_control_port);
    }
    audio_receiver_set_playing(true);
  }
  conn->stream_paused = false;
  playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_PLAYING, NULL);

  // Apple reports the main audio session's platform latency here, in
  // samples, and compensates for it internally as well -- as this receiver
  // does, by scheduling every block that much early.  An Apple TV's buffered
  // session (the hose) answers 0; only realtime sessions report a figure,
  // and it leaves the I/O buffer out: the device latency past the DMA ring.
  // [I] Whether senders act on it is not known; the tvOS sender has no code
  // that reads it.  AirPlay 1 keeps the 0 it has always reported: nothing
  // says how a RAOP sender would read it.
  char headers[128];
  const int sample_rate = conn->sample_rate > 0 ? conn->sample_rate : 44100;
  const uint32_t latency_samples =
      conn->protocol_version == 2 && conn->stream_type == 96
          ? (uint32_t)(((uint64_t)audio_output_get_device_latency_us() *
                        (uint64_t)sample_rate) /
                       1000000ULL)
          : 0U;
  snprintf(headers, sizeof(headers),
           "Audio-Latency: %" PRIu32 "\r\n"
           "Audio-Jack-Status: connected\r\n",
           latency_samples);

  rtsp_send_response(socket, conn, 200, "OK", req->cseq, headers, NULL, 0);
}

// ============================================================================
// Metadata and Progress Logging Helpers
// ============================================================================

/**
 * Format time in seconds as mm:ss string
 * @param seconds Time in seconds
 * @param out Output buffer (at least 8 bytes for "999:59\0")
 * @param out_size Size of output buffer
 */
static void format_time_mmss(uint32_t seconds, char *out, size_t out_size) {
  uint32_t mins = seconds / 60;
  uint32_t secs = seconds % 60;
  snprintf(out, out_size, "%" PRIu32 ":%02" PRIu32, mins, secs);
}

/**
 * Parse DMAP-tagged data and extract metadata into struct
 * DMAP format: 4-byte tag, 4-byte BE length, data
 * Common tags:
 *   minm = item name (track title)
 *   asar = artist
 *   asal = album
 *   asgn = genre
 *   asai = album id (64-bit)
 */
#define DMAP_MAX_NESTING 8

static void parse_dmap_metadata(const uint8_t *data, size_t len,
                                playback_metadata_t *meta, int depth) {
  size_t pos = 0;

  if (depth > DMAP_MAX_NESTING) {
    return;
  }

  while (pos + 8 <= len) {
    // Read 4-byte tag
    char tag[5] = {0};
    memcpy(tag, data + pos, 4);
    pos += 4;

    // Read 4-byte big-endian length
    uint32_t item_len = ((uint32_t)data[pos] << 24) |
                        ((uint32_t)data[pos + 1] << 16) |
                        ((uint32_t)data[pos + 2] << 8) | data[pos + 3];
    pos += 4;

    // Subtraction, not pos + item_len: that addition wraps on 32-bit size_t
    // for a hostile length and would let a malformed frame read out of bounds.
    if (item_len > len - pos) {
      break; // Malformed
    }

    // Extract known metadata tags
    if (strcmp(tag, "minm") == 0 && item_len > 0) {
      size_t copy_len = item_len < METADATA_STRING_MAX - 1
                            ? item_len
                            : METADATA_STRING_MAX - 1;
      memcpy(meta->title, data + pos, copy_len);
      meta->title[copy_len] = '\0';
      ESP_LOGI(TAG, "  Title  = %s", meta->title);
    } else if (strcmp(tag, "asar") == 0 && item_len > 0) {
      size_t copy_len = item_len < METADATA_STRING_MAX - 1
                            ? item_len
                            : METADATA_STRING_MAX - 1;
      memcpy(meta->artist, data + pos, copy_len);
      meta->artist[copy_len] = '\0';
      ESP_LOGI(TAG, "  Artist = %s", meta->artist);
    } else if (strcmp(tag, "asal") == 0 && item_len > 0) {
      size_t copy_len = item_len < METADATA_STRING_MAX - 1
                            ? item_len
                            : METADATA_STRING_MAX - 1;
      memcpy(meta->album, data + pos, copy_len);
      meta->album[copy_len] = '\0';
      ESP_LOGI(TAG, "  Album  = %s", meta->album);
    } else if (strcmp(tag, "asgn") == 0 && item_len > 0) {
      size_t copy_len = item_len < METADATA_STRING_MAX - 1
                            ? item_len
                            : METADATA_STRING_MAX - 1;
      memcpy(meta->genre, data + pos, copy_len);
      meta->genre[copy_len] = '\0';
      ESP_LOGI(TAG, "  Genre  = %s", meta->genre);
    } else if (strcmp(tag, "mlit") == 0 || strcmp(tag, "cmst") == 0 ||
               strcmp(tag, "mdst") == 0) {
      // Container tags - recurse into them
      parse_dmap_metadata(data + pos, item_len, meta, depth + 1);
    }

    pos += item_len;
  }
}

/**
 * Parse progress string and populate metadata fields
 * Progress format: "start/current/end" in RTP timestamp units
 * Sample rate is typically 44100
 */
static void parse_progress(const char *progress_str, uint32_t sample_rate,
                           playback_metadata_t *meta) {
  uint64_t start = 0, current = 0, end = 0;

  // NOLINTNEXTLINE(bugprone-unchecked-string-to-number-conversion)
  if (sscanf(progress_str, "%" PRIu64 "/%" PRIu64 "/%" PRIu64, &start, &current,
             &end) == 3) {
    if (sample_rate == 0) {
      sample_rate = 44100; // Default sample rate
    }

    meta->position_secs = (uint32_t)((current - start) / sample_rate);
    meta->duration_secs = (uint32_t)((end - start) / sample_rate);

    char pos_str[16], dur_str[16];
    format_time_mmss(meta->position_secs, pos_str, sizeof(pos_str));
    format_time_mmss(meta->duration_secs, dur_str, sizeof(dur_str));

    ESP_LOGI(TAG,
             "Progress: %s / %s (raw: %" PRIu64 "/%" PRIu64 "/%" PRIu64 ")",
             pos_str, dur_str, start, current, end);
  }
}

static void handle_set_parameter(int socket, rtsp_conn_t *conn,
                                 const rtsp_request_t *req, const uint8_t *raw,
                                 size_t raw_len) {
  const uint8_t *body = req->body;
  size_t body_len = req->body_len;
  playback_event_data_t event_data;
  memset(&event_data, 0, sizeof(event_data));
  bool has_metadata = false;

  // Check for progress header in raw request
  const char *progress_hdr = strstr((const char *)raw, "progress:");
  if (!progress_hdr) {
    progress_hdr = strstr((const char *)raw, "Progress:");
  }
  if (progress_hdr) {
    // Find end of header line
    const char *line_end = strstr(progress_hdr, "\r\n");
    if (line_end) {
      size_t val_start = 9; // Length of "progress:"
      while (progress_hdr[val_start] == ' ') {
        val_start++;
      }
      char progress_val[64];
      size_t val_len = line_end - (progress_hdr + val_start);
      if (val_len < sizeof(progress_val)) {
        memcpy(progress_val, progress_hdr + val_start, val_len);
        progress_val[val_len] = '\0';
        parse_progress(progress_val, 44100, &event_data.metadata);
        has_metadata = true;
      }
    }
  }

  if (strstr(req->content_type, "text/parameters")) {
    if (body) {
      if (strstr((const char *)body, "volume:")) {
        const char *vol = strstr((const char *)body, "volume:");
        if (vol) {
          float volume = strtof(vol + 7, NULL);
          ESP_LOGI(TAG, "SET_PARAMETER: volume=%.2f dB", volume);
          rtsp_conn_set_volume(conn, volume);
        }
      }
      // Check for progress in body (AirPlay 1 style)
      const char *prog = strstr((const char *)body, "progress:");
      if (prog) {
        prog += 9;
        while (*prog == ' ') {
          prog++;
        }
        parse_progress(prog, 44100, &event_data.metadata);
        has_metadata = true;
      }
    }
  } else if (strstr(req->content_type, "application/x-dmap-tagged")) {
    // DMAP-tagged metadata (AirPlay 1)
    if (body && body_len > 0) {
      ESP_LOGI(TAG, "Received DMAP metadata (%zu bytes)", body_len);
      parse_dmap_metadata(body, body_len, &event_data.metadata, 0);
      has_metadata = true;
    }
  } else if (strstr(req->content_type, "image/jpeg") ||
             strstr(req->content_type, "image/png")) {
#ifdef CONFIG_ENABLE_AIRPLAY_ARTWORK
    // Artwork - log and flag in metadata
    ESP_LOGI(TAG, "Received artwork: %s (%zu bytes)", req->content_type,
             body_len);
    event_data.metadata.has_artwork = true;
    has_metadata = true;
#else
    // Artwork reception disabled — ignore it.  The md txt record already asks
    // senders not to transmit cover art, but some send it regardless.
    ESP_LOGD(TAG, "Ignoring artwork (%s, %zu bytes): disabled in config",
             req->content_type, body_len);
#endif
  } else if (strstr(req->content_type, "application/x-apple-binary-plist")) {
    if (body && body_len >= 8 && memcmp(body, "bplist00", 8) == 0) {
      int64_t value;
      if (bplist_find_int(body, body_len, "networkTimeSecs", &value)) {
        ESP_LOGI(TAG, "SET_PARAMETER: networkTimeSecs=%lld", (long long)value);
      }
      double rate;
      if (bplist_find_real(body, body_len, "rate", &rate)) {
        ESP_LOGI(TAG, "SET_PARAMETER: rate=%.2f", rate);
      }
      // Try to extract metadata from bplist (AirPlay 2)
      char str_val[METADATA_STRING_MAX];
      if (bplist_find_string(body, body_len, "itemName", str_val,
                             sizeof(str_val))) {
        ESP_LOGI(TAG, "Metadata: Title = %s", str_val);
        strlcpy(event_data.metadata.title, str_val, METADATA_STRING_MAX);
        has_metadata = true;
      }
      if (bplist_find_string(body, body_len, "artistName", str_val,
                             sizeof(str_val))) {
        ESP_LOGI(TAG, "Metadata: Artist = %s", str_val);
        strlcpy(event_data.metadata.artist, str_val, METADATA_STRING_MAX);
        has_metadata = true;
      }
      if (bplist_find_string(body, body_len, "albumName", str_val,
                             sizeof(str_val))) {
        ESP_LOGI(TAG, "Metadata: Album = %s", str_val);
        strlcpy(event_data.metadata.album, str_val, METADATA_STRING_MAX);
        has_metadata = true;
      }
      // Progress info from bplist
      double elapsed = 0, duration = 0;
      if (bplist_find_real(body, body_len, "elapsed", &elapsed)) {
        event_data.metadata.position_secs = (uint32_t)elapsed;
        has_metadata = true;
        char elapsed_str[16];
        format_time_mmss((uint32_t)elapsed, elapsed_str, sizeof(elapsed_str));
        if (bplist_find_real(body, body_len, "duration", &duration)) {
          event_data.metadata.duration_secs = (uint32_t)duration;
          char duration_str[16];
          format_time_mmss((uint32_t)duration, duration_str,
                           sizeof(duration_str));
          ESP_LOGI(TAG, "Progress: %s / %s", elapsed_str, duration_str);
        } else {
          ESP_LOGI(TAG, "Progress: %s", elapsed_str);
        }
      }
    }
  }

  if (has_metadata) {
    playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_METADATA,
                         &event_data);
  }

  rtsp_send_ok(socket, conn, req->cseq);
}

static void handle_get_parameter(int socket, rtsp_conn_t *conn,
                                 const rtsp_request_t *req, const uint8_t *raw,
                                 size_t raw_len) {
  (void)raw;
  (void)raw_len;

  if (req->body && req->body_len > 0) {
    if (strstr((const char *)req->body, "volume")) {
      char vol_response[32];
      int vol_len = snprintf(vol_response, sizeof(vol_response),
                             "volume: %.2f\r\n", conn->volume_db);
      rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                         "Content-Type: text/parameters\r\n", vol_response,
                         vol_len);
      return;
    }
  }

  rtsp_send_ok(socket, conn, req->cseq);
}

static void handle_pause(int socket, rtsp_conn_t *conn,
                         const rtsp_request_t *req, const uint8_t *raw,
                         size_t raw_len) {
  (void)raw;
  (void)raw_len;

  ESP_LOGI(TAG, "PAUSE received");

  // Stop the audio consumer but leave the buffer filling.  The phone will
  // send a fresh SETRATEANCHORTIME (rate=1) anchor on resume that re-aligns
  // the buffered frames to the correct wall-clock position.
  audio_receiver_pause();
  audio_output_flush();
  conn->stream_paused = true;

  rtsp_send_ok(socket, conn, req->cseq);
}

// The flush point from a FLUSH's "RTP-Info: seq=...;rtptime=..." header.
static bool parse_flush_rtptime(const uint8_t *raw, size_t raw_len,
                                uint32_t *rtptime) {
  static const char key[] = "rtptime=";
  const size_t key_len = sizeof(key) - 1;
  if (!raw) {
    return false;
  }
  const uint8_t *end = rtsp_find_header_end(raw, raw_len);
  size_t len = end ? (size_t)(end - raw) : raw_len;
  for (size_t i = 0; i + key_len < len; i++) {
    if (strncasecmp((const char *)raw + i, key, key_len) != 0) {
      continue;
    }
    uint64_t value = 0;
    size_t j = i + key_len;
    if (j >= len || raw[j] < '0' || raw[j] > '9') {
      return false;
    }
    for (; j < len && raw[j] >= '0' && raw[j] <= '9'; j++) {
      value = value * 10 + (uint64_t)(raw[j] - '0');
      if (value > UINT32_MAX) {
        return false;
      }
    }
    *rtptime = (uint32_t)value;
    return true;
  }
  return false;
}

static void handle_flush(int socket, rtsp_conn_t *conn,
                         const rtsp_request_t *req, const uint8_t *raw,
                         size_t raw_len) {
  // Plain FLUSH: AirPlay 1, and AirPlay 2 realtime (macOS).  Always immediate.
  uint32_t until_rtp = 0;
  bool until_valid = parse_flush_rtptime(raw, raw_len, &until_rtp);
  if (until_valid) {
    ESP_LOGI(TAG, "FLUSH received (rtptime=%" PRIu32 ")", until_rtp);
  } else {
    ESP_LOGI(TAG, "FLUSH received");
  }
  audio_receiver_realtime_flush(until_valid, until_rtp);
  audio_output_flush();
  rtsp_send_ok(socket, conn, req->cseq);
}

static void handle_flushbuffered(int socket, rtsp_conn_t *conn,
                                 const rtsp_request_t *req, const uint8_t *raw,
                                 size_t raw_len) {
  (void)raw;
  (void)raw_len;

  const int64_t sample_rate = conn->sample_rate > 0 ? conn->sample_rate : 44100;

  // AirPlay 2 FLUSHBUFFERED, read as Apple's receiver reads it:
  //   flushUntilSeq / flushUntilTS (required)
  //   flushFromSeq / flushFromTS (both present: a deferred, within-range
  //   flush of [from, until); otherwise everything before until, now).
  // Sequence numbers decide; the timestamps are logged.
  audio_flush_record_t record;
  if (!rtsp_parse_flush_buffered(req->body, req->body_len, sample_rate,
                                 &record)) {
    ESP_LOGW(TAG, "FLUSHBUFFERED without flushUntilSeq/TS: refused");
    rtsp_send_response(socket, conn, 400, "Bad Request", req->cseq, NULL, NULL,
                       0);
    return;
  }

  char headers[64] = "";
  if (conn->stream_type == 103) {
    uint32_t rtp = 0;
    if (audio_receiver_flush_buffered(&record, &rtp)) {
      // The RTP of the last packet handed to the output, as Apple replies.
      snprintf(headers, sizeof(headers), "RTP-Info: rtptime=%" PRIu32 "\r\n",
               rtp);
    }
  } else {
    // Not a buffered stream: what FLUSH does.
    audio_receiver_realtime_flush(false, 0);
  }
  if (!record.has_from) {
    audio_output_flush();
  }

  rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                     headers[0] ? headers : NULL, NULL, 0);
}

static void handle_teardown(int socket, rtsp_conn_t *conn,
                            const rtsp_request_t *req, const uint8_t *raw,
                            size_t raw_len) {
  (void)raw;
  (void)raw_len;

  const uint8_t *body = req->body;
  size_t body_len = req->body_len;
  bool has_streams = false;
  size_t stream_count = 0;

  if (body && body_len >= 8 && memcmp(body, "bplist00", 8) == 0) {
    if (bplist_get_streams_count(body, body_len, &stream_count)) {
      has_streams = true;
    }
  }

  // TEARDOWN with streams = stream teardown (may be followed by new SETUP)
  // TEARDOWN without streams = full session teardown (disconnect)
  ESP_LOGI(TAG, "TEARDOWN: has_streams=%d stream_count=%zu", has_streams,
           stream_count);
  // Stream-level teardown is a pause: freeze playout immediately so audio
  // silences on ALL boards. playing=false makes the output emit silence at
  // once (software mute, for software-volume/DAC-less boards); the synchronous
  // PAUSED event mutes the hardware DAC.  Both run ahead of the slower
  // receiver/decoder teardown below (audio_receiver_stop can block ~1 s
  // waiting for the listener task to exit).
  if (has_streams) {
    audio_receiver_set_playing(false);
    settings_persist_volume();
    playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_PAUSED, NULL);
  }
  audio_receiver_stop();
  audio_output_flush();
  // A stream TEARDOWN leaves the clock alone: Apple's receiver starts it at
  // the session SETUP and stops it only when the session ends, so a phone
  // moving to its next track (TEARDOWN + SETUP within half a second) or
  // resuming a pause finds it still locked.
  conn->stream_active = false;
  // Without a buffered stream the session and keepalive limits drop back.
  rtsp_set_keepalive(socket, RTSP_IDLE_TIMEOUT_S / 5);
  conn->stream_paused =
      has_streams; // Keep session ready if only streams torn down

  if (!has_streams) {
    // Full teardown — server cleanup will emit PLAYBACK_EVENT_DISCONNECTED
    // when the TCP connection closes.
    // Release the audio path now rather than behind the grace period below:
    // the session is over, so anything else waiting for the speaker can have
    // it. A stream-level teardown is a pause and deliberately keeps the claim.
    audio_receiver_end_session();
    // For v1 sessions, keep the DACP session alive across teardown so the
    // grace period can probe mDNS to differentiate pause from real
    // disconnect. v2 sessions clear immediately.
    if (conn->protocol_version != 1) {
      dacp_clear_session();
      conn->dacp_id[0] = '\0';
      conn->active_remote[0] = '\0';
    }
    ntp_clock_stop();
    rtsp_session_end_clock();
    conn->timing_port = 0;
  }

  rtsp_send_ok(socket, conn, req->cseq);
}

// Refuse a SETRATEANCHORTIME the way Apple's receiver does: the RTSP request
// succeeds, and the plist says why it was not applied.
static void reply_status(int socket, rtsp_conn_t *conn,
                         const rtsp_request_t *req, int64_t status) {
  uint8_t plist[64];
  const size_t len = bplist_build_status(plist, sizeof(plist), status);
  if (len == 0) {
    rtsp_send_ok(socket, conn, req->cseq);
    return;
  }
  rtsp_send_response(socket, conn, 200, "OK", req->cseq,
                     "Content-Type: application/x-apple-binary-plist\r\n",
                     (const char *)plist, len);
}

static void handle_setrateanchortime(int socket, rtsp_conn_t *conn,
                                     const rtsp_request_t *req,
                                     const uint8_t *raw, size_t raw_len) {
  (void)raw;
  (void)raw_len;

  const int64_t sample_rate = conn->sample_rate > 0 ? conn->sample_rate : 44100;
  audio_rate_anchor_t anchor;
  int64_t status = 0;
  if (!rtsp_parse_rate_anchor(req->body, req->body_len, sample_rate, &anchor,
                              &status)) {
    ESP_LOGW(TAG, "SETRATEANCHORTIME refused: status %lld", (long long)status);
    reply_status(socket, conn, req, status);
    return;
  }
  ESP_LOGI(TAG,
           "SETRATEANCHORTIME: rate=%" PRIu32 " rtp=%" PRIu32
           " net=%llu.%09llu timeline=%016llx flags=0x%" PRIx32
           " fas=%s%" PRIu32,
           anchor.rate, anchor.rtp,
           (unsigned long long)(anchor.network_ns / 1000000000ULL),
           (unsigned long long)(anchor.network_ns % 1000000000ULL),
           (unsigned long long)anchor.timeline_id, anchor.network_flags,
           anchor.fas_valid ? "" : "n/a:", anchor.fas_rtp);

  if (anchor.rate == 0U) {
    // Mute the DAC first via the synchronous event so audio stops now.
    playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_PAUSED, NULL);
    conn->stream_paused = true;
    audio_receiver_set_rate_anchor(&anchor);
    audio_output_flush();
    settings_persist_volume();
  } else {
    conn->stream_paused = false;
    if (conn->stream_type == 96) {
      // A realtime stream keeps its own anchor path (the control channel
      // re-anchors it every second).
      audio_receiver_set_anchor_time(anchor.timeline_id, anchor.network_ns,
                                     anchor.rtp);
      audio_receiver_set_playing(true);
    } else {
      audio_receiver_set_rate_anchor(&anchor);
    }
    playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_PLAYING, NULL);
  }

  // A plain 200 on success: Apple's reply carries nothing either.
  rtsp_send_ok(socket, conn, req->cseq);
}

static void handle_setpeers(int socket, rtsp_conn_t *conn,
                            const rtsp_request_t *req, const uint8_t *raw,
                            size_t raw_len) {
  (void)raw;
  (void)raw_len;

  // SETPEERS carries an array of address strings, SETPEERSX an array of peer
  // dicts; both replace the whole peer list.  Apple applies them only to a
  // PTP session and ignores them otherwise.  The anchor timing is not reset:
  // the anchor names its timeline, and the playback task follows a
  // grandmaster change by re-expressing the anchor on the new one.
  if (!conn->timing_ptp) {
    ESP_LOGI(TAG, "%s ignored: not a PTP session", req->method);
    rtsp_send_ok(socket, conn, req->cseq);
    return;
  }
  uint32_t self_ip = 0;
  (void)rtsp_local_ipv4(socket, &self_ip);
  uint32_t peers[AIRPLAY_MAX_TIMING_PEERS];
  const size_t count = collect_timing_peers(
      req->body, req->body_len, NULL, self_ip, peers, AIRPLAY_MAX_TIMING_PEERS);
  log_timing_peers(req->method, peers, count);
  ptp_clock_set_peers(peers, count);

  rtsp_send_ok(socket, conn, req->cseq);
}

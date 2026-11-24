// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtsp_server.h"

#include <errno.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "audio_receiver.h"
#include "audio_output.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "rtsp_conn.h"
#include "rtsp_crypto.h"
#include "rtsp_handlers.h"
#include "rtsp_message.h"
#include "rtsp_rsa.h"

#include "rtsp_events.h"
#include "rtsp_metadata.h"
#include "rtsp_policy.h"

#include "ntp_clock.h"
#include "ptp_clock.h"
#include "playback_events.h"
#include "plist.h"
#include "settings.h"
#include "dacp_client.h"

static const char *TAG = "rtsp_server";

#define RTSP_BUFFER_INITIAL 4096
#define RTSP_BUFFER_LARGE   ((size_t)256 * 1024)

#define CLIENT_STACK_SIZE 8192
#define SERVER_STACK_SIZE 4096

static int server_socket = -1;
static TaskHandle_t server_task_handle = NULL;
static bool server_running = false;

// RTSP tasks are restartable. Use dynamic TCB allocation so
// reconnect/start-stop paths cannot reuse static task memory before FreeRTOS
// idle finishes deletion.

// Client slot for tracking connections
typedef struct {
  rtsp_conn_t *conn;
  TaskHandle_t task;
  int socket;
  volatile bool should_stop;
  volatile bool is_old; // Its session was taken over by another connection
  // When the last request was dispatched (any method, /feedback included),
  // for the session idle timeout, and when the status line last went out.
  int64_t last_request_us;
  int64_t last_status_us;
} client_slot_t;

// One status line for the session/audio side every this often, from the RTSP
// task (never the playback task).
#define SESSION_STATUS_INTERVAL_US 10000000LL

// Connections served at once.  Apple's receiver serves any number, each with
// its own request processor; a second phone's /info or pairing, or a probe,
// runs beside the playing session and never touches it.  Three covers the
// session plus two such visitors; each costs a task stack in internal RAM
// only while it is open.
#define RTSP_MAX_CLIENTS 3

// A connection that does not own the session and has sent nothing for this
// long is closed, as Apple's HTTP server sweeps idle connections every 90 s.
#define RTSP_VISITOR_IDLE_US (90LL * 1000000LL)

// How long a takeover waits for the old connection's task to finish.
#define RTSP_TAKEOVER_WAIT_MS 3000

static client_slot_t clients[RTSP_MAX_CLIENTS] = {0};

// The slot whose connection owns the AirPlay session (did the session SETUP,
// or ANNOUNCE for AirPlay 1), or -1.  Written under s_owner_lock; the
// playback task reads it bare for the volume, as it read the old "current
// slot".
static volatile int s_owner = -1;
static SemaphoreHandle_t s_owner_lock;

// Flag set by the play/pause button to tell the grace period loop
// to send a DACP resume command and keep waiting for reconnect.
static volatile bool s_resume_requested = false;

static void owner_lock(void) {
  xSemaphoreTake(s_owner_lock, portMAX_DELAY);
}

static void owner_unlock(void) {
  xSemaphoreGive(s_owner_lock);
}

static rtsp_conn_t *owner_conn(void) {
  const int owner = s_owner;
  if (owner < 0 || owner >= RTSP_MAX_CLIENTS || clients[owner].is_old) {
    return NULL;
  }
  return clients[owner].conn;
}

// Public API for volume control
void airplay_set_volume(float volume_db) {
  if (!s_owner_lock) {
    return;
  }
  owner_lock();
  rtsp_conn_t *c = owner_conn();
  if (c) {
    rtsp_conn_set_volume(c, volume_db);
  }
  owner_unlock();
}

int32_t airplay_get_volume_q15(void) {
  // The playback task: no lock.  A connection is freed only after its slot
  // has given up ownership.
  const rtsp_conn_t *c = owner_conn();
  if (c) {
    return c->volume_q15;
  }
  return 16384; // 50% volume for new clients
}

void rtsp_server_request_resume(void) {
  s_resume_requested = true;
}

uint16_t airplay_rtsp_port(void) {
  return settings_airplay_v1() ? 5000 : 7000;
}

// Helper to grow buffer
static uint8_t *grow_buffer(uint8_t *old_buf, size_t old_size, size_t new_size,
                            size_t data_len) {
  (void)old_size;
  uint8_t *new_buf =
      heap_caps_malloc(new_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!new_buf) {
    new_buf = malloc(new_size);
  }
  if (!new_buf) {
    return NULL;
  }
  if (old_buf && data_len > 0) {
    memcpy(new_buf, old_buf, data_len);
  }
  free(old_buf);
  return new_buf;
}

// Process buffered RTSP requests
static void process_rtsp_buffer(client_slot_t *slot, uint8_t *buffer,
                                size_t *buf_len) {
  while (*buf_len > 0 && !slot->should_stop) {
    const uint8_t *header_end = rtsp_find_header_end(buffer, *buf_len);
    if (!header_end) {
      break;
    }

    size_t header_len = (size_t)(header_end - buffer) + 4;
    char *header_str = malloc(header_len + 1);
    if (!header_str) {
      *buf_len = 0;
      break;
    }
    memcpy(header_str, buffer, header_len);
    header_str[header_len] = '\0';

    int content_len = rtsp_parse_content_length(header_str);
    if (content_len < 0) {
      content_len = 0;
    }

    size_t total_len = header_len + (size_t)content_len;
    if (total_len > RTSP_BUFFER_LARGE || *buf_len < total_len) {
      free(header_str);
      if (total_len > RTSP_BUFFER_LARGE) {
        *buf_len = 0;
      }
      break;
    }

    // Null-terminate so strcasestr in parse_raw_header won't read past
    // the message boundary (buffer capacity > total_len).
    uint8_t saved = buffer[total_len];
    buffer[total_len] = '\0';
    slot->last_request_us = esp_timer_get_time();
    rtsp_dispatch(slot->socket, slot->conn, buffer, total_len);
    buffer[total_len] = saved;
    free(header_str);

    if (*buf_len > total_len) {
      memmove(buffer, buffer + total_len, *buf_len - total_len);
    }
    *buf_len -= total_len;
  }
}

static void log_session_status(const client_slot_t *slot, int64_t now_us) {
  audio_receiver_diag_t d;
  audio_receiver_get_diag(&d);
  ESP_LOGI(TAG,
           "session: timing=%s idle=%llds/%ds state=%s err=%" PRId32
           "us anchors installed=%" PRIu32 " same=%" PRIu32 " reset=%" PRIu32
           " waited=%" PRIu32 " gm_rebases=%" PRIu32 " (failed %" PRIu32
           ") freewheel=%" PRIu32 "%s steps=%" PRIu32
           " (clock-deferred %" PRIu32 ") trims=%" PRIu32
           " flush pending=%" PRIu32 " dropped=%" PRIu32 " blocks=%" PRIu32,
           slot->conn->timing_ptp ? "PTP" : "NTP",
           (long long)((now_us - slot->last_request_us) / 1000000LL),
           rtsp_session_idle_limit_s(slot->conn), d.state, d.error_us,
           d.anchors_installed, d.anchors_consistent, d.anchor_resets,
           d.anchors_deferred, d.gm_rebases, d.gm_rebase_failures, d.freewheels,
           d.freewheeling ? " (now)" : "", d.steps, d.step_clock_deferrals,
           d.trims, d.flushes_pending, d.flush_dropped, d.buffered_blocks);
}

static bool slot_owns_session(const client_slot_t *slot) {
  const int owner = s_owner;
  return owner >= 0 && &clients[owner] == slot && !slot->is_old;
}

// Once a second or so, between reads: the session idle timeout and the
// status line.  False when the connection should close.  Apple's receiver
// terminates a session that has gone idle for too long, and so does this one,
// through the same cleanup as a closed connection.  Only an AirPlay 2 session
// that has done its session SETUP is timed: an AirPlay 1 sender has no
// /feedback to keep it alive.  A connection that holds no session (an /info
// probe, another phone's pairing) is swept after 90 s of silence instead.
static bool client_tick(client_slot_t *slot) {
  const rtsp_conn_t *conn = slot->conn;
  const int64_t now_us = esp_timer_get_time();
  if (!conn) {
    return true;
  }
  if (!slot_owns_session(slot)) {
    if (now_us - slot->last_request_us > RTSP_VISITOR_IDLE_US) {
      ESP_LOGI(TAG, "Closing idle connection (no session)");
      return false;
    }
    return true;
  }
  if (conn->protocol_version != 2 || conn->event_port == 0) {
    return true;
  }
  const int limit_s = rtsp_session_idle_limit_s(conn);
  if (now_us - slot->last_request_us > (int64_t)limit_s * 1000000LL) {
    ESP_LOGW(TAG, "Idle timeout after %d seconds with no activity", limit_s);
    return false;
  }
  if (now_us - slot->last_status_us >= SESSION_STATUS_INTERVAL_US) {
    slot->last_status_us = now_us;
    log_session_status(slot, now_us);
  }
  return true;
}

// The connection that owns the session is gone: everything the session
// started stops, and listeners hear about it.  An AirPlay 1 sender with a
// DACP remote gets a grace period first, during which the slot keeps the
// session (owner, socket closed) so a reconnect or a resume button can find
// it.
static void end_owned_session(client_slot_t *slot, rtsp_conn_t *conn) {
  // Immediate: stop audio and NTP.  The socket is gone, so the claim on the
  // audio path goes with it; the DACP grace period below only decides when to
  // report a disconnect.
  audio_receiver_stop();
  audio_receiver_end_session();
  audio_output_flush();
  ntp_clock_stop();

  bool has_dacp_remote = conn && conn->protocol_version == 1 &&
                         conn->dacp_id[0] != '\0' &&
                         conn->active_remote[0] != '\0';

  // iOS v1 pause handling needs DACP to distinguish pause from disconnect.
  // Third-party RAOP clients often have no DACP remote; disconnect them
  // immediately instead of delaying slot cleanup with an iOS-only grace path.
  if (has_dacp_remote && !slot->should_stop) {
    s_resume_requested = false;
    playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_PAUSED, NULL);

    // Phase 1: let mDNS settle (3 s), but exit early on resume or reconnect
    for (int i = 0; i < 6 && !slot->should_stop; i++) {
      vTaskDelay(pdMS_TO_TICKS(500));
      if (s_resume_requested) {
        ESP_LOGI(TAG, "Resume requested during Phase 1 — skipping to Phase 2");
        break;
      }
    }

    // Phase 2: wait for reconnect as long as DACP service is advertised.
    // Re-probe every ~5 s. The phone unadvertises the service when the
    // user switches away, so disappearance = genuine disconnect.
    if (!slot->should_stop) {
      bool stay = dacp_probe_service() || s_resume_requested;
      if (s_resume_requested) {
        s_resume_requested = false;
        ESP_LOGI(TAG, "Resume requested via button — waiting for reconnect");
        stay = true;
      }
      if (stay) {
        ESP_LOGI(TAG, "DACP still advertised — waiting for reconnect");
      }
      while (stay && !slot->should_stop) {
        // Wait 5 s between probes (10 × 500 ms), checking flags each tick
        for (int i = 0; i < 10 && !slot->should_stop; i++) {
          vTaskDelay(pdMS_TO_TICKS(500));
          if (s_resume_requested) {
            s_resume_requested = false;
            ESP_LOGI(TAG,
                     "Resume requested via button — extending grace period");
          }
        }
        if (slot->should_stop) {
          break;
        }
        // Re-probe: still advertised?
        stay = dacp_probe_service();
        if (stay) {
          ESP_LOGD(TAG, "DACP still advertised — continuing wait");
        } else {
          ESP_LOGI(TAG, "DACP service gone — genuine disconnect");
        }
      }
    }

    if (slot->is_old) {
      // The sender came back with a new connection: it owns the session now.
      ESP_LOGI(TAG, "Client reconnected during grace period");
      return;
    }
    ESP_LOGI(TAG, "Grace period over — full disconnect");
  }

  dacp_clear_session();
  playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_DISCONNECTED,
                       NULL);
  rtsp_metadata_reset();
  // The session is over: its timing peers and PTP session go with it.
  rtsp_session_end_clock();
  ptp_clock_init(); // Restart PTP (stopped during v1 SETUP to free sockets)
  rtsp_stop_event_port_task();
}

// Client task
static void client_task(void *pvParameters) {
  int slot_idx = (int)(intptr_t)pvParameters;
  client_slot_t *slot = &clients[slot_idx];

  // Create connection state
  rtsp_conn_t *conn = rtsp_conn_create();
  if (!conn) {
    ESP_LOGE(TAG, "Failed to create connection state");
    close(slot->socket);
    slot->socket = -1;
    slot->task = NULL;
    vTaskDelete(NULL);
    return;
  }
  slot->conn = conn;

  // Get client IP address for timing requests
  struct sockaddr_in peer_addr;
  socklen_t peer_len = sizeof(peer_addr);
  if (getpeername(slot->socket, (struct sockaddr *)&peer_addr, &peer_len) ==
      0) {
    conn->client_ip = peer_addr.sin_addr.s_addr;
    ESP_LOGI(TAG, "Client IP: %u.%u.%u.%u",
             (unsigned int)(conn->client_ip & 0xFF),
             (unsigned int)((conn->client_ip >> 8) & 0xFF),
             (unsigned int)((conn->client_ip >> 16) & 0xFF),
             (unsigned int)((conn->client_ip >> 24) & 0xFF));
  }

  // Allocate buffer
  size_t buf_capacity = RTSP_BUFFER_INITIAL;
  uint8_t *buffer = malloc(buf_capacity);
  if (!buffer) {
    ESP_LOGE(TAG, "Failed to allocate buffer");
    rtsp_conn_free(conn);
    slot->conn = NULL;
    close(slot->socket);
    slot->socket = -1;
    slot->task = NULL;
    vTaskDelete(NULL);
    return;
  }

  size_t buf_len = 0;

  // Socket timeout for stop signal responsiveness
  struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
  setsockopt(slot->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Disable Nagle: RTSP control commands (volume, pause) are tiny and must
  // not wait for coalescing/delayed-ACK, which adds tens to hundreds of ms of
  // latency to every command on this connection.
  int nodelay = 1;
  setsockopt(slot->socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

  // A peer that vanished (phone off Wi-Fi) is noticed by TCP keepalive; a
  // buffered stream lengthens it (see handle_setup()).
  rtsp_set_keepalive(slot->socket, rtsp_session_idle_limit_s(conn) / 5);
  slot->last_request_us = esp_timer_get_time();
  slot->last_status_us = slot->last_request_us;

  while (server_running && !slot->should_stop) {
    if (conn->encrypted_mode) {
      // Encrypted mode
      while (server_running && conn->encrypted_mode && !slot->should_stop) {
        if (buf_len >= buf_capacity - 1024) {
          size_t new_cap = buf_capacity < RTSP_BUFFER_LARGE ? RTSP_BUFFER_LARGE
                                                            : buf_capacity * 2;
          if (new_cap > RTSP_BUFFER_LARGE) {
            goto cleanup;
          }
          uint8_t *new_buf =
              grow_buffer(buffer, buf_capacity, new_cap, buf_len);
          if (!new_buf) {
            goto cleanup;
          }
          buffer = new_buf;
          buf_capacity = new_cap;
        }

        int block_len = rtsp_crypto_read_block(
            slot->socket, conn, buffer + buf_len, buf_capacity - buf_len);
        if (!client_tick(slot)) {
          goto cleanup;
        }
        if (block_len <= 0) {
          if (slot->should_stop || (errno != EAGAIN && errno != EWOULDBLOCK)) {
            goto cleanup;
          }
          continue;
        }

        buf_len += (size_t)block_len;
        process_rtsp_buffer(slot, buffer, &buf_len);
      }
      goto cleanup;
    }

    // Plain-text mode
    if (buf_len >= buf_capacity - 1024) {
      size_t new_cap = buf_capacity < RTSP_BUFFER_LARGE ? RTSP_BUFFER_LARGE
                                                        : buf_capacity * 2;
      if (new_cap > RTSP_BUFFER_LARGE) {
        break;
      }
      uint8_t *new_buf = grow_buffer(buffer, buf_capacity, new_cap, buf_len);
      if (!new_buf) {
        break;
      }
      buffer = new_buf;
      buf_capacity = new_cap;
    }

    ssize_t recv_len =
        recv(slot->socket, buffer + buf_len, buf_capacity - buf_len, 0);
    if (!client_tick(slot)) {
      break;
    }
    if (recv_len <= 0) {
      if (recv_len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        continue;
      }
      break;
    }
    buf_len += (size_t)recv_len;
    process_rtsp_buffer(slot, buffer, &buf_len);
  }

cleanup:
  ESP_LOGI(TAG, "Client slot %d disconnected", slot_idx);
  free(buffer);
  close(slot->socket);
  slot->socket = -1;

  if (slot_owns_session(slot)) {
    end_owned_session(slot, conn);
  } else if (slot->is_old) {
    // Taken over: rtsp_server_claim_session() on the new connection is
    // waiting for this task to finish, and then stops this session's audio
    // and tells the listeners.  Touching anything global here would hit the
    // new session.
    ESP_LOGI(TAG, "Slot %d: session taken over by another sender", slot_idx);
  }
  // A connection that never owned the session (/info, pairing, a refused
  // SETUP) has nothing global to undo either.

  if (conn->event_socket >= 0 &&
      rtsp_event_port_listen_socket() == conn->event_socket) {
    // Still serving this connection's event port: stop it before closing.
    rtsp_stop_event_port_task();
  }
  if (conn->event_socket >= 0) {
    close(conn->event_socket);
    conn->event_socket = -1;
  }

  owner_lock();
  if (s_owner == slot_idx) {
    s_owner = -1;
  }
  // Out of reach of airplay_get_volume_q15() before it is freed.
  slot->conn = NULL;
  owner_unlock();

  rtsp_conn_cleanup(conn);
  rtsp_conn_free(conn);

  slot->socket = -1;
  slot->task = NULL;
  slot->should_stop = false;
  slot->is_old = false;

  vTaskDelete(NULL);
}

// Ask a slot's task to finish: it sees should_stop, or a recv that fails
// once the socket is shut down.  Non-blocking.
static void signal_client_stop(client_slot_t *slot, bool taken_over) {
  if (slot->task == NULL) {
    return;
  }
  if (taken_over) {
    slot->is_old = true;
  }
  slot->should_stop = true;
  if (slot->socket >= 0) {
    shutdown(slot->socket, SHUT_RDWR);
  }
}

// Wait for the task the slot has now to finish.  Compared by handle, so a new
// connection that lands in the freed slot meanwhile is not waited for too.
static bool wait_client_stopped(const client_slot_t *slot, int timeout_ms) {
  const TaskHandle_t task = slot->task;
  for (int waited = 0;
       task != NULL && slot->task == task && waited < timeout_ms;
       waited += 100) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  return task == NULL || slot->task != task;
}

static int slot_of(const rtsp_conn_t *conn) {
  for (int i = 0; i < RTSP_MAX_CLIENTS; i++) {
    if (clients[i].conn == conn && clients[i].task != NULL) {
      return i;
    }
  }
  return -1;
}

int rtsp_server_claim_session(rtsp_conn_t *conn, bool remote_control_only) {
  if (!conn || !s_owner_lock) {
    return 500;
  }
  owner_lock();
  const int self = slot_of(conn);
  const int owner = s_owner;
  const bool has_owner = owner >= 0 && clients[owner].task != NULL;
  const rtsp_preempt_input_t in = {
      // A connection being closed (taken over, or swept) may not claim the
      // session back while it winds down.
      .self_closing =
          self < 0 || clients[self].should_stop || clients[self].is_old,
      .remote_control_only = remote_control_only,
      .has_owner = has_owner,
      .owner_is_self = has_owner && owner == self,
      .owner_closed = has_owner && clients[owner].socket < 0,
      .owner_ended = has_owner && clients[owner].conn &&
                     clients[owner].conn->session_ended,
      .same_sender = has_owner && clients[owner].conn &&
                     clients[owner].conn->client_ip == conn->client_ip,
      .deny_interruptions = settings_airplay_deny_interruptions(),
  };
  const rtsp_preempt_t decision = rtsp_preempt_decide(&in);
  const int status = rtsp_preempt_status(decision);
  if (status != 0) {
    owner_unlock();
    if (decision == RTSP_PREEMPT_REFUSE) {
      ESP_LOGW(TAG, "SETUP refused: the session may not be interrupted");
    } else if (decision == RTSP_PREEMPT_REMOTE_ONLY) {
      ESP_LOGW(TAG, "Remote-control-only SETUP refused: not supported");
    } else {
      ESP_LOGW(TAG, "SETUP on a connection being closed: %d", status);
    }
    return status;
  }
  if (decision == RTSP_PREEMPT_PROCEED) {
    if (!in.owner_is_self) {
      ESP_LOGI(TAG, "Slot %d owns the session", self);
      rtsp_metadata_reset();
    }
    s_owner = self;
    conn->owns_session = true;
    conn->session_ended = false;
    owner_unlock();
    return 0;
  }

  // Takeover: the last sender wins, as on Apple's receiver, which destroys
  // the old connection before the new session is set up.  From here the old
  // connection no longer owns the session, so any session method it still
  // sends gets 455 and a SETUP gets 455 above.  But a request it had already
  // begun may still be running on its task, so the old session's audio is
  // stopped only once that task has finished: stopped any earlier, that
  // request could start or re-anchor audio again behind this SETUP's back.
  client_slot_t *old = &clients[owner];
  rtsp_conn_t *old_conn = old->conn;
  const bool was_live = !in.owner_closed;
  if (old_conn) {
    old_conn->owns_session = false;
  }
  signal_client_stop(old, true);
  s_owner = self;
  conn->owns_session = true;
  conn->session_ended = false;
  owner_unlock();

  ESP_LOGI(TAG, "*** Session taken over: slot %d replaces slot %d%s", self,
           owner, was_live ? "" : " (closed)");
  // This task holds no lock while it waits, and the old task never waits on
  // this one, so the wait cannot deadlock; at worst it times out.
  if (!wait_client_stopped(old, RTSP_TAKEOVER_WAIT_MS)) {
    ESP_LOGW(TAG, "Slot %d still finishing; carrying on", owner);
  }

  // A third sender may have taken the session from this connection while it
  // waited.  It then does the cleanup below, after this task has finished.
  owner_lock();
  const bool still_owner =
      s_owner == self && !clients[self].is_old && !clients[self].should_stop;
  owner_unlock();
  if (!still_owner) {
    ESP_LOGW(TAG, "Slot %d lost the session while taking it over", self);
    return 455;
  }

  rtsp_metadata_reset();
  if (was_live) {
    audio_receiver_stop();
    audio_output_flush();
    ntp_clock_stop();
    rtsp_stop_event_port_task();
    dacp_clear_session();
    playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_DISCONNECTED,
                         NULL);
  }
  return 0;
}

bool rtsp_server_session_snapshot(rtsp_session_snapshot_t *out) {
  memset(out, 0, sizeof(*out));
  if (!s_owner_lock) {
    return false;
  }
  owner_lock();
  const rtsp_conn_t *c = owner_conn();
  if (c) {
    out->active = true;
    out->protocol_version = c->protocol_version;
    out->sender_srcvers = c->sender_srcvers;
    out->stream_type = c->stream_type;
    out->closed = clients[s_owner].socket < 0;
  }
  owner_unlock();
  return out->active;
}

static void server_task(void *pvParameters) {
  (void)pvParameters;

  struct sockaddr_in server_addr, client_addr;
  socklen_t client_addr_len = sizeof(client_addr);

  // Initialize slots
  for (int i = 0; i < RTSP_MAX_CLIENTS; i++) {
    clients[i].socket = -1;
    clients[i].conn = NULL;
    clients[i].task = NULL;
    clients[i].should_stop = false;
    clients[i].is_old = false;
  }
  s_owner = -1;

  server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (server_socket < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %d", errno);
    server_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  int opt = 1;
  setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(airplay_rtsp_port());

  if (bind(server_socket, (struct sockaddr *)&server_addr,
           sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind: %d", errno);
    close(server_socket);
    server_socket = -1;
    server_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  if (listen(server_socket, 5) < 0) {
    ESP_LOGE(TAG, "Failed to listen: %d", errno);
    close(server_socket);
    server_socket = -1;
    server_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "RTSP server listening on port %d", airplay_rtsp_port());
  server_running = true;

  while (server_running) {
    int new_socket = accept(server_socket, (struct sockaddr *)&client_addr,
                            &client_addr_len);
    if (new_socket < 0) {
      if (server_running) {
        ESP_LOGE(TAG, "Failed to accept: %d", errno);
      }
      continue;
    }

    // A new connection never disturbs the session: it gets a free slot.
    int new_slot = -1;
    for (int i = 0; i < RTSP_MAX_CLIENTS && new_slot < 0; i++) {
      if (clients[i].task == NULL) {
        new_slot = i;
      }
    }
    if (new_slot < 0) {
      // All taken: make room by closing the longest-idle connection that
      // does not own the session.
      int victim = -1;
      for (int i = 0; i < RTSP_MAX_CLIENTS; i++) {
        if (i != s_owner &&
            (victim < 0 ||
             clients[i].last_request_us < clients[victim].last_request_us)) {
          victim = i;
        }
      }
      if (victim >= 0) {
        ESP_LOGI(TAG, "All %d slots busy: closing idle slot %d",
                 RTSP_MAX_CLIENTS, victim);
        signal_client_stop(&clients[victim], false);
        if (wait_client_stopped(&clients[victim], RTSP_TAKEOVER_WAIT_MS)) {
          new_slot = victim;
        }
      }
      if (new_slot < 0) {
        ESP_LOGE(TAG, "No free RTSP slot");
        close(new_socket);
        continue;
      }
    }
    ESP_LOGI(TAG, "New client connected (slot %d)", new_slot);

    clients[new_slot].socket = new_socket;
    clients[new_slot].should_stop = false;
    clients[new_slot].is_old = false;
    clients[new_slot].last_request_us = esp_timer_get_time();

    clients[new_slot].task = NULL;
    BaseType_t task_ret =
        xTaskCreate(client_task, "rtsp_client", CLIENT_STACK_SIZE,
                    (void *)(intptr_t)new_slot, 5, &clients[new_slot].task);
    if (task_ret != pdPASS || clients[new_slot].task == NULL) {
      ESP_LOGE(TAG, "Failed to create client task");
      close(new_socket);
      clients[new_slot].socket = -1;
      clients[new_slot].task = NULL;
    }
  }

  // Stop all clients
  for (int i = 0; i < RTSP_MAX_CLIENTS; i++) {
    signal_client_stop(&clients[i], false);
  }

  vTaskDelay(pdMS_TO_TICKS(500));

  if (server_socket >= 0) {
    close(server_socket);
    server_socket = -1;
  }

  server_task_handle = NULL;
  vTaskDelete(NULL);
}

static bool rtsp_server_wait_for_task_stopped(int timeout_ticks) {
  while (server_task_handle != NULL && timeout_ticks-- > 0) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return server_task_handle == NULL;
}

esp_err_t rtsp_server_start(void) {
  if (server_task_handle != NULL) {
    if (server_running) {
      return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGW(TAG, "RTSP server task still stopping, waiting");
    if (!rtsp_server_wait_for_task_stopped(40)) {
      ESP_LOGE(TAG, "Previous RTSP server task did not stop");
      return ESP_ERR_INVALID_STATE;
    }
  }

  if (!s_owner_lock) {
    s_owner_lock = xSemaphoreCreateMutex();
    if (!s_owner_lock) {
      return ESP_ERR_NO_MEM;
    }
  }

  // Parsing the key and deriving the first blinding pair costs the best part
  // of a second; a sender waiting on its OPTIONS response gives up before
  // that. Do it before the listener exists so it cannot land mid-request.
  rsa_init();

  BaseType_t task_ret =
      xTaskCreate(server_task, "rtsp_server", SERVER_STACK_SIZE, NULL, 5,
                  &server_task_handle);
  if (task_ret != pdPASS || server_task_handle == NULL) {
    return ESP_FAIL;
  }

  return ESP_OK;
}

void rtsp_server_stop(void) {
  server_running = false;

  if (server_socket >= 0) {
    shutdown(server_socket, SHUT_RDWR);
    close(server_socket);
    server_socket = -1;
  }

  if (server_task_handle != NULL) {
    if (!rtsp_server_wait_for_task_stopped(40)) {
      ESP_LOGW(TAG, "RTSP server task did not exit within timeout");
    }
  }
}

// ============================================================================
// Remote control: commands from this device back to the sender
// ============================================================================

static bool send_mr_command(const airplay_mr_command_plist_t *cmd) {
  uint8_t body[192];
  const size_t len = bplist_build_mr_command(body, sizeof(body), cmd);
  return len > 0 && rtsp_events_send_command(body, len);
}

static airplay_route_t current_route(rtsp_session_snapshot_t *session) {
  if (!rtsp_server_session_snapshot(session)) {
    return AIRPLAY_ROUTE_NONE;
  }
  return airplay_remote_route(session->protocol_version,
                              session->sender_srcvers, rtsp_events_connected(),
                              dacp_is_active());
}

bool airplay_remote_command(airplay_mr_command_t command) {
  rtsp_session_snapshot_t session;
  const airplay_route_t route = current_route(&session);
  if (route == AIRPLAY_ROUTE_EVENTS) {
    // As Apple's receiver does it: a pause-type command silences the
    // buffered output at once, keeping the buffer; the sender's
    // SETRATEANCHORTIME rate 0 (or FLUSH) follows and finds it paused.
    const bool pause_here = airplay_mr_pauses_locally(command) &&
                            session.stream_type == 103 &&
                            audio_receiver_is_playing();
    const airplay_mr_command_plist_t cmd = {
        .fourcc = airplay_mr_fourcc(command),
        .mr_command = (int)command,
    };
    if (!send_mr_command(&cmd)) {
      return false;
    }
    if (pause_here) {
      playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_PAUSED,
                           NULL);
      audio_receiver_pause();
      audio_output_flush();
    }
    ESP_LOGI(TAG, "Remote command %s (%d) sent on the event channel%s",
             cmd.fourcc, (int)command, pause_here ? "; paused here" : "");
    return true;
  }
  if (route == AIRPLAY_ROUTE_DACP) {
    if (command == AIRPLAY_MR_TOGGLE || command == AIRPLAY_MR_PLAY) {
      // An AirPlay 1 sender pauses by dropping the connection: tell the grace
      // period a resume is coming, so it waits for the reconnect.
      rtsp_server_request_resume();
    }
    dacp_send_command(airplay_mr_dacp_word(command));
    ESP_LOGI(TAG, "Remote command %s sent over DACP",
             airplay_mr_dacp_word(command));
    return true;
  }
  return false;
}

bool airplay_remote_volume(float volume_db, bool muted) {
  rtsp_session_snapshot_t session;
  const airplay_route_t route = current_route(&session);
  if (route == AIRPLAY_ROUTE_EVENTS) {
    // dvlc, the slider the phone shows: linear over -30..0 dB, the same map
    // the sender uses to turn it back into a SET_PARAMETER volume.
    const airplay_mr_command_plist_t cmd = {
        .fourcc = "dvlc",
        .mr_command = -1,
        .has_volume = true,
        .volume = airplay_volume_db_to_slider(volume_db),
        .is_muted = muted,
    };
    return send_mr_command(&cmd);
  }
  if (route == AIRPLAY_ROUTE_DACP) {
    dacp_send_volume(airplay_volume_db_to_slider(volume_db) * 100.0f);
    return true;
  }
  return false;
}

void airplay_remote_interrupted(void) {
  rtsp_session_snapshot_t session;
  if (current_route(&session) != AIRPLAY_ROUTE_EVENTS) {
    return;
  }
  // Apple's "mimic user pause" when local audio takes over: the phone stops
  // rather than playing on into a speaker that is no longer listening.
  const airplay_mr_command_plist_t cmd = {
      .fourcc = airplay_mr_fourcc(AIRPLAY_MR_PAUSE),
      .mr_command = AIRPLAY_MR_PAUSE,
  };
  if (send_mr_command(&cmd)) {
    ESP_LOGI(TAG, "Output taken by another input: pause sent to the sender");
  }
}

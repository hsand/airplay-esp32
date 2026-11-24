// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtsp_conn.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "audio_receiver.h"
#include "audio_volume.h"
#include "esp_log.h"
#include "settings.h"

static const char *TAG = "rtsp_conn";

rtsp_conn_t *rtsp_conn_create(void) {
  rtsp_conn_t *conn = calloc(1, sizeof(rtsp_conn_t));
  if (!conn) {
    return NULL;
  }

  // Load saved volume or use default
  float saved_volume;
  (void)settings_get_volume(&saved_volume);
  conn->volume_db = saved_volume;
  conn->volume_q15 = audio_volume_q15(saved_volume);

  ESP_LOGI(TAG, "New conn: starting volume=%.2f dB (q15=%ld)", conn->volume_db,
           (long)conn->volume_q15);

  conn->data_socket = -1;
  conn->control_socket = -1;
  conn->event_socket = -1;

  return conn;
}

void rtsp_conn_free(rtsp_conn_t *conn) {
  if (!conn) {
    return;
  }

  // Persist volume at disconnect
  settings_persist_volume();

  // Cleanup any resources
  rtsp_conn_cleanup(conn);

  // Free HAP session if present
  if (conn->hap_session) {
    hap_session_free(conn->hap_session);
    conn->hap_session = NULL;
  }

  free(conn);
}

void rtsp_conn_reset_stream(rtsp_conn_t *conn) {
  if (!conn) {
    return;
  }

  // Reset stream state but keep session alive
  conn->stream_active = false;
  conn->stream_paused = true; // Paused, not fully torn down

  // Keep ports allocated for quick resume
  // Don't clear: data_port, control_port, timing_port, event_port
}

void rtsp_conn_cleanup(rtsp_conn_t *conn) {
  if (!conn) {
    return;
  }

  // Note: audio_receiver_stop() is NOT called here — it is a global operation
  // and must be managed by the caller (rtsp_server cleanup / handle_teardown)
  // to avoid killing a new session's audio during client replacement.

  // Close sockets
  if (conn->data_socket >= 0) {
    close(conn->data_socket);
    conn->data_socket = -1;
  }
  if (conn->control_socket >= 0) {
    close(conn->control_socket);
    conn->control_socket = -1;
  }
  if (conn->event_socket >= 0) {
    close(conn->event_socket);
    conn->event_socket = -1;
  }

  // Reset stream state
  conn->stream_active = false;
  conn->stream_paused = false;
  conn->data_port = 0;
  conn->control_port = 0;
  conn->timing_port = 0;
  conn->event_port = 0;
  conn->buffered_port = 0;

  // The PTP clock is released by the caller, and only for the connection that
  // owns the session (rtsp_server.c): a connection being replaced must not
  // clear the clock its successor's SETUP has already started using.

  // Reset encryption state
  conn->encrypted_mode = false;
}

void rtsp_conn_set_volume(rtsp_conn_t *conn, float volume_db) {
  if (!conn) {
    return;
  }

  // The slider's range, as Apple keeps it: the sender's -144 ("muted") is
  // its bottom, -30 dB.
  if (!(volume_db > -30.0f)) {
    volume_db = -30.0f;
  } else if (volume_db > 0.0f) {
    volume_db = 0.0f;
  }
  // Apple drops a change it would not hear: under 0.3 dB, 1 % of the slider.
  if (!audio_volume_change_noticeable(conn->volume_db, volume_db)) {
    return;
  }
  conn->volume_db = volume_db;
  // AirPlay volume: 0 dB = max, -30 dB = mute, by the configured law.
  conn->volume_q15 = audio_volume_q15(volume_db);

  // Update cached volume + DAC (NVS persisted at disconnect)
  settings_set_volume(volume_db);
}

int32_t rtsp_conn_get_volume_q15(rtsp_conn_t *conn) {
  if (!conn) {
    return 32768; // Default full volume
  }
  return conn->volume_q15;
}

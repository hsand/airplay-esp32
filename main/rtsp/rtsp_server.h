// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#include "rtsp_policy.h"

struct rtsp_conn;

/**
 * Port the RTSP server listens on, which the mDNS advertisement must match.
 *
 * 7000 is the AirPlay 2 port; classic RAOP lives on 5000, and a sender that
 * sees 7000 in the SRV record may take the AirPlay 2 path regardless of the
 * TXT record.
 */
uint16_t airplay_rtsp_port(void);

/**
 * Start the AirPlay RTSP server.
 * Handles initial connection requests from iOS devices
 */
esp_err_t rtsp_server_start(void);

/**
 * Stop the RTSP server
 */
void rtsp_server_stop(void);

/**
 * Set volume from AirPlay (in dB, range -144 to 0)
 * @param volume_db Volume in dB (0 = max, -144 = mute)
 */
void airplay_set_volume(float volume_db);

/**
 * Get current volume as Q15 scale factor for audio processing
 * @return Q15 fixed-point multiplier (0 = mute, 32768 = unity)
 */
int32_t airplay_get_volume_q15(void);

/**
 * Request resume during the AirPlay v1 grace period.
 * Called from the play/pause button when the source is still AirPlay
 * but the RTSP connection has been torn down (paused). Sends a DACP
 * playpause to the phone so it reconnects.
 */
void rtsp_server_request_resume(void);

/**
 * A connection asks for the AirPlay session (AirPlay 2 SETUP, AirPlay 1
 * ANNOUNCE or SETUP).  Called on that connection's task.  If another
 * connection holds the session it is taken over -- its connection closed, and
 * once its task has finished, its audio stopped -- unless interruptions are
 * denied (see rtsp_preempt_decide()).  A takeover blocks the caller until the
 * old connection's task has finished, for at most a few seconds.
 * @param remote_control_only the SETUP has isRemoteControlOnly set
 * @return 0 when this connection now owns the session, else the RTSP status
 *         to answer with: 453 refused, 455 this connection is being closed,
 *         403 remote-control-only
 */
int rtsp_server_claim_session(struct rtsp_conn *conn, bool remote_control_only);

/** What the remote-control path needs to know about the session. */
typedef struct {
  bool active; // A connection owns the session.
  bool closed; // ...whose socket is gone (AirPlay 1 grace period).
  int protocol_version;
  uint32_t sender_srcvers; // From the SETUP's sourceVersion, 0 if unknown.
  int64_t stream_type;     // 96 realtime, 103 buffered, 0 none yet.
} rtsp_session_snapshot_t;

bool rtsp_server_session_snapshot(rtsp_session_snapshot_t *out);

/**
 * Send a transport command to the session's sender: MediaRemote over the
 * event channel for an AirPlay 2 sender, DACP for AirPlay 1 and senders
 * older than 354.54.6.  A pause, stop or toggle also pauses the buffered
 * output here at once, as Apple's receiver does.
 * @return false when the sender cannot be reached (the caller falls back)
 */
bool airplay_remote_command(airplay_mr_command_t command);

/**
 * Tell the sender this device's volume changed (dvlc on the event channel,
 * dmcp.volume over DACP), so the phone's slider follows.
 * @return false when the sender cannot be reached
 */
bool airplay_remote_volume(float volume_db, bool muted);

/**
 * Another input (Bluetooth, the USB sink, Sendspin) is taking the output:
 * send the AirPlay 2 sender a pause, as Apple's receiver does on an audio
 * interruption, so the phone stops instead of playing into a dead speaker.
 */
void airplay_remote_interrupted(void);

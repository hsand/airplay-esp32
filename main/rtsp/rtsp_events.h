// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/**
 * The AirPlay 2 event channel: the TCP connection the sender opens to the
 * session SETUP's eventPort.  Apple's receiver uses it to send the sender
 * requests of its own, chiefly POST /command {type: sendMediaRemoteCommand},
 * which is how a speaker's buttons reach a modern iPhone or Mac.
 *
 * It is encrypted like the RTSP connection, with its own keys: HKDF-SHA512
 * over the pairing's input keying material, salt "Events-Salt", and the
 * receiver *writes* with "Events-Write-Encryption-Key" and reads with
 * "Events-Read-Encryption-Key" (the opposite naming from Control-Salt, where
 * the receiver decrypts with Control-Write).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * Serve the event port's listening socket for one session: accept the
 * sender's connection, read and log its replies, and notice when it closes.
 * Replaces any previous session's task.
 */
esp_err_t rtsp_start_event_port_task(int listen_socket);
void rtsp_stop_event_port_task(void);
int rtsp_event_port_listen_socket(void);

/**
 * The keying material for the next event connection, from the session's
 * pairing.  ikm_len 0 leaves the channel in plaintext, as an unencrypted RTSP
 * connection (legacy pair-verify) is.  Call before the sender connects, i.e.
 * at the session SETUP.
 */
void rtsp_events_set_keys(const uint8_t *ikm, size_t ikm_len);

/** True while the sender's event connection is open. */
bool rtsp_events_connected(void);

/**
 * Send POST /command with a binary-plist body on the event channel.  Callable
 * from any task; returns once the request is on the socket (the reply is read
 * and logged by the event task).
 * @return false when there is no event connection or the send failed
 */
bool rtsp_events_send_command(const uint8_t *body, size_t body_len);

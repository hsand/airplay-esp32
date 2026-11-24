// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "rtsp_conn.h"
#include "rtsp_message.h"

/**
 * RTSP Method Handler Dispatch Table
 * Inspired by shairport-sync's method_handlers pattern
 */

/**
 * AirPlay feature bits, as (hi << 32) | lo.
 *
 * Key bits:
 *   Bit 38: SupportsCoreUtilsPairingAndEncryption
 *   Bit 46: SupportsHKPairingAndAccessControl
 *   Bit 48: SupportsTransientPairing
 * AirPlay 1 strips the pairing/encryption bits so iOS uses classic RAOP.
 */
uint64_t airplay_features(void);

// audioBufferSize for a buffered stream is sized per codec in
// airplay_audio_buffer_size() (rtsp_policy.h), capped at the 512 KiB this
// receiver used to advertise: the sender queues that much ahead, and on a
// seek or skip every queued byte is stale but still has to cross the link
// before the new position arrives.  Under BT coexistence the link only
// manages ~0.6 Mbit/s, so 1 MB (5.9 s of PCM) cost ~2 s of pure drain per
// seek.  Realtime (type 96) is paced over UDP and gets none, as on Apple.

// Include for audio_format_t
#include "audio_receiver.h"

/**
 * Codec registry entry
 */
typedef struct {
  const char *name; // Codec name: "ALAC", "AAC", "OPUS"
  int64_t type_id;  // bplist "ct" value (2=ALAC, 4=AAC, 8=AAC-ELD)
} rtsp_codec_t;

/**
 * Configure audio format from codec type ID
 * Looks up codec in registry and calls its configure function.
 * @param type_id Codec type from bplist "ct" field
 * @param fmt Audio format struct to configure
 * @param sample_rate Sample rate from bplist
 * @param samples_per_frame Samples per frame from bplist
 * @return true if codec found and configured, false otherwise
 */
bool rtsp_codec_configure(int64_t type_id, audio_format_t *fmt,
                          int64_t sample_rate, int64_t samples_per_frame);

/**
 * Handler function type
 */
typedef void (*rtsp_handler_fn)(int socket, rtsp_conn_t *conn,
                                const rtsp_request_t *req,
                                const uint8_t *raw_request, size_t raw_len);

/**
 * Method handler entry
 */
typedef struct {
  const char *method;
  rtsp_handler_fn handler;
} rtsp_method_handler_t;

/**
 * Dispatch RTSP request to appropriate handler
 * @param socket Client socket
 * @param conn Connection state
 * @param raw_request Raw request data
 * @param raw_len Length of raw request
 * @return 0 on success, -1 on error
 */
int rtsp_dispatch(int socket, rtsp_conn_t *conn, const uint8_t *raw_request,
                  size_t raw_len);

/**
 * Get device ID string (MAC address format)
 * @param device_id Output buffer (at least 18 bytes)
 * @param len Buffer size
 */
void rtsp_get_device_id(char *device_id, size_t len);

/**
 * This unit's pairing identity ("pi" in TXT and /info): a UUID derived from
 * the Wi-Fi MAC, stable across boots and different on every unit.
 */
void airplay_get_pairing_identity(char out[37]);

/**
 * End of an AirPlay session (full TEARDOWN, connection closed, idle timeout):
 * drop the session's timing peers and end the PTP session.  Apple stops its
 * clock for the session only here, never on a stream TEARDOWN or a pause.
 * Only for the connection that owns the session.
 */
void rtsp_session_end_clock(void);

/**
 * Seconds without any request (/feedback included) after which the session
 * is torn down: 30, or 120 while a buffered stream exists.
 */
int rtsp_session_idle_limit_s(const rtsp_conn_t *conn);

/**
 * TCP keepalive on an RTSP connection, as Apple's receiver sets it: probes
 * after idle_s seconds of silence (6, or 24 with a buffered stream), 5 of
 * them, idle_s apart.
 */
void rtsp_set_keepalive(int socket, int idle_s);

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hap.h"

/**
 * RTSP Connection State Management
 * Consolidates all session state for an AirPlay connection
 */

// Forward declaration
typedef struct rtsp_conn rtsp_conn_t;

/**
 * Connection state struct - consolidates all session state
 */
struct rtsp_conn {
  // HAP session for pairing/encryption
  hap_session_t *hap_session;
  bool encrypted_mode;

  // Volume control: Q15 fixed-point (0-32768)
  // 32768 = 0 dB (unity), 0 = mute
  volatile int32_t volume_q15;
  float volume_db;

  // Audio streaming state
  bool stream_active;
  bool stream_paused;
  int64_t stream_type;    // 96=UDP realtime, 103=TCP buffered
  uint16_t data_port;     // UDP port for audio data (type 96)
  uint16_t control_port;  // UDP port for control (retransmit requests)
  uint16_t timing_port;   // UDP port for timing (our local port)
  uint16_t event_port;    // TCP port for server->client events
  uint16_t buffered_port; // TCP port for buffered audio (type 103)
  int data_socket;
  int control_socket;
  int event_socket; // TCP listener for event port

  // Client address for AirPlay 1 timing requests
  uint32_t client_ip;           // Client IP (network byte order)
  uint16_t client_timing_port;  // Client's timing port (for sending requests)
  uint16_t client_control_port; // Client's control port

  // Codec info from ANNOUNCE/SETUP
  char codec[32];
  int sample_rate;
  int channels;
  int bits_per_sample;

  // DACP identifiers for sending commands back to the client
  char dacp_id[32];       // DACP-ID header (hex string)
  char active_remote[32]; // Active-Remote header (token string)

  // AirPlay protocol version detected from request shape:
  //   0 = unknown (handshake not complete)
  //   1 = classic RAOP (Apple-Challenge / rsaaeskey / Transport: header)
  //   2 = AirPlay 2 (HAP / bplist streams)
  uint8_t protocol_version;

  // AirPlay 2 session timing from the session SETUP's timingProtocol: PTP, or
  // NTP when the key is absent or says so (Apple's default).
  bool timing_ptp;
  // The buffered stream SETUP's keys have been logged for this session.
  bool stream_keys_logged;

  // This connection holds the AirPlay session (rtsp_server_claim_session());
  // only it may start, pause or flush audio.  Other connections may still
  // ask for /info and pair, as on Apple's receiver.
  bool owns_session;
  // ...and has since ended it with a full TEARDOWN: another sender's SETUP
  // takes over even when interruptions are denied.
  bool session_ended;

  // The sender's sourceVersion from its session SETUP, as an integer
  // (airplay_source_version_parse()); 0 when it did not say.
  uint32_t sender_srcvers;

  // X-Apple-HKP from the last pairing request: the pairing type the sender
  // asked for (4 transient, 0/1 PIN, 3 system...), or -1 when absent.
  int hkp;

  // /command setSenderDisplayLatencyMs.  Apple subtracts it from its
  // platform latency only for a home-theater session, which this receiver
  // never is, so it is kept and logged but changes nothing.
  bool has_sender_display_latency;
  uint32_t sender_display_latency_ms;

  // /command mute: the output is at -144 dB and sender_unmuted_db is the level
  // unmute restores (a SET_PARAMETER volume while muted moves only that).
  bool sender_muted;
  float sender_unmuted_db;
};

/**
 * Create a new connection state
 * @return Allocated connection state, or NULL on failure
 */
rtsp_conn_t *rtsp_conn_create(void);

/**
 * Free connection state and associated resources
 */
void rtsp_conn_free(rtsp_conn_t *conn);

/**
 * Reset stream-related state (called on stream teardown)
 * Keeps session alive but clears audio stream state
 */
void rtsp_conn_reset_stream(rtsp_conn_t *conn);

/**
 * Full cleanup when connection closes
 * Stops audio, closes sockets, clears PTP
 */
void rtsp_conn_cleanup(rtsp_conn_t *conn);

/**
 * Set volume in dB (converts to Q15 internally)
 * @param conn Connection state
 * @param volume_db Volume in dB (0 = max, -30 = mute)
 */
void rtsp_conn_set_volume(rtsp_conn_t *conn, float volume_db);

/**
 * Get volume as Q15 scale factor
 * @param conn Connection state
 * @return Q15 fixed-point multiplier (0 = mute, 32768 = unity)
 */
int32_t rtsp_conn_get_volume_q15(rtsp_conn_t *conn);

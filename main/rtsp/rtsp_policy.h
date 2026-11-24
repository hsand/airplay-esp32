// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/**
 * AirPlay receiver decisions with no platform dependency, kept apart so the
 * host tests can check them (tests/airplay_remote): what is advertised, who
 * gets the speaker when a second sender sets up, which channel a remote
 * command travels on, and the arithmetic for progress and buffer sizes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ============================================================================
// Identity and advertisement
// ============================================================================

// The source version this receiver reports (TXT srcvers, /info, Server:).
// At or above 370.35 the sender may send /command mute and unmute, which the
// receiver handles; at or above 366.0 it gets PTP multi-room; below 635.42 it
// does not expect the asynchronous seek reply this receiver lacks.
#define AIRPLAY_SOURCE_VERSION "377.40.00"

// Senders below this source version are driven over DACP; at or above it
// Apple's receiver sends MediaRemote commands on the event channel instead,
// and a modern sender only offers DACP to receivers below it.  354.54.6.
#define AIRPLAY_SRCVERS_MEDIA_REMOTE 3545406U

// "a.b.c" as Apple compares source versions: a*10000 + b*100 + c.  Missing
// parts count as 0; 0 for NULL or an empty string.
uint32_t airplay_source_version_parse(const char *text);

/** The model the receiver presents itself as (TXT model/am, /info): a
 *  HomePod mini. */
#define AIRPLAY_MODEL "AudioAccessory5,1"

/**
 * AirPlay feature bits, (hi << 32) | lo.  AirPlay 1 mode keeps the classic
 * set; AirPlay 2 adds the metadata bits Apple's receiver always sets (17 text,
 * 16 progress, and 15 artwork only when artwork is accepted) and leaves out
 * bit 47, which Apple never sets and no sender reads.
 */
uint64_t airplay_features_compute(bool airplay_v1, bool artwork);

/** TXT "md": the metadata types matching the feature bits above. */
const char *airplay_metadata_types(bool artwork);

/**
 * /info featuresEx: the feature bits as little-endian bytes, trailing zero
 * bytes dropped, in base64 (Apple's APSFeaturesCopyStringRepresentation).
 * @return false if out is too small
 */
bool airplay_features_ex(uint64_t features, char *out, size_t capacity);

/**
 * The pairing identity ("pi"): a UUID derived from the MAC, so each unit has
 * its own and keeps it.  Formatted 8-4-4-4-12, upper-case, as a version-5
 * style name-based UUID.  out must hold 37 bytes.
 */
void airplay_pairing_identity(const uint8_t mac[6], char out[37]);

/** One TXT key and value, in the layout of mdns_txt_item_t. */
typedef struct {
  const char *key;
  const char *value;
} airplay_txt_item_t;

/** The values the AirPlay 2 TXT records are built from. */
typedef struct {
  const char *device_id; // "AA:BB:CC:DD:EE:FF"
  const char *features;  // "0xLO,0xHI"
  const char *model;
  const char *pk_hex;
  const char *pi;
  const char *metadata_types; // "0,1,2" or "0,2"
} airplay_txt_input_t;

/**
 * The _airplay._tcp TXT record for AirPlay 2 mode.
 * @return items written, or 0 if max is too small
 */
size_t airplay_txt_airplay(const airplay_txt_input_t *in,
                           airplay_txt_item_t *out, size_t max);

/** The _raop._tcp TXT record for AirPlay 2 mode. */
size_t airplay_txt_raop(const airplay_txt_input_t *in, airplay_txt_item_t *out,
                        size_t max);

// ============================================================================
// Sessions and remote control
// ============================================================================

/** What a session SETUP (AirPlay 1: ANNOUNCE) from a connection may do. */
typedef enum {
  RTSP_PREEMPT_PROCEED,  // No other session, or this connection owns it.
  RTSP_PREEMPT_TAKEOVER, // Another connection's session ends; this one wins.
  RTSP_PREEMPT_REFUSE,   // 453: the active session may not be interrupted.
} rtsp_preempt_t;

typedef struct {
  bool has_owner;     // Some connection holds the session.
  bool owner_is_self; // ...and it is this one.
  bool owner_closed;  // Its socket is already gone (AirPlay 1 grace period).
  bool owner_ended;   // It sent a full TEARDOWN and has no session left.
  bool same_sender;   // Same sender address: the sender reconnecting.
  bool deny_interruptions; // The "don't interrupt" setting.
} rtsp_preempt_input_t;

/**
 * Apple's hijack rule: the last sender wins, unless interruptions are denied,
 * in which case anyone else is refused with 453.  Only a session SETUP gets
 * here; /info, pairing and OPTIONS on another connection never touch the
 * session.  A connection that has torn its session down holds nothing worth
 * protecting.  Two departures from Apple: a session whose connection has
 * already closed is always taken over, and the same sender may always take its
 * own
 * session back (a phone reconnecting after a Wi-Fi drop would otherwise be
 * locked out by its own dead connection until keepalive noticed).
 */
rtsp_preempt_t rtsp_preempt_decide(const rtsp_preempt_input_t *in);

/** How a remote command reaches the sender. */
typedef enum {
  AIRPLAY_ROUTE_NONE,   // No way to reach it: the caller falls back.
  AIRPLAY_ROUTE_EVENTS, // POST /command on the event channel (AirPlay 2).
  AIRPLAY_ROUTE_DACP,   // HTTP to the sender's DACP server (AirPlay 1).
} airplay_route_t;

/**
 * As Apple's receiver chooses: MediaRemote over the event channel for an
 * AirPlay 2 sender at or above 354.54.6 (or one that did not say), DACP for
 * AirPlay 1 and older senders.  Either falls back to the other when it is the
 * only one available.
 */
airplay_route_t airplay_remote_route(int protocol_version,
                                     uint32_t sender_srcvers,
                                     bool events_connected, bool dacp_active);

/** MediaRemote commands, by Apple's enum value. */
typedef enum {
  AIRPLAY_MR_PLAY = 0,
  AIRPLAY_MR_PAUSE = 1,
  AIRPLAY_MR_TOGGLE = 2,
  AIRPLAY_MR_STOP = 3,
  AIRPLAY_MR_NEXT = 4,
  AIRPLAY_MR_PREVIOUS = 5,
} airplay_mr_command_t;

/** The four-character AirPlay code for a MediaRemote command ("plps"). */
const char *airplay_mr_fourcc(airplay_mr_command_t command);

/** The DACP /ctrl-int/1/ word for a MediaRemote command ("playpause"). */
const char *airplay_mr_dacp_word(airplay_mr_command_t command);

/** Whether Apple pauses its own output before sending this command. */
bool airplay_mr_pauses_locally(airplay_mr_command_t command);

/** AirPlay dB (-30..0, -144 for mute) as Apple's volume slider, 0..1. */
float airplay_volume_db_to_slider(float db);

// ============================================================================
// Arithmetic
// ============================================================================

/**
 * SET_PARAMETER progress "start/current/end" in seconds, as Apple converts
 * it: 32-bit differences, so an RTP wrap inside the track is harmless, read
 * as signed and divided by the stream's sample rate.  A negative position (the
 * playhead before the start) reads as 0.
 * @return false if sample_rate is 0
 */
bool airplay_progress_seconds(uint32_t start, uint32_t current, uint32_t end,
                              uint32_t sample_rate, uint32_t *elapsed,
                              uint32_t *duration);

/**
 * The position to report: what is being played when the receiver knows it and
 * it lies inside the track, else what the sender said.
 */
uint32_t airplay_progress_current(uint32_t start, uint32_t end,
                                  uint32_t sender_current, bool have_rendered,
                                  uint32_t rendered);

// Seconds of audio the sender may queue ahead on a buffered stream.  Apple
// says 120 s; a seek or skip strands everything queued, and on an ESP32 with
// Bluetooth coexistence the link manages ~0.6 Mbit/s, so a smaller lead is
// worth more here than a long one.  The size allocates nothing here: what
// the sender has queued beyond TCP_WND waits in the sender, so PSRAM holds
// only the decoded timeline either way.  Five seconds rides out a Wi-Fi stall.
#define AIRPLAY_AUDIO_BUFFER_SECONDS 5
// Never above the 512 KiB this receiver advertised before for every codec:
// 1 MiB was measured to cost ~2 s of stale drain per seek under BT coex, and
// 512 KiB still exceeded the ring's steady-state fill.
#define AIRPLAY_AUDIO_BUFFER_MAX (512U * 1024U)
#define AIRPLAY_AUDIO_BUFFER_MIN (128U * 1024U)
// Bytes per second budgeted for AAC: 320 kbit/s, Apple Music's 256 kbit/s
// plus RTP and framing overhead with room to spare.
#define AIRPLAY_AAC_BYTES_PER_SEC 40000U

/**
 * audioBufferSize for a stream SETUP reply, sized from the stream's codec
 * ("ct": 2 ALAC, 4 AAC, 8 AAC-ELD, other = PCM) and sample rate to
 * AIRPLAY_AUDIO_BUFFER_SECONDS, within [MIN, MAX].  0 for a realtime stream
 * (type 96): Apple's reply has no audioBufferSize there, so it is omitted.
 */
uint32_t airplay_audio_buffer_size(int64_t stream_type, int64_t codec_type,
                                   int64_t sample_rate);

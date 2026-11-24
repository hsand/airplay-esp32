// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "audio_flush_queue.h"

/**
 * Audio receiver for AirPlay RTP streams
 * Handles UDP packet reception and audio decoding
 */

// Audio format info from ANNOUNCE SDP
typedef struct {
  char codec[32];      // "AppleLossless", "AAC", etc.
  int sample_rate;     // 44100, 48000, etc.
  int channels;        // 1 or 2
  int bits_per_sample; // 16, 24
  int frame_size;      // Samples per frame (ALAC: 352)

  // ALAC-specific config (from fmtp line)
  uint32_t max_samples_per_frame;
  uint8_t sample_size;
  uint8_t rice_history_mult;
  uint8_t rice_initial_history;
  uint8_t rice_limit;
  uint8_t num_channels;
  uint16_t max_run;
  uint32_t max_coded_frame_size;
  uint32_t avg_bit_rate;
  uint32_t sample_rate_config;
} audio_format_t;

// Audio encryption types
typedef enum {
  AUDIO_ENCRYPT_NONE = 0,
  AUDIO_ENCRYPT_AES_CBC,
  AUDIO_ENCRYPT_CHACHA20_POLY1305
} audio_encrypt_type_t;

// Audio encryption configuration
typedef struct {
  audio_encrypt_type_t type;
  uint8_t key[32]; // AES-128 uses 16, ChaCha20 uses 32
  uint8_t iv[16];  // AES-CBC IV
  size_t key_len;
} audio_encrypt_t;

// Audio buffer statistics
typedef struct {
  uint32_t packets_received;
  uint32_t packets_decoded;
  uint32_t packets_dropped;
  uint32_t decrypt_errors;
  uint32_t buffer_underruns;
  uint32_t buffer_overruns;
  uint32_t late_frames;
  uint16_t last_seq;
  uint32_t last_timestamp;
} audio_stats_t;

/**
 * Initialize audio receiver
 */
esp_err_t audio_receiver_init(void);

/**
 * Called when an AirPlay session takes the audio path and again when it lets
 * go, so the arbitration in main.c can stand the other inputs down.
 */
typedef void (*audio_receiver_activity_cb_t)(bool active);
void audio_receiver_set_activity_callback(audio_receiver_activity_cb_t cb);

/**
 * Give up the claim on the audio path. Called on a full TEARDOWN and when the
 * RTSP connection goes away -- not on audio_receiver_stop(), which also runs
 * for a stream-level teardown, and that is a pause.
 */
void audio_receiver_end_session(void);

/**
 * Set audio format from ANNOUNCE SDP
 */
void audio_receiver_set_format(const audio_format_t *format);

/**
 * Set encryption parameters for RTP decryption
 */
void audio_receiver_set_encryption(const audio_encrypt_t *encrypt);

/**
 * Start receiving audio on specified port
 */
esp_err_t audio_receiver_start(uint16_t data_port, uint16_t control_port);

/**
 * Start the active stream type using the provided ports.
 */
esp_err_t audio_receiver_start_stream(uint16_t data_port, uint16_t control_port,
                                      uint16_t tcp_port);

/**
 * Stop receiving audio
 */
void audio_receiver_stop(void);

/**
 * Get audio statistics
 */
void audio_receiver_get_stats(audio_stats_t *stats);

/**
 * Read decoded PCM samples from buffer
 * @param buffer Output buffer for PCM samples (interleaved stereo, 16-bit)
 * @param samples Maximum number of samples to read (per channel)
 * @return Number of samples actually read
 */
size_t audio_receiver_read(int16_t *buffer, size_t samples);

/**
 * Check if audio data is available
 */
bool audio_receiver_has_data(void);

/**
 * Flush audio buffer (full stop path — used by TEARDOWN and stop).
 */
void audio_receiver_flush(void);

/**
 * Flush audio buffer for a mid-stream seek (FLUSH / immediate FLUSHBUFFERED).
 * Identical to audio_receiver_flush(): the timeline re-prerolls from whatever
 * anchor the sender supplies next, so the seek costs no more than the new
 * stream's own lead time.
 */
void audio_receiver_seek_flush(void);

/**
 * Flush for a plain RTSP FLUSH.  On the realtime stream only audio from
 * before the flush point is stale, so packets arriving after it are kept
 * rather than discarded until the next anchor.  until_rtp, when valid, is
 * the flush point from the request's RTP-Info.  On the buffered stream this
 * is audio_receiver_seek_flush().
 */
void audio_receiver_realtime_flush(bool until_valid, uint32_t until_rtp);

/**
 * AirPlay 2 FLUSHBUFFERED, as Apple's receiver applies it (see
 * audio_flush_queue.h): by sequence number, both to what is buffered and to
 * what is still arriving.  An immediate flush (no flushFromSeq) also stops
 * output until the next SETRATEANCHORTIME.
 *
 * @return true when the reply carries RTP-Info: rtptime=*reply_rtp (the
 *         record was processed at once and something had been rendered).
 */
bool audio_receiver_flush_buffered(const audio_flush_record_t *record,
                                   uint32_t *reply_rtp);

/**
 * SETRATEANCHORTIME rate=0 (and PAUSE): stop output, keep every buffered
 * packet, forget the anchor.  The resume brings a new one.
 */
void audio_receiver_pause(void);

/**
 * A validated SETRATEANCHORTIME, rate 0 or 1.  For rate 1 every anchor field
 * was present in the request; the first audible sample is optional.
 */
typedef struct {
  uint32_t rate;
  uint32_t rtp;
  uint64_t network_ns;
  uint64_t timeline_id;
  uint32_t network_flags;
  bool fas_valid;
  uint32_t fas_rtp;
} audio_rate_anchor_t;

/**
 * Apply a buffered stream's SETRATEANCHORTIME.  Rate 1 installs the anchor
 * (or keeps the running timeline when it agrees with it) as soon as the clock
 * can place it, and resumes; rate 0 is audio_receiver_pause().
 */
void audio_receiver_set_rate_anchor(const audio_rate_anchor_t *anchor);

/**
 * The AirPlay 2 session's timing protocol: PTP (true) or NTP.  Decides which
 * clock a buffered anchor is read through.
 */
void audio_receiver_set_session_timing(bool ptp);

/**
 * Sender latency for the buffered stream, in samples: audioLatencyMs or
 * latencyMin from its SETUP, 0 when it sent neither.
 */
void audio_receiver_set_buffered_latency_samples(uint32_t latency_samples);

/**
 * Numbers for the session status line (see rtsp_server.c).
 */
typedef struct {
  uint32_t anchors_installed;
  uint32_t anchors_consistent;
  uint32_t anchor_resets;
  uint32_t anchors_deferred;
  uint32_t gm_rebases;
  uint32_t gm_rebase_failures;
  uint32_t freewheels;
  bool freewheeling;
  uint32_t steps;
  uint32_t step_clock_deferrals;
  uint32_t trims;
  int32_t error_us;
  uint32_t flushes_pending;
  uint32_t flush_dropped;
  uint32_t buffered_blocks;
  const char *state;
} audio_receiver_diag_t;

void audio_receiver_get_diag(audio_receiver_diag_t *diag);

/**
 * Set the realtime stream's playout latency in samples, added to every
 * frame's anchor-scheduled play time: latencyMin from SETUP (default 11025 =
 * 250 ms).  Buffered streams use audio_receiver_set_buffered_latency_samples().
 */
void audio_receiver_set_playout_latency_samples(uint32_t latency_samples);

/**
 * Set advertised/target output latency in microseconds.
 */
void audio_receiver_set_output_latency_us(uint32_t latency_us);

/**
 * Get current output latency in microseconds (buffer latency only).
 */
uint32_t audio_receiver_get_output_latency_us(void);

/**
 * Get hardware output latency in microseconds (I2S DMA pipeline delay).
 */
uint32_t audio_receiver_get_hardware_latency_us(void);

/**
 * Get total latency in microseconds (jitter-buffer target depth + hardware
 * DMA delay + fixed pipeline constant).
 *
 * DIAGNOSTIC ONLY — do NOT report this in outputLatencyMicros.  The RTSP
 * layer intentionally advertises 0; see audio_timing_get_advertised_latency()
 * in audio_timing.h for why advertising a non-zero value double-compensates
 * and makes this device play ahead of other speakers.
 */
uint32_t audio_receiver_get_advertised_latency_us(void);

/**
 * Provide anchor timing information from SETRATEANCHORTIME.
 * @param clock_id PTP clock ID (networkTimeTimelineID)
 * @param network_time_ns Anchor time in nanoseconds (PTP timeline)
 * @param rtp_time RTP timestamp for the anchor
 */
void audio_receiver_set_anchor_time(uint64_t clock_id, uint64_t network_time_ns,
                                    uint32_t rtp_time);

/**
 * Enable or pause playout scheduling.
 */
void audio_receiver_set_playing(bool playing);

/**
 * Check if playback is currently active (not paused).
 */
bool audio_receiver_is_playing(void);

/**
 * The RTP timestamp being handed to the output now, while playing: what the
 * now-playing position is computed from, as Apple's buffered receiver does.
 * @return false when nothing is playing or nothing has been rendered yet
 */
bool audio_receiver_get_rendered_rtp(uint32_t *rtp);

/**
 * Reset timing anchor (call when PTP clock changes, e.g., SETPEERS)
 */
void audio_receiver_reset_timing(void);

/**
 * Set the client's control address for NACK retransmission requests.
 * @param client_ip Client IP in network byte order
 * @param client_control_port Client's control port (host byte order)
 */
void audio_receiver_set_client_control(uint32_t client_ip,
                                       uint16_t client_control_port);

/**
 * Stream types for AirPlay 2
 */
typedef enum {
  AUDIO_STREAM_NONE = 0,
  AUDIO_STREAM_REALTIME = 96, // UDP, ALAC
  AUDIO_STREAM_BUFFERED = 103 // TCP, AAC-ELD
} audio_stream_type_t;

/**
 * Start buffered audio receiver (type=103) on TCP port
 * @param tcp_port Port to listen on for TCP connections
 * @return ESP_OK on success
 */
esp_err_t audio_receiver_start_buffered(uint16_t tcp_port);

/**
 * Get the active stream port (data or buffered).
 */
uint16_t audio_receiver_get_stream_port(void);

/**
 * Get the TCP port for buffered audio (after start_buffered)
 */
uint16_t audio_receiver_get_buffered_port(void);

/**
 * Stop only the buffered receiver but keep playing buffered data
 * Used when TEARDOWN with streams array is received (sender done sending)
 */
void audio_receiver_stop_buffered_only(void);

/**
 * Set the stream type (realtime vs buffered)
 */
void audio_receiver_set_stream_type(audio_stream_type_t type);

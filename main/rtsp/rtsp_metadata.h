// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/**
 * Now-playing for the AirPlay session, merged from the pieces a sender sends
 * separately -- DMAP text (SET_PARAMETER application/x-dmap-tagged), progress
 * (text/parameters "progress: start/current/end") and artwork -- into one
 * state, which is what every PLAYBACK_EVENT_METADATA carries.  A listener
 * never sees a progress-only event with blank strings, or a text-only one
 * that zeroes the position.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "playback_events.h"

/** What one DMAP listing (an mlit) says. */
typedef struct {
  char title[METADATA_STRING_MAX];  // minm
  char artist[METADATA_STRING_MAX]; // asar
  char album[METADATA_STRING_MAX];  // asal
  char genre[METADATA_STRING_MAX];  // asgn
  bool has_duration;
  uint32_t duration_ms; // astm
  bool has_caps;
  uint8_t caps; // play state: 2 paused, 3 stopped, 4 fast-forward, 5 rewind
} airplay_dmap_t;

/**
 * Parse a DMAP listing.  Containers (mlit, cmst, mdst) are descended into;
 * unknown tags are skipped; a truncated item ends the walk.
 * @return true if any known tag was found
 */
bool airplay_dmap_parse(const uint8_t *data, size_t len, airplay_dmap_t *out);

/** Whether a DMAP play state (caps) means the sender's rate is 0. */
bool airplay_dmap_caps_paused(uint8_t caps);

/** A new session: forget the previous one's now-playing. */
void rtsp_metadata_reset(void);

/** DMAP text from SET_PARAMETER; merges and emits. */
void rtsp_metadata_dmap(const uint8_t *data, size_t len);

/**
 * SET_PARAMETER "progress: start/current/end"; merges and emits.  The
 * position is what this receiver is playing when it knows that, as Apple's
 * buffered receiver reports it.
 * @param sample_rate the stream's rate (0 = 44100)
 */
void rtsp_metadata_progress(const char *text, uint32_t sample_rate);

/** Artwork arrived (only flagged; the image itself is not kept). */
void rtsp_metadata_artwork(void);

/**
 * Anything else that carries now-playing fields (a third-party sender's
 * bplist SET_PARAMETER): non-empty strings and a non-zero duration replace
 * the merged ones; the position only when has_position.
 */
void rtsp_metadata_update(const playback_metadata_t *partial,
                          bool has_position);

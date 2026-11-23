// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "audio_flush_queue.h"
#include "audio_receiver.h"

/* Request parsing for SETRATEANCHORTIME and FLUSHBUFFERED, kept apart from
 * the handlers so the host tests can check it against real plists. */

// OSStatus values Apple's receiver answers a refused request with.
#define AIRPLAY_STATUS_PARAM_ERR     (-6705) // kParamErr: rate not 0 or 1
#define AIRPLAY_STATUS_NOT_FOUND_ERR (-6727) // kNotFoundErr: a key missing

/**
 * Validate a SETRATEANCHORTIME body as Apple's receiver does: `rate` must be
 * 0 or 1; at rate 1 the anchor (rtpTime or mediaTimeValue/Scale),
 * networkTimeSecs, networkTimeFrac, networkTimeTimelineID and
 * networkTimeFlags must all be present, and firstAudibleRTPTime is optional.
 * At rate 0 no time key is read.
 * @return false with *status set when the request must be refused
 */
bool rtsp_parse_rate_anchor(const uint8_t *body, size_t body_len,
                            int64_t sample_rate, audio_rate_anchor_t *anchor,
                            int64_t *status);

/**
 * Read a FLUSHBUFFERED body: flushUntilSeq is required; flushFromSeq with a
 * valid flushFromTS makes it a deferred, within-range flush.
 * @return false when flushUntilSeq is missing
 */
bool rtsp_parse_flush_buffered(const uint8_t *body, size_t body_len,
                               int64_t sample_rate,
                               audio_flush_record_t *record);

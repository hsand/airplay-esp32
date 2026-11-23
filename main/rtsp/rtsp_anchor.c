// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtsp_anchor.h"

#include <string.h>

#include "plist.h"

static bool is_bplist(const uint8_t *body, size_t body_len) {
  return body && body_len >= 8 && memcmp(body, "bplist00", 8) == 0;
}

// An AirPlay "transport time": an RTP timestamp under `rtp_key`, or a media
// time as a value/scale pair, converted to samples at `sample_rate`.
static bool find_transport_time(const uint8_t *body, size_t body_len,
                                const char *rtp_key, const char *value_key,
                                const char *scale_key, int64_t sample_rate,
                                uint32_t *rtp) {
  int64_t value = 0;
  if (bplist_find_int(body, body_len, rtp_key, &value)) {
    *rtp = (uint32_t)value;
    return true;
  }
  int64_t scale = 0;
  if (bplist_find_int(body, body_len, value_key, &value) &&
      bplist_find_int(body, body_len, scale_key, &scale) && scale > 0) {
    *rtp =
        (uint32_t)(scale == sample_rate ? value : value * sample_rate / scale);
    return true;
  }
  return false;
}

bool rtsp_parse_rate_anchor(const uint8_t *body, size_t body_len,
                            int64_t sample_rate, audio_rate_anchor_t *anchor,
                            int64_t *status) {
  memset(anchor, 0, sizeof(*anchor));
  *status = 0;
  if (!is_bplist(body, body_len)) {
    *status = AIRPLAY_STATUS_NOT_FOUND_ERR;
    return false;
  }

  // An integer on Apple's side; a real is accepted when it is exactly 0 or 1.
  int64_t rate = -1;
  double real_rate = 0.0;
  if (!bplist_find_int(body, body_len, "rate", &rate)) {
    if (!bplist_find_real(body, body_len, "rate", &real_rate)) {
      *status = AIRPLAY_STATUS_NOT_FOUND_ERR;
      return false;
    }
    rate = real_rate == 0.0 ? 0 : real_rate == 1.0 ? 1 : 2;
  }
  if (rate != 0 && rate != 1) {
    *status = AIRPLAY_STATUS_PARAM_ERR;
    return false;
  }
  anchor->rate = (uint32_t)rate;
  if (rate == 0) {
    // None of the time keys mean anything at rate 0.
    return true;
  }

  // Rate 1: every anchor key is required -- present, whatever its value: an
  // rtpTime of 0 is a valid anchor.
  int64_t secs = 0;
  int64_t frac = 0;
  int64_t timeline = 0;
  int64_t flags = 0;
  if (!find_transport_time(body, body_len, "rtpTime", "mediaTimeValue",
                           "mediaTimeScale", sample_rate, &anchor->rtp) ||
      !bplist_find_int(body, body_len, "networkTimeSecs", &secs) ||
      !bplist_find_int(body, body_len, "networkTimeFrac", &frac) ||
      !bplist_find_int(body, body_len, "networkTimeTimelineID", &timeline) ||
      !bplist_find_int(body, body_len, "networkTimeFlags", &flags)) {
    *status = AIRPLAY_STATUS_NOT_FOUND_ERR;
    return false;
  }
  if (secs < 0 || secs > INT32_MAX || flags < 0 || flags > UINT32_MAX) {
    *status = AIRPLAY_STATUS_PARAM_ERR;
    return false;
  }
  // The fraction is a 64-bit binary fraction of a second; its top 32 bits
  // resolve 0.23 ns, which is plenty.
  const uint64_t frac_ns = (((uint64_t)frac >> 32) * 1000000000ULL) >> 32;
  anchor->network_ns = (uint64_t)secs * 1000000000ULL + frac_ns;
  anchor->timeline_id = (uint64_t)timeline;
  // Required and range-checked, then only logged: Apple's receiver passes it
  // along with the network time and acts on nothing in it.
  anchor->network_flags = (uint32_t)flags;
  // Optional, with a validity flag, as on Apple.
  anchor->fas_valid = find_transport_time(
      body, body_len, "firstAudibleRTPTime", "firstAudibleMediaTimeValue",
      "firstAudibleMediaTimeScale", sample_rate, &anchor->fas_rtp);
  return true;
}

bool rtsp_parse_flush_buffered(const uint8_t *body, size_t body_len,
                               int64_t sample_rate,
                               audio_flush_record_t *record) {
  memset(record, 0, sizeof(*record));
  int64_t value = 0;
  if (!is_bplist(body, body_len) ||
      !bplist_find_int(body, body_len, "flushUntilSeq", &value) ||
      !find_transport_time(
          body, body_len, "flushUntilTS", "flushUntilMediaTimeValue",
          "flushUntilMediaTimeScale", sample_rate, &record->until_ts)) {
    return false;
  }
  record->until_seq = (uint32_t)value & AUDIO_SEQ24_MASK;
  // Within-range only with both a fromSeq and a valid fromTS, as on Apple.
  if (bplist_find_int(body, body_len, "flushFromSeq", &value) &&
      find_transport_time(body, body_len, "flushFromTS",
                          "flushFromMediaTimeValue", "flushFromMediaTimeScale",
                          sample_rate, &record->from_ts)) {
    record->has_from = true;
    record->from_seq = (uint32_t)value & AUDIO_SEQ24_MASK;
  }
  return true;
}

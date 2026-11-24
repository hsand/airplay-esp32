// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtsp_policy.h"

#include <stdio.h>
#include <string.h>

#include "base64.h"

// ============================================================================
// Identity and advertisement
// ============================================================================

uint32_t airplay_source_version_parse(const char *text) {
  if (!text) {
    return 0;
  }
  uint32_t parts[3] = {0, 0, 0};
  size_t part = 0;
  for (const char *p = text; *p && part < 3; p++) {
    if (*p >= '0' && *p <= '9') {
      // Saturate rather than wrap on a silly value.
      if (parts[part] < 100000U) {
        parts[part] = parts[part] * 10U + (uint32_t)(*p - '0');
      }
    } else if (*p == '.') {
      part++;
    } else {
      break;
    }
  }
  return parts[0] * 10000U + parts[1] * 100U + parts[2];
}

#define FEATURE(bit) (UINT64_C(1) << (bit))

uint64_t airplay_features_compute(bool airplay_v1, bool artwork) {
  if (airplay_v1) {
    // Classic RAOP, exactly as before: the pairing and buffered bits are
    // stripped so iOS takes the AirPlay 1 path.
    return UINT64_C(0x5C4A00);
  }
  // 9, 11, 14, 18, 19, 20, 22 (audio, formats, unencrypted), 30 (unified
  // advertiser info), 38 (CU pairing), 40 (buffered), 41 (PTP), 46 (HK
  // pairing), 48 (transient pairing).
  uint64_t features = UINT64_C(0x14340405C4A00);
  features |= FEATURE(17) | FEATURE(16); // text and progress metadata
  if (artwork) {
    features |= FEATURE(15);
  }
  return features;
}

const char *airplay_metadata_types(bool artwork) {
  // 0 = text, 1 = artwork, 2 = progress.  Without artwork the sender is
  // asked not to send cover art at all.
  return artwork ? "0,1,2" : "0,2";
}

bool airplay_features_ex(uint64_t features, char *out, size_t capacity) {
  uint8_t bytes[8];
  size_t len = 0;
  for (size_t i = 0; i < sizeof(bytes); i++) {
    bytes[i] = (uint8_t)(features >> (8 * i));
    if (bytes[i] != 0) {
      len = i + 1;
    }
  }
  const size_t need = base64_encoded_length(len) + 1;
  if (!out || capacity < need) {
    return false;
  }
  if (len == 0) {
    out[0] = '\0';
    return true;
  }
  const int written = base64_encode(bytes, len, out, capacity - 1);
  if (written < 0) {
    return false;
  }
  out[written] = '\0';
  return true;
}

// FNV-1a, 64-bit, then the splitmix64 finaliser so that neighbouring MACs
// give unrelated identities: fixed, documented functions, so a given MAC has
// the same identity on every build and in the host test.
static uint64_t fnv1a64(uint64_t hash, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    hash ^= data[i];
    hash *= UINT64_C(0x100000001B3);
  }
  return hash;
}

static uint64_t mix64(uint64_t x) {
  x = (x ^ (x >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
  x = (x ^ (x >> 27)) * UINT64_C(0x94D049BB133111EB);
  return x ^ (x >> 31);
}

void airplay_pairing_identity(const uint8_t mac[6], char out[37]) {
  static const char domain[] = "airplay-esp32 pairing identity";
  uint8_t uuid[16];
  for (int half = 0; half < 2; half++) {
    uint64_t h = UINT64_C(0xCBF29CE484222325) ^ (uint64_t)half;
    h = fnv1a64(h, (const uint8_t *)domain, sizeof(domain) - 1);
    h = mix64(fnv1a64(h, mac, 6));
    for (int i = 0; i < 8; i++) {
      uuid[half * 8 + i] = (uint8_t)(h >> (56 - 8 * i));
    }
  }
  uuid[6] = (uint8_t)((uuid[6] & 0x0F) | 0x50); // version 5 (name-based)
  uuid[8] = (uint8_t)((uuid[8] & 0x3F) | 0x80); // RFC 4122 variant
  snprintf(out, 37,
           "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-"
           "%02X%02X%02X%02X%02X%02X",
           uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6],
           uuid[7], uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13],
           uuid[14], uuid[15]);
}

static size_t put_txt(airplay_txt_item_t *out, size_t max, size_t n,
                      const char *key, const char *value) {
  if (n < max) {
    out[n].key = key;
    out[n].value = value;
  }
  return n + 1;
}

size_t airplay_txt_airplay(const airplay_txt_input_t *in,
                           airplay_txt_item_t *out, size_t max) {
  size_t n = 0;
  n = put_txt(out, max, n, "acl", "0");
  n = put_txt(out, max, n, "deviceid", in->device_id);
  n = put_txt(out, max, n, "features", in->features);
  // Status flags: 0x4, the bit Apple's receiver always sets.
  n = put_txt(out, max, n, "flags", "0x4");
  n = put_txt(out, max, n, "model", in->model);
  n = put_txt(out, max, n, "protovers", "1.1");
  n = put_txt(out, max, n, "pk", in->pk_hex);
  n = put_txt(out, max, n, "pi", in->pi);
  n = put_txt(out, max, n, "srcvers", AIRPLAY_SOURCE_VERSION);
  // The sender ORs "md" into the feature-derived metadata types; Apple's
  // _airplay record has no md, but the bits already say the same thing and
  // a sender that reads only this record then has it too.
  n = put_txt(out, max, n, "md", in->metadata_types);
  // Apple's 26.x receivers send vv=1.
  n = put_txt(out, max, n, "vv", "1");
  return n <= max ? n : 0;
}

size_t airplay_txt_raop(const airplay_txt_input_t *in, airplay_txt_item_t *out,
                        size_t max) {
  size_t n = 0;
  n = put_txt(out, max, n, "am", in->model);
  // PCM, ALAC, AAC.  Not AAC-ELD (3): esp_audio_codec decodes only AAC-LC
  // and HE-AAC, and fed ELD it fails and then faults (measured in QEMU), so a
  // sender that chose it would get silence and a crash.
  n = put_txt(out, max, n, "cn", "0,1,2");
  n = put_txt(out, max, n, "da", "true");
  // et=1 (RSA) and ek=1 keep RAOP-only senders (TuneBlade, AirMusic...)
  // working; Apple's own record has et=0,3,5.
  n = put_txt(out, max, n, "ek", "1");
  n = put_txt(out, max, n, "et", "0,1,3,5");
  n = put_txt(out, max, n, "ft", in->features);
  n = put_txt(out, max, n, "md", in->metadata_types);
  n = put_txt(out, max, n, "pk", in->pk_hex);
  n = put_txt(out, max, n, "sf", "0x4");
  n = put_txt(out, max, n, "tp", "UDP");
  n = put_txt(out, max, n, "vn", "65537");
  n = put_txt(out, max, n, "vs", AIRPLAY_SOURCE_VERSION);
  n = put_txt(out, max, n, "vv", "1");
  return n <= max ? n : 0;
}

// ============================================================================
// Sessions and remote control
// ============================================================================

rtsp_preempt_t rtsp_preempt_decide(const rtsp_preempt_input_t *in) {
  if (in->self_closing) {
    return RTSP_PREEMPT_CLOSING;
  }
  if (in->has_owner && in->owner_is_self) {
    // A later SETUP on the session's own connection updates it; Apple reads
    // the session kind only from the first.
    return RTSP_PREEMPT_PROCEED;
  }
  if (in->remote_control_only) {
    return RTSP_PREEMPT_REMOTE_ONLY;
  }
  if (!in->has_owner) {
    return RTSP_PREEMPT_PROCEED;
  }
  if (in->owner_closed || in->owner_ended || in->same_sender) {
    return RTSP_PREEMPT_TAKEOVER;
  }
  return in->deny_interruptions ? RTSP_PREEMPT_REFUSE : RTSP_PREEMPT_TAKEOVER;
}

int rtsp_preempt_status(rtsp_preempt_t decision) {
  switch (decision) {
  case RTSP_PREEMPT_PROCEED:
  case RTSP_PREEMPT_TAKEOVER:
    return 0;
  case RTSP_PREEMPT_REFUSE:
    return 453;
  case RTSP_PREEMPT_CLOSING:
    return 455;
  case RTSP_PREEMPT_REMOTE_ONLY:
    return 403;
  }
  return 500;
}

airplay_route_t airplay_remote_route(int protocol_version,
                                     uint32_t sender_srcvers,
                                     bool events_connected, bool dacp_active) {
  const bool old_sender =
      sender_srcvers != 0 && sender_srcvers < AIRPLAY_SRCVERS_MEDIA_REMOTE;
  const bool prefer_dacp = protocol_version != 2 || old_sender;
  if (prefer_dacp && dacp_active) {
    return AIRPLAY_ROUTE_DACP;
  }
  if (protocol_version == 2 && events_connected) {
    return AIRPLAY_ROUTE_EVENTS;
  }
  return dacp_active ? AIRPLAY_ROUTE_DACP : AIRPLAY_ROUTE_NONE;
}

const char *airplay_mr_fourcc(airplay_mr_command_t command) {
  switch (command) {
  case AIRPLAY_MR_PLAY:
    return "play";
  case AIRPLAY_MR_PAUSE:
    return "paus";
  case AIRPLAY_MR_TOGGLE:
    return "plps";
  case AIRPLAY_MR_STOP:
    return "stop";
  case AIRPLAY_MR_NEXT:
    return "nitm";
  case AIRPLAY_MR_PREVIOUS:
    return "pitm";
  }
  return NULL;
}

const char *airplay_mr_dacp_word(airplay_mr_command_t command) {
  switch (command) {
  case AIRPLAY_MR_PLAY:
    return "play";
  case AIRPLAY_MR_PAUSE:
    return "pause";
  case AIRPLAY_MR_TOGGLE:
    return "playpause";
  case AIRPLAY_MR_STOP:
    return "stop";
  case AIRPLAY_MR_NEXT:
    return "nextitem";
  case AIRPLAY_MR_PREVIOUS:
    return "previtem";
  }
  return NULL;
}

bool airplay_mr_pauses_locally(airplay_mr_command_t command) {
  return command == AIRPLAY_MR_PAUSE || command == AIRPLAY_MR_STOP ||
         command == AIRPLAY_MR_TOGGLE;
}

float airplay_volume_db_to_slider(float db) {
  // APSVolumeConvertDBToClampedSliderValue: linear over -30..0 dB.
  if (db <= -30.0f) {
    return 0.0f;
  }
  if (db >= 0.0f) {
    return 1.0f;
  }
  return (db + 30.0f) / 30.0f;
}

// ============================================================================
// Arithmetic
// ============================================================================

bool airplay_progress_seconds(uint32_t start, uint32_t current, uint32_t end,
                              uint32_t sample_rate, uint32_t *elapsed,
                              uint32_t *duration) {
  if (sample_rate == 0) {
    return false;
  }
  const int32_t pos = (int32_t)(current - start);
  const int32_t len = (int32_t)(end - start);
  *elapsed = pos > 0 ? (uint32_t)pos / sample_rate : 0U;
  *duration = len > 0 ? (uint32_t)len / sample_rate : 0U;
  return true;
}

uint32_t airplay_progress_current(uint32_t start, uint32_t end,
                                  uint32_t sender_current, bool have_rendered,
                                  uint32_t rendered) {
  if (!have_rendered) {
    return sender_current;
  }
  const int32_t pos = (int32_t)(rendered - start);
  const int32_t len = (int32_t)(end - start);
  // The playhead is in another track (a skip whose progress came first) or
  // the numbers are garbage: believe the sender.
  if (pos < 0 || (len > 0 && pos > len)) {
    return sender_current;
  }
  return rendered;
}

uint32_t airplay_audio_buffer_size(int64_t stream_type, int64_t codec_type,
                                   int64_t sample_rate) {
  if (stream_type != 103) {
    return 0;
  }
  if (sample_rate <= 0 || sample_rate > 384000) {
    sample_rate = 44100;
  }
  uint64_t bytes_per_sec;
  if (codec_type == 4 || codec_type == 8) {
    bytes_per_sec = AIRPLAY_AAC_BYTES_PER_SEC;
  } else {
    // ALAC or PCM, 16-bit stereo: the PCM rate is the upper bound, as Apple
    // sizes lossless.
    bytes_per_sec = (uint64_t)sample_rate * 2U * 2U;
  }
  uint64_t size = bytes_per_sec * AIRPLAY_AUDIO_BUFFER_SECONDS;
  if (size > AIRPLAY_AUDIO_BUFFER_MAX) {
    size = AIRPLAY_AUDIO_BUFFER_MAX;
  }
  if (size < AIRPLAY_AUDIO_BUFFER_MIN) {
    size = AIRPLAY_AUDIO_BUFFER_MIN;
  }
  return (uint32_t)size;
}

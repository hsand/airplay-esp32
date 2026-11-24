// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "audio_stream.h"
#include "plist.h"

/* No terminator. */
static const uint8_t BPLIST_MAGIC[8] = {'b', 'p', 'l', 'i', 's', 't', '0', '0'};

static bool bplist_has_room(size_t pos, size_t need, size_t capacity) {
  return pos <= capacity && need <= capacity - pos;
}

static bool bplist_write_u64(uint8_t *out, size_t capacity, size_t *pos,
                             uint64_t value) {
  if (!bplist_has_room(*pos, 8, capacity)) {
    return false;
  }
  for (int i = 7; i >= 0; i--) {
    out[(*pos)++] = (uint8_t)(value >> (i * 8));
  }
  return true;
}

static bool bplist_write_length(uint8_t *out, size_t capacity, size_t *pos,
                                uint8_t marker_base, size_t length) {
  if (length < 15) {
    if (!bplist_has_room(*pos, 1, capacity)) {
      return false;
    }
    out[(*pos)++] = marker_base | (uint8_t)length;
    return true;
  }
  if (length > UINT8_MAX || !bplist_has_room(*pos, 3, capacity)) {
    return false;
  }
  out[(*pos)++] = marker_base | 0x0F;
  out[(*pos)++] = 0x10;
  out[(*pos)++] = (uint8_t)length;
  return true;
}

static bool bplist_write_ascii_string(uint8_t *out, size_t capacity,
                                      size_t *pos, const char *value) {
  size_t len = strlen(value);
  if (!bplist_write_length(out, capacity, pos, 0x50, len) ||
      !bplist_has_room(*pos, len, capacity)) {
    return false;
  }
  // Length-prefixed, no terminator.
  // NOLINTNEXTLINE(bugprone-not-null-terminated-result)
  memcpy(out + *pos, value, len);
  *pos += len;
  return true;
}

static bool bplist_utf8_next(const char **cursor, uint32_t *codepoint) {
  const uint8_t *s = (const uint8_t *)*cursor;
  uint32_t cp;
  size_t n;

  if (s[0] < 0x80) {
    cp = s[0];
    n = 1;
  } else if ((s[0] & 0xE0) == 0xC0) {
    if (s[1] == 0 || (s[1] & 0xC0) != 0x80) {
      return false;
    }
    cp = ((uint32_t)(s[0] & 0x1F) << 6) | (uint32_t)(s[1] & 0x3F);
    if (cp < 0x80) {
      return false;
    }
    n = 2;
  } else if ((s[0] & 0xF0) == 0xE0) {
    if (s[1] == 0 || s[2] == 0 || (s[1] & 0xC0) != 0x80 ||
        (s[2] & 0xC0) != 0x80) {
      return false;
    }
    cp = ((uint32_t)(s[0] & 0x0F) << 12) | ((uint32_t)(s[1] & 0x3F) << 6) |
         (uint32_t)(s[2] & 0x3F);
    if (cp < 0x800 || (cp >= 0xD800 && cp <= 0xDFFF)) {
      return false;
    }
    n = 3;
  } else if ((s[0] & 0xF8) == 0xF0) {
    if (s[1] == 0 || s[2] == 0 || s[3] == 0 || (s[1] & 0xC0) != 0x80 ||
        (s[2] & 0xC0) != 0x80 || (s[3] & 0xC0) != 0x80) {
      return false;
    }
    cp = ((uint32_t)(s[0] & 0x07) << 18) | ((uint32_t)(s[1] & 0x3F) << 12) |
         ((uint32_t)(s[2] & 0x3F) << 6) | (uint32_t)(s[3] & 0x3F);
    if (cp < 0x10000 || cp > 0x10FFFF) {
      return false;
    }
    n = 4;
  } else {
    return false;
  }

  *cursor += n;
  *codepoint = cp;
  return true;
}

static bool bplist_write_utf8_string(uint8_t *out, size_t capacity, size_t *pos,
                                     const char *value) {
  bool ascii_only = true;
  for (const uint8_t *p = (const uint8_t *)value; *p; p++) {
    if (*p & 0x80) {
      ascii_only = false;
      break;
    }
  }
  if (ascii_only) {
    return bplist_write_ascii_string(out, capacity, pos, value);
  }

  size_t utf16_units = 0;
  const char *p = value;
  while (*p) {
    uint32_t cp;
    if (!bplist_utf8_next(&p, &cp)) {
      return false;
    }
    utf16_units += (cp <= 0xFFFF) ? 1 : 2;
  }

  if (!bplist_write_length(out, capacity, pos, 0x60, utf16_units) ||
      !bplist_has_room(*pos, utf16_units * 2, capacity)) {
    return false;
  }

  p = value;
  while (*p) {
    uint32_t cp;
    if (!bplist_utf8_next(&p, &cp)) {
      return false;
    }

    if (cp <= 0xFFFF) {
      out[(*pos)++] = (uint8_t)(cp >> 8);
      out[(*pos)++] = (uint8_t)cp;
    } else {
      cp -= 0x10000;
      uint16_t high = (uint16_t)(0xD800 + (cp >> 10));
      uint16_t low = (uint16_t)(0xDC00 + (cp & 0x3FF));
      out[(*pos)++] = (uint8_t)(high >> 8);
      out[(*pos)++] = (uint8_t)high;
      out[(*pos)++] = (uint8_t)(low >> 8);
      out[(*pos)++] = (uint8_t)low;
    }
  }
  return true;
}

static bool bplist_write_data(uint8_t *out, size_t capacity, size_t *pos,
                              const uint8_t *data, size_t len) {
  if (!bplist_write_length(out, capacity, pos, 0x40, len) ||
      !bplist_has_room(*pos, len, capacity)) {
    return false;
  }
  memcpy(out + *pos, data, len);
  *pos += len;
  return true;
}

static bool bplist_write_int(uint8_t *out, size_t capacity, size_t *pos,
                             uint64_t value) {
  uint8_t marker;
  size_t bytes;
  if (value <= UINT8_MAX) {
    marker = 0x10;
    bytes = 1;
  } else if (value <= UINT16_MAX) {
    marker = 0x11;
    bytes = 2;
  } else if (value <= UINT32_MAX) {
    marker = 0x12;
    bytes = 4;
  } else {
    marker = 0x13;
    bytes = 8;
  }

  if (!bplist_has_room(*pos, 1 + bytes, capacity)) {
    return false;
  }
  out[(*pos)++] = marker;
  for (int i = (int)bytes - 1; i >= 0; i--) {
    out[(*pos)++] = (uint8_t)(value >> (i * 8));
  }
  return true;
}

static bool bplist_write_refs(uint8_t *out, size_t capacity, size_t *pos,
                              const uint8_t *refs, size_t count) {
  if (!bplist_has_room(*pos, count, capacity)) {
    return false;
  }
  memcpy(out + *pos, refs, count);
  *pos += count;
  return true;
}

static bool bplist_write_array(uint8_t *out, size_t capacity, size_t *pos,
                               const uint8_t *refs, size_t count) {
  return bplist_write_length(out, capacity, pos, 0xA0, count) &&
         bplist_write_refs(out, capacity, pos, refs, count);
}

static bool bplist_write_dict(uint8_t *out, size_t capacity, size_t *pos,
                              const uint8_t *keys, const uint8_t *values,
                              size_t count) {
  return bplist_write_length(out, capacity, pos, 0xD0, count) &&
         bplist_write_refs(out, capacity, pos, keys, count) &&
         bplist_write_refs(out, capacity, pos, values, count);
}

static bool bplist_finish(uint8_t *out, size_t capacity, size_t *pos,
                          const size_t *offsets, size_t object_count,
                          size_t top_object) {
  size_t offset_table_offset = *pos;
  for (size_t i = 0; i < object_count; i++) {
    if (offsets[i] > UINT16_MAX || !bplist_has_room(*pos, 2, capacity)) {
      return false;
    }
    out[(*pos)++] = (uint8_t)(offsets[i] >> 8);
    out[(*pos)++] = (uint8_t)offsets[i];
  }

  if (!bplist_has_room(*pos, 32, capacity)) {
    return false;
  }
  memset(out + *pos, 0, 6);
  *pos += 6;
  out[(*pos)++] = 2; // offset size
  out[(*pos)++] = 1; // object ref size

  if (!bplist_write_u64(out, capacity, pos, object_count) ||
      !bplist_write_u64(out, capacity, pos, top_object) ||
      !bplist_write_u64(out, capacity, pos, offset_table_offset)) {
    return false;
  }
  return true;
}

size_t bplist_build_initial_setup(uint8_t *out, size_t capacity,
                                  uint16_t event_port) {
  if (capacity < 100) {
    return 0;
  }

  size_t pos = 0;
  memcpy(out + pos, BPLIST_MAGIC, sizeof(BPLIST_MAGIC));
  pos += 8;

  size_t offsets[10];
  size_t obj = 0;

  offsets[obj++] = pos;
  out[pos++] = 0x59;
  memcpy(out + pos, "eventPort", 9);
  pos += 9;

  offsets[obj++] = pos;
  out[pos++] = 0x5A;
  memcpy(out + pos, "timingPort", 10);
  pos += 10;

  offsets[obj++] = pos;
  out[pos++] = 0x11;
  out[pos++] = (event_port >> 8) & 0xFF;
  out[pos++] = event_port & 0xFF;

  offsets[obj++] = pos;
  out[pos++] = 0x10;
  out[pos++] = 0;

  offsets[obj++] = pos;
  out[pos++] = 0xD2;
  out[pos++] = 0;
  out[pos++] = 1;
  out[pos++] = 2;
  out[pos++] = 3;

  size_t offset_table_offset = pos;
  for (size_t i = 0; i < obj; i++) {
    if (offsets[i] > 0xFF) {
      return 0;
    }
    out[pos++] = (uint8_t)offsets[i];
  }

  memset(out + pos, 0, 6);
  pos += 6;
  out[pos++] = 1;
  out[pos++] = 1;

  for (int i = 0; i < 7; i++) {
    out[pos++] = 0;
  }
  out[pos++] = (uint8_t)obj;

  for (int i = 0; i < 7; i++) {
    out[pos++] = 0;
  }
  out[pos++] = 4;

  for (int i = 0; i < 7; i++) {
    out[pos++] = 0;
  }
  out[pos++] = (uint8_t)offset_table_offset;

  return pos;
}

size_t bplist_build_stream_setup(uint8_t *out, size_t capacity,
                                 int64_t stream_type, uint16_t data_port,
                                 uint16_t control_port,
                                 uint32_t audio_buffer_size,
                                 int64_t arrival_to_render_ms) {
  if (!out || capacity < sizeof(BPLIST_MAGIC)) {
    return 0;
  }
  /* Objects, in order:
   *  0 "streams"  1 "type"  2 "dataPort"  3 "controlPort"
   *  4 "audioBufferSize" (buffered) or "arrivalToRenderLatencyMs" (realtime,
   *    when given)
   *  5 type  6 data port  7 control port  8 the value for 4
   *  9 {1: 5, 2: 6, 3: 7[, 4: 8]}  10 [9]  11 {0: 10}  (top) */
  enum { OBJECTS = 12 };
  const bool buffered =
      audio_stream_uses_buffer((audio_stream_type_t)stream_type);
  const bool extra = buffered || arrival_to_render_ms >= 0;
  size_t offsets[OBJECTS];
  size_t obj = 0;
  size_t pos = 0;
  memcpy(out, BPLIST_MAGIC, sizeof(BPLIST_MAGIC));
  pos += sizeof(BPLIST_MAGIC);

  const char *const keys[] = {"streams", "type", "dataPort", "controlPort",
                              buffered ? "audioBufferSize"
                                       : "arrivalToRenderLatencyMs"};
  for (size_t i = 0; i < 5; i++) {
    offsets[obj++] = pos;
    if (!bplist_write_ascii_string(out, capacity, &pos, keys[i])) {
      return 0;
    }
  }
  const uint64_t values[] = {
      (uint64_t)stream_type, data_port, control_port,
      buffered ? audio_buffer_size
               : (uint64_t)(extra ? arrival_to_render_ms : 0)};
  for (size_t i = 0; i < 4; i++) {
    offsets[obj++] = pos;
    if (!bplist_write_int(out, capacity, &pos, values[i])) {
      return 0;
    }
  }
  static const uint8_t stream_keys[] = {1, 2, 3, 4};
  static const uint8_t stream_values[] = {5, 6, 7, 8};
  offsets[obj++] = pos;
  if (!bplist_write_dict(out, capacity, &pos, stream_keys, stream_values,
                         extra ? 4U : 3U)) {
    return 0;
  }
  static const uint8_t stream_list[] = {9};
  offsets[obj++] = pos;
  if (!bplist_write_array(out, capacity, &pos, stream_list, 1)) {
    return 0;
  }
  static const uint8_t top_keys[] = {0};
  static const uint8_t top_values[] = {10};
  offsets[obj++] = pos;
  if (!bplist_write_dict(out, capacity, &pos, top_keys, top_values, 1) ||
      !bplist_finish(out, capacity, &pos, offsets, obj, obj - 1U)) {
    return 0;
  }
  return pos;
}

size_t bplist_build_feedback_response(uint8_t *out, size_t capacity,
                                      int64_t stream_type, double sample_rate) {
  // Feedback response for buffered audio streams (type 103)
  // Format: { streams: [ { type: 103, sr: 44100.0 } ] }
  // This acts as a keepalive mechanism to prevent iPhone from
  // sending TEARDOWN during extended pause
  if (capacity < 100) {
    return 0;
  }

  size_t pos = 0;
  memcpy(out + pos, BPLIST_MAGIC, sizeof(BPLIST_MAGIC));
  pos += 8;

  size_t offsets[10];
  size_t obj = 0;

  // Object 0: "streams" key string
  offsets[obj++] = pos;
  out[pos++] = 0x57; // String, length 7
  memcpy(out + pos, "streams", 7);
  pos += 7;

  // Object 1: "type" key string
  offsets[obj++] = pos;
  out[pos++] = 0x54; // String, length 4
  memcpy(out + pos, "type", 4);
  pos += 4;

  // Object 2: "sr" key string (sample rate)
  offsets[obj++] = pos;
  out[pos++] = 0x52; // String, length 2
  memcpy(out + pos, "sr", 2);
  pos += 2;

  // Object 3: type value (103 for buffered audio)
  offsets[obj++] = pos;
  out[pos++] = 0x10; // Int, 1 byte
  out[pos++] = (uint8_t)stream_type;

  // Object 4: sample rate value as double
  // IEEE 754 double-precision big-endian
  offsets[obj++] = pos;
  out[pos++] = 0x23; // Real, 8 bytes (double)
  union {
    double d;
    uint8_t bytes[8];
  } sr;
  sr.d = sample_rate;
  // Convert to big-endian
  for (int i = 7; i >= 0; i--) {
    out[pos++] = sr.bytes[i];
  }

  // Object 5: stream dict { type: 3, sr: 4 }
  offsets[obj++] = pos;
  out[pos++] = 0xD2; // Dict, 2 key-value pairs
  out[pos++] = 1;    // Key: object 1 (type)
  out[pos++] = 2;    // Key: object 2 (sr)
  out[pos++] = 3;    // Value: object 3 (type value)
  out[pos++] = 4;    // Value: object 4 (sr value)

  // Object 6: streams array [ object 5 ]
  offsets[obj++] = pos;
  out[pos++] = 0xA1; // Array, 1 element
  out[pos++] = 5;    // Contains object 5 (stream dict)

  // Object 7: top-level dict { streams: 6 }
  offsets[obj++] = pos;
  out[pos++] = 0xD1; // Dict, 1 key-value pair
  out[pos++] = 0;    // Key: object 0 (streams)
  out[pos++] = 6;    // Value: object 6 (streams array)

  // Offset table
  size_t offset_table_offset = pos;
  for (size_t i = 0; i < obj; i++) {
    if (offsets[i] > 0xFF) {
      return 0;
    }
    out[pos++] = (uint8_t)offsets[i];
  }

  // Trailer: 6 unused bytes, then metadata
  memset(out + pos, 0, 6);
  pos += 6;
  out[pos++] = 1; // Offset size
  out[pos++] = 1; // Object ref size

  // Number of objects (8 bytes big-endian)
  for (int i = 0; i < 7; i++) {
    out[pos++] = 0;
  }
  out[pos++] = (uint8_t)obj;

  // Top object index (8 bytes big-endian)
  for (int i = 0; i < 7; i++) {
    out[pos++] = 0;
  }
  out[pos++] = 7; // Top object is object 7

  // Offset table offset (8 bytes big-endian)
  for (int i = 0; i < 7; i++) {
    out[pos++] = 0;
  }
  out[pos++] = (uint8_t)offset_table_offset;

  return pos;
}

// ============================================================================
// Generic writer: objects are appended as they are made and referenced by
// index, so a builder reads top-down instead of by hand-numbered offsets.
// One-byte object references: at most 255 objects.
// ============================================================================

#define BPW_MAX_OBJECTS 64

typedef struct {
  uint8_t *out;
  size_t capacity;
  size_t pos;
  size_t offsets[BPW_MAX_OBJECTS];
  size_t count;
  bool ok;
} bpw_t;

static void bpw_init(bpw_t *w, uint8_t *out, size_t capacity) {
  w->out = out;
  w->capacity = capacity;
  w->pos = 0;
  w->count = 0;
  w->ok = out != NULL && bplist_has_room(0, sizeof(BPLIST_MAGIC), capacity);
  if (w->ok) {
    memcpy(out, BPLIST_MAGIC, sizeof(BPLIST_MAGIC));
    w->pos = sizeof(BPLIST_MAGIC);
  }
}

// Reserve the next object index; 0 with w->ok cleared once full.
static uint8_t bpw_begin(bpw_t *w) {
  if (!w->ok || w->count >= BPW_MAX_OBJECTS) {
    w->ok = false;
    return 0;
  }
  w->offsets[w->count] = w->pos;
  return (uint8_t)w->count++;
}

static uint8_t bpw_string(bpw_t *w, const char *value) {
  const uint8_t ref = bpw_begin(w);
  w->ok = w->ok && value &&
          bplist_write_utf8_string(w->out, w->capacity, &w->pos, value);
  return ref;
}

static uint8_t bpw_int(bpw_t *w, int64_t value) {
  const uint8_t ref = bpw_begin(w);
  w->ok =
      w->ok && bplist_write_int(w->out, w->capacity, &w->pos, (uint64_t)value);
  return ref;
}

static uint8_t bpw_real(bpw_t *w, double value) {
  const uint8_t ref = bpw_begin(w);
  uint64_t bits;
  memcpy(&bits, &value, sizeof(bits));
  w->ok = w->ok && bplist_has_room(w->pos, 1, w->capacity);
  if (w->ok) {
    w->out[w->pos++] = 0x23; // real, 2^3 bytes
    w->ok = bplist_write_u64(w->out, w->capacity, &w->pos, bits);
  }
  return ref;
}

static uint8_t bpw_bool(bpw_t *w, bool value) {
  const uint8_t ref = bpw_begin(w);
  w->ok = w->ok && bplist_has_room(w->pos, 1, w->capacity);
  if (w->ok) {
    w->out[w->pos++] = value ? 0x09 : 0x08;
  }
  return ref;
}

static uint8_t bpw_data(bpw_t *w, const uint8_t *data, size_t len) {
  const uint8_t ref = bpw_begin(w);
  w->ok = w->ok && data &&
          bplist_write_data(w->out, w->capacity, &w->pos, data, len);
  return ref;
}

static uint8_t bpw_array(bpw_t *w, const uint8_t *refs, size_t count) {
  const uint8_t ref = bpw_begin(w);
  w->ok =
      w->ok && bplist_write_array(w->out, w->capacity, &w->pos, refs, count);
  return ref;
}

static uint8_t bpw_dict(bpw_t *w, const uint8_t *keys, const uint8_t *values,
                        size_t count) {
  const uint8_t ref = bpw_begin(w);
  w->ok = w->ok &&
          bplist_write_dict(w->out, w->capacity, &w->pos, keys, values, count);
  return ref;
}

static size_t bpw_finish(bpw_t *w, uint8_t top) {
  if (!w->ok ||
      !bplist_finish(w->out, w->capacity, &w->pos, w->offsets, w->count, top)) {
    return 0;
  }
  return w->pos;
}

// A dict under construction: key and value references side by side.
#define BPW_DICT_MAX 32
typedef struct {
  uint8_t keys[BPW_DICT_MAX];
  uint8_t values[BPW_DICT_MAX];
  size_t count;
} bpw_dict_t;

static void bpw_entry(bpw_t *w, bpw_dict_t *d, uint8_t key, uint8_t value) {
  if (d->count >= BPW_DICT_MAX) {
    w->ok = false;
    return;
  }
  d->keys[d->count] = key;
  d->values[d->count] = value;
  d->count++;
}

size_t bplist_build_info(uint8_t *out, size_t capacity,
                         const airplay_info_t *info) {
  if (!out || !info || !info->device_id || !info->device_name || !info->model ||
      !info->pi || !info->source_version || !info->pk || info->pk_len == 0) {
    return 0;
  }
  bpw_t w;
  bpw_init(&w, out, capacity);
  bpw_dict_t top = {.count = 0};

  // The keys this receiver always sent, for senders that read them...
  bpw_entry(&w, &top, bpw_string(&w, "deviceid"),
            bpw_string(&w, info->device_id));
  const uint8_t features = bpw_int(&w, (int64_t)info->features);
  bpw_entry(&w, &top, bpw_string(&w, "features"), features);
  bpw_entry(&w, &top, bpw_string(&w, "model"), bpw_string(&w, info->model));
  const uint8_t protocol = bpw_string(&w, "1.1");
  bpw_entry(&w, &top, bpw_string(&w, "protovers"), protocol);
  const uint8_t source = bpw_string(&w, info->source_version);
  bpw_entry(&w, &top, bpw_string(&w, "srcvers"), source);
  bpw_entry(&w, &top, bpw_string(&w, "vv"), bpw_int(&w, info->vv));
  bpw_entry(&w, &top, bpw_string(&w, "statusFlags"),
            bpw_int(&w, info->status_flags));
  bpw_entry(&w, &top, bpw_string(&w, "pk"),
            bpw_data(&w, info->pk, info->pk_len));
  bpw_entry(&w, &top, bpw_string(&w, "pi"), bpw_string(&w, info->pi));
  bpw_entry(&w, &top, bpw_string(&w, "name"),
            bpw_string(&w, info->device_name));

  // ...and the names Apple's receiver uses, which are the ones Apple's
  // sender reads (airPlayDescription_SetEndpointInfo).
  bpw_entry(&w, &top, bpw_string(&w, "deviceID"),
            bpw_string(&w, info->device_id));
  bpw_entry(&w, &top, bpw_string(&w, "protocolVersion"), protocol);
  bpw_entry(&w, &top, bpw_string(&w, "sourceVersion"), source);
  if (info->features_ex && info->features_ex[0]) {
    bpw_entry(&w, &top, bpw_string(&w, "featuresEx"),
              bpw_string(&w, info->features_ex));
  }
  if (info->has_initial_volume) {
    bpw_entry(&w, &top, bpw_string(&w, "initialVolume"),
              bpw_real(&w, info->initial_volume_db));
  }

  // audioFormats: [{type: 96, audioInputFormats, audioOutputFormats}]
  const uint8_t type_key = bpw_string(&w, "type");
  const uint8_t type96 = bpw_int(&w, 96);
  const uint8_t mask = bpw_int(&w, 0x01000000);
  {
    bpw_dict_t f = {.count = 0};
    bpw_entry(&w, &f, type_key, type96);
    bpw_entry(&w, &f, bpw_string(&w, "audioInputFormats"), mask);
    bpw_entry(&w, &f, bpw_string(&w, "audioOutputFormats"), mask);
    const uint8_t dict = bpw_dict(&w, f.keys, f.values, f.count);
    const uint8_t list = bpw_array(&w, &dict, 1);
    bpw_entry(&w, &top, bpw_string(&w, "audioFormats"), list);
  }
  // audioLatencies: zero for both stream types (see rtsp_handlers.c).
  {
    const uint8_t audio_type_key = bpw_string(&w, "audioType");
    const uint8_t input_key = bpw_string(&w, "inputLatencyMicros");
    const uint8_t output_key = bpw_string(&w, "outputLatencyMicros");
    const uint8_t audio_type = bpw_int(&w, 0x64);
    const uint8_t zero = bpw_int(&w, 0);
    const uint8_t types[2] = {type96, bpw_int(&w, 103)};
    uint8_t dicts[2];
    for (size_t i = 0; i < 2; i++) {
      bpw_dict_t l = {.count = 0};
      bpw_entry(&w, &l, type_key, types[i]);
      bpw_entry(&w, &l, audio_type_key, audio_type);
      bpw_entry(&w, &l, input_key, zero);
      bpw_entry(&w, &l, output_key, zero);
      dicts[i] = bpw_dict(&w, l.keys, l.values, l.count);
    }
    const uint8_t list = bpw_array(&w, dicts, 2);
    bpw_entry(&w, &top, bpw_string(&w, "audioLatencies"), list);
  }

  const uint8_t root = bpw_dict(&w, top.keys, top.values, top.count);
  return bpw_finish(&w, root);
}

size_t bplist_build_mr_command(uint8_t *out, size_t capacity,
                               const airplay_mr_command_plist_t *cmd) {
  if (!out || !cmd || !cmd->fourcc) {
    return 0;
  }
  bpw_t w;
  bpw_init(&w, out, capacity);
  bpw_dict_t top = {.count = 0};
  bpw_entry(&w, &top, bpw_string(&w, "type"),
            bpw_string(&w, "sendMediaRemoteCommand"));
  bpw_entry(&w, &top, bpw_string(&w, "value"), bpw_string(&w, cmd->fourcc));
  if (cmd->mr_command >= 0) {
    // A decimal string: the sender reads it with TextToInt32.
    char number[12];
    snprintf(number, sizeof(number), "%d", cmd->mr_command);
    bpw_entry(&w, &top, bpw_string(&w, "modernMediaRemoteCommand"),
              bpw_string(&w, number));
  }
  if (cmd->has_volume) {
    // dvlc: the options are merged into the top level and also sent as
    // params, as Apple's receiver does.
    const uint8_t volume_key = bpw_string(&w, "volume");
    const uint8_t volume = bpw_real(&w, cmd->volume);
    const uint8_t muted_key = bpw_string(&w, "isMuted");
    const uint8_t muted = bpw_bool(&w, cmd->is_muted);
    bpw_dict_t params = {.count = 0};
    bpw_entry(&w, &params, volume_key, volume);
    bpw_entry(&w, &params, muted_key, muted);
    bpw_entry(&w, &top, volume_key, volume);
    bpw_entry(&w, &top, muted_key, muted);
    bpw_entry(&w, &top, bpw_string(&w, "params"),
              bpw_dict(&w, params.keys, params.values, params.count));
  }
  const uint8_t root = bpw_dict(&w, top.keys, top.values, top.count);
  return bpw_finish(&w, root);
}

size_t bplist_build_session_setup(uint8_t *out, size_t capacity,
                                  uint16_t event_port, const char *address,
                                  const char *peer_id) {
  if (!out || !address || !peer_id || capacity < sizeof(BPLIST_MAGIC)) {
    return 0;
  }
  /* Objects, in order:
   *  0 "eventPort"   1 "timingPort"   2 "timingPeerInfo"
   *  3 event_port    4 0
   *  5 "Addresses"   6 "ID"           7 address   8 [7]   9 peer_id
   * 10 {5: 8, 6: 9}
   * 11 {0: 3, 1: 4, 2: 10}  (top) */
  enum { OBJECTS = 12 };
  size_t offsets[OBJECTS];
  size_t obj = 0;
  size_t pos = 0;
  memcpy(out, BPLIST_MAGIC, sizeof(BPLIST_MAGIC));
  pos += sizeof(BPLIST_MAGIC);

  static const char *const strings[] = {"eventPort", "timingPort",
                                        "timingPeerInfo"};
  for (size_t i = 0; i < 3; i++) {
    offsets[obj++] = pos;
    if (!bplist_write_ascii_string(out, capacity, &pos, strings[i])) {
      return 0;
    }
  }
  offsets[obj++] = pos;
  if (!bplist_write_int(out, capacity, &pos, event_port)) {
    return 0;
  }
  offsets[obj++] = pos;
  if (!bplist_write_int(out, capacity, &pos, 0)) {
    return 0;
  }
  offsets[obj++] = pos;
  if (!bplist_write_ascii_string(out, capacity, &pos, "Addresses")) {
    return 0;
  }
  offsets[obj++] = pos;
  if (!bplist_write_ascii_string(out, capacity, &pos, "ID")) {
    return 0;
  }
  offsets[obj++] = pos;
  if (!bplist_write_utf8_string(out, capacity, &pos, address)) {
    return 0;
  }
  static const uint8_t address_list[] = {7};
  offsets[obj++] = pos;
  if (!bplist_write_array(out, capacity, &pos, address_list, 1)) {
    return 0;
  }
  offsets[obj++] = pos;
  if (!bplist_write_utf8_string(out, capacity, &pos, peer_id)) {
    return 0;
  }
  static const uint8_t info_keys[] = {5, 6};
  static const uint8_t info_values[] = {8, 9};
  offsets[obj++] = pos;
  if (!bplist_write_dict(out, capacity, &pos, info_keys, info_values, 2)) {
    return 0;
  }
  static const uint8_t top_keys[] = {0, 1, 2};
  static const uint8_t top_values[] = {3, 4, 10};
  offsets[obj++] = pos;
  if (!bplist_write_dict(out, capacity, &pos, top_keys, top_values, 3) ||
      !bplist_finish(out, capacity, &pos, offsets, obj, obj - 1U)) {
    return 0;
  }
  return pos;
}

size_t bplist_build_status(uint8_t *out, size_t capacity, int64_t status) {
  if (!out || capacity < sizeof(BPLIST_MAGIC)) {
    return 0;
  }
  size_t offsets[3];
  size_t pos = 0;
  memcpy(out, BPLIST_MAGIC, sizeof(BPLIST_MAGIC));
  pos += sizeof(BPLIST_MAGIC);
  offsets[0] = pos;
  if (!bplist_write_ascii_string(out, capacity, &pos, "status")) {
    return 0;
  }
  offsets[1] = pos;
  /* A negative OSStatus goes out as the 8-byte two's complement, which is
   * how a binary plist stores any negative integer. */
  if (!bplist_write_int(out, capacity, &pos, (uint64_t)status)) {
    return 0;
  }
  static const uint8_t keys[] = {0};
  static const uint8_t values[] = {1};
  offsets[2] = pos;
  if (!bplist_write_dict(out, capacity, &pos, keys, values, 1) ||
      !bplist_finish(out, capacity, &pos, offsets, 3, 2)) {
    return 0;
  }
  return pos;
}

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <inttypes.h>
#include <string.h>

#include "plist.h"

// Binary plist object types (high nibble of marker byte)
#define BPLIST_NULL    0x00
#define BPLIST_BOOL    0x00
#define BPLIST_INT     0x10
#define BPLIST_REAL    0x20
#define BPLIST_DATE    0x30
#define BPLIST_DATA    0x40
#define BPLIST_STRING  0x50
#define BPLIST_UNICODE 0x60
#define BPLIST_UID     0x80
#define BPLIST_ARRAY   0xA0
#define BPLIST_SET     0xC0
#define BPLIST_DICT    0xD0

#define BPLIST_HEADER_LEN  8U
#define BPLIST_TRAILER_LEN 32U

// ---------------------------------------------------------------------------
// A binary plist is "bplist00", the objects, an offset table holding each
// object's position, and a 32-byte trailer.  The plists parsed here arrive
// from the network, some of them before pairing, so nothing in them is
// trusted: bplist_open() checks the trailer against the buffer once, and
// every object, length and reference list is then checked against the object
// area (between the header and the offset table) before it is read.  Counts
// and lengths can be as large as 2^64 - 1, so the checks divide rather than
// multiply.
// ---------------------------------------------------------------------------

typedef struct {
  const uint8_t *data;
  size_t objects_end; // where the offset table starts
  uint8_t offset_size;
  uint8_t ref_size;
  uint64_t num_objects;
  uint64_t top_object;
} bplist_t;

static uint64_t read_be_int(const uint8_t *data, size_t bytes) {
  uint64_t val = 0;
  for (size_t i = 0; i < bytes; i++) {
    val = (val << 8) | data[i];
  }
  return val;
}

static bool bplist_open(const uint8_t *plist, size_t plist_len, bplist_t *bp) {
  if (!plist || plist_len < BPLIST_HEADER_LEN + BPLIST_TRAILER_LEN ||
      memcmp(plist, "bplist00", BPLIST_HEADER_LEN) != 0) {
    return false;
  }

  const uint8_t *trailer = plist + plist_len - BPLIST_TRAILER_LEN;
  const uint8_t offset_size = trailer[6];
  const uint8_t ref_size = trailer[7];
  const uint64_t num_objects = read_be_int(trailer + 8, 8);
  const uint64_t top_object = read_be_int(trailer + 16, 8);
  const uint64_t table = read_be_int(trailer + 24, 8);
  const size_t table_end = plist_len - BPLIST_TRAILER_LEN;

  if (offset_size < 1 || offset_size > 8 || ref_size < 1 || ref_size > 8) {
    return false;
  }
  // At least one byte of objects, and the whole offset table before the
  // trailer.
  if (table <= BPLIST_HEADER_LEN || table > table_end) {
    return false;
  }
  if (num_objects == 0 ||
      num_objects > (table_end - (size_t)table) / offset_size ||
      top_object >= num_objects) {
    return false;
  }

  bp->data = plist;
  bp->objects_end = (size_t)table;
  bp->offset_size = offset_size;
  bp->ref_size = ref_size;
  bp->num_objects = num_objects;
  bp->top_object = top_object;
  return true;
}

// The position of object `index`, which must lie in the object area.
static bool bp_offset(const bplist_t *bp, uint64_t index, size_t *offset) {
  if (index >= bp->num_objects) {
    return false;
  }
  const uint64_t value =
      read_be_int(bp->data + bp->objects_end + (size_t)index * bp->offset_size,
                  bp->offset_size);
  if (value < BPLIST_HEADER_LEN || value >= bp->objects_end) {
    return false;
  }
  *offset = (size_t)value;
  return true;
}

// True when `count` items of `unit` bytes fit between `pos` and the end of
// the object area.
static bool bp_fits(const bplist_t *bp, size_t pos, uint64_t count,
                    size_t unit) {
  return pos <= bp->objects_end && count <= (bp->objects_end - pos) / unit;
}

// The marker type of a data, string or container object and its count: the
// marker's low nibble, or, when that is 0xF, the integer that follows it.
// *body is where the object's contents start.
static bool bp_object(const bplist_t *bp, size_t offset, uint8_t *type,
                      uint64_t *count, size_t *body) {
  if (offset >= bp->objects_end) {
    return false;
  }
  const uint8_t marker = bp->data[offset];
  size_t pos = offset + 1;
  *type = marker & 0xF0;
  *count = marker & 0x0F;
  if (*count == 0x0F) {
    if (pos >= bp->objects_end) {
      return false;
    }
    const uint8_t len_marker = bp->data[pos++];
    if ((len_marker & 0xF0) != BPLIST_INT || (len_marker & 0x0F) > 3) {
      return false;
    }
    const size_t len_bytes = (size_t)1 << (len_marker & 0x0F);
    if (len_bytes > bp->objects_end - pos) {
      return false;
    }
    *count = read_be_int(bp->data + pos, len_bytes);
    pos += len_bytes;
  }
  *body = pos;
  return true;
}

static bool bp_read_string(const bplist_t *bp, size_t offset, char *out,
                           size_t out_capacity) {
  uint8_t type = 0;
  uint64_t len = 0;
  size_t pos = 0;
  if (!bp_object(bp, offset, &type, &len, &pos)) {
    return false;
  }

  if (type == BPLIST_STRING) {
    if (!bp_fits(bp, pos, len, 1) || len >= out_capacity) {
      return false;
    }
    memcpy(out, bp->data + pos, (size_t)len);
    out[len] = '\0';
    return true;
  }

  if (type == BPLIST_UNICODE) {
    if (!bp_fits(bp, pos, len, 2) || len >= out_capacity) {
      return false;
    }
    for (size_t i = 0; i < len; i++) {
      uint16_t code =
          (uint16_t)(bp->data[pos + i * 2] << 8) | bp->data[pos + i * 2 + 1];
      if (code > 0x7F) {
        return false;
      }
      out[i] = (char)code;
    }
    out[len] = '\0';
    return true;
  }

  return false;
}

static bool bp_read_string_len(const bplist_t *bp, size_t offset,
                               size_t *out_len) {
  uint8_t type = 0;
  uint64_t len = 0;
  size_t pos = 0;
  if (!bp_object(bp, offset, &type, &len, &pos)) {
    return false;
  }
  if ((type == BPLIST_STRING && bp_fits(bp, pos, len, 1)) ||
      (type == BPLIST_UNICODE && bp_fits(bp, pos, len, 2))) {
    *out_len = (size_t)len;
    return true;
  }
  return false;
}

// A data object's contents; out may be NULL to ask for the length only.
static bool bp_read_data(const bplist_t *bp, size_t offset, uint8_t *out,
                         size_t out_capacity, size_t *out_len) {
  uint8_t type = 0;
  uint64_t len = 0;
  size_t pos = 0;
  if (!bp_object(bp, offset, &type, &len, &pos) || type != BPLIST_DATA ||
      !bp_fits(bp, pos, len, 1)) {
    return false;
  }
  if (out) {
    if (len > out_capacity) {
      return false;
    }
    memcpy(out, bp->data + pos, (size_t)len);
  }
  *out_len = (size_t)len;
  return true;
}

// An integer, or a boolean as 0 or 1, as CFDictionaryGetInt64 reads one:
// Apple sends some flags (SETUP's isRemoteControlOnly) as booleans.
static bool bp_read_int(const bplist_t *bp, size_t offset, int64_t *out) {
  if (offset >= bp->objects_end) {
    return false;
  }
  const uint8_t marker = bp->data[offset];
  if (marker == 0x08 || marker == 0x09) {
    *out = marker == 0x09;
    return true;
  }
  // 1 to 8 bytes, or 16 for an unsigned value above INT64_MAX (whose low 8
  // bytes are returned).
  if ((marker & 0xF0) != BPLIST_INT || (marker & 0x0F) > 4) {
    return false;
  }
  const size_t len = (size_t)1 << (marker & 0x0F);
  if (len > bp->objects_end - offset - 1) {
    return false;
  }
  *out = (int64_t)read_be_int(bp->data + offset + 1, len);
  return true;
}

static bool bp_read_real(const bplist_t *bp, size_t offset, double *out) {
  if (offset >= bp->objects_end) {
    return false;
  }
  const uint8_t marker = bp->data[offset];
  if ((marker & 0xF0) != BPLIST_REAL) {
    return false;
  }
  const size_t len = (size_t)1 << (marker & 0x0F);
  if ((len != 4 && len != 8) || len > bp->objects_end - offset - 1) {
    return false;
  }
  if (len == 4) {
    uint32_t bits = (uint32_t)read_be_int(bp->data + offset + 1, 4);
    float f;
    memcpy(&f, &bits, sizeof(f));
    *out = (double)f;
  } else {
    uint64_t bits = read_be_int(bp->data + offset + 1, 8);
    memcpy(out, &bits, sizeof(*out));
  }
  return true;
}

// A dict, array or set: its type, its count, and its reference list.  A
// dict's list is the n key references followed by the n value references.
static bool bp_container(const bplist_t *bp, size_t offset, uint8_t *type,
                         size_t *n, const uint8_t **refs) {
  uint64_t count = 0;
  size_t pos = 0;
  if (!bp_object(bp, offset, type, &count, &pos)) {
    return false;
  }
  size_t per = 0;
  if (*type == BPLIST_DICT) {
    per = 2;
  } else if (*type == BPLIST_ARRAY || *type == BPLIST_SET) {
    per = 1;
  } else {
    return false;
  }
  if (!bp_fits(bp, pos, count, per * bp->ref_size)) {
    return false;
  }
  *n = (size_t)count;
  *refs = bp->data + pos;
  return true;
}

// Reference i of a list bp_container() checked.
static uint64_t bp_ref(const bplist_t *bp, const uint8_t *refs, size_t i) {
  return read_be_int(refs + i * bp->ref_size, bp->ref_size);
}

// The value reference for `key` in the dict at `offset`.
static bool bp_dict_lookup_at(const bplist_t *bp, size_t offset,
                              const char *key, uint64_t *value) {
  uint8_t type = 0;
  size_t n = 0;
  const uint8_t *refs = NULL;
  if (!bp_container(bp, offset, &type, &n, &refs) || type != BPLIST_DICT) {
    return false;
  }
  for (size_t i = 0; i < n; i++) {
    size_t key_offset = 0;
    char name[64];
    if (bp_offset(bp, bp_ref(bp, refs, i), &key_offset) &&
        bp_read_string(bp, key_offset, name, sizeof(name)) &&
        strcmp(name, key) == 0) {
      *value = bp_ref(bp, refs, n + i);
      return true;
    }
  }
  return false;
}

// The value reference for `key` in the dict that is object `index`.
static bool bp_dict_lookup(const bplist_t *bp, uint64_t index, const char *key,
                           uint64_t *value) {
  size_t offset = 0;
  return bp_offset(bp, index, &offset) &&
         bp_dict_lookup_at(bp, offset, key, value);
}

// The position of the value of `key` in the top-level dict.
static bool bp_find_top(const uint8_t *plist, size_t plist_len, const char *key,
                        bplist_t *bp, size_t *offset) {
  uint64_t value = 0;
  return bplist_open(plist, plist_len, bp) &&
         bp_dict_lookup(bp, bp->top_object, key, &value) &&
         bp_offset(bp, value, offset);
}

bool bplist_find_data(const uint8_t *plist, size_t plist_len, const char *key,
                      uint8_t *out_data, size_t out_capacity, size_t *out_len) {
  bplist_t bp;
  size_t offset = 0;
  return out_data && out_len &&
         bp_find_top(plist, plist_len, key, &bp, &offset) &&
         bp_read_data(&bp, offset, out_data, out_capacity, out_len);
}

// Bounds on the deep search, so a plist whose references fan out or loop
// cannot keep the RTSP task busy.
#define FIND_DEEP_MAX_DEPTH  10
#define FIND_DEEP_MAX_VISITS 1024U

static bool bp_find_data_recursive(const bplist_t *bp, uint64_t obj_idx,
                                   const char *key, uint8_t *out_data,
                                   size_t out_capacity, size_t *out_len,
                                   int depth, unsigned *visits) {
  size_t offset = 0;
  uint8_t type = 0;
  size_t n = 0;
  const uint8_t *refs = NULL;
  if (depth > FIND_DEEP_MAX_DEPTH || ++*visits > FIND_DEEP_MAX_VISITS ||
      !bp_offset(bp, obj_idx, &offset) ||
      !bp_container(bp, offset, &type, &n, &refs)) {
    return false;
  }

  // A dict's own keys first, then each value in turn.
  const uint8_t *children = refs;
  if (type == BPLIST_DICT) {
    uint64_t value = 0;
    size_t value_offset = 0;
    if (bp_dict_lookup_at(bp, offset, key, &value)) {
      return bp_offset(bp, value, &value_offset) &&
             bp_read_data(bp, value_offset, out_data, out_capacity, out_len);
    }
    children = refs + n * bp->ref_size;
  }
  for (size_t i = 0; i < n; i++) {
    if (bp_find_data_recursive(bp, bp_ref(bp, children, i), key, out_data,
                               out_capacity, out_len, depth + 1, visits)) {
      return true;
    }
  }
  return false;
}

bool bplist_find_data_deep(const uint8_t *plist, size_t plist_len,
                           const char *key, uint8_t *out_data,
                           size_t out_capacity, size_t *out_len) {
  bplist_t bp;
  unsigned visits = 0;
  return out_data && out_len && bplist_open(plist, plist_len, &bp) &&
         bp_find_data_recursive(&bp, bp.top_object, key, out_data, out_capacity,
                                out_len, 0, &visits);
}

bool bplist_find_int(const uint8_t *plist, size_t plist_len, const char *key,
                     int64_t *out_value) {
  bplist_t bp;
  size_t offset = 0;
  return out_value && bp_find_top(plist, plist_len, key, &bp, &offset) &&
         bp_read_int(&bp, offset, out_value);
}

bool bplist_find_real(const uint8_t *plist, size_t plist_len, const char *key,
                      double *out_value) {
  bplist_t bp;
  size_t offset = 0;
  if (!out_value || !bp_find_top(plist, plist_len, key, &bp, &offset)) {
    return false;
  }
  if (bp_read_real(&bp, offset, out_value)) {
    return true;
  }
  int64_t int_val = 0;
  if (bp_read_int(&bp, offset, &int_val)) {
    *out_value = (double)int_val;
    return true;
  }
  return false;
}

bool bplist_find_string(const uint8_t *plist, size_t plist_len, const char *key,
                        char *out_str, size_t out_capacity) {
  bplist_t bp;
  size_t offset = 0;
  return out_str && bp_find_top(plist, plist_len, key, &bp, &offset) &&
         bp_read_string(&bp, offset, out_str, out_capacity);
}

// ---------------------------------------------------------------------------
// The "streams" array of a SETUP request: [{type, ekey?, eiv?, shk?, ...}]
// ---------------------------------------------------------------------------

static bool bp_streams(const uint8_t *plist, size_t plist_len, bplist_t *bp,
                       size_t *count, const uint8_t **refs) {
  size_t offset = 0;
  uint8_t type = 0;
  return bp_find_top(plist, plist_len, "streams", bp, &offset) &&
         bp_container(bp, offset, &type, count, refs) && type == BPLIST_ARRAY;
}

// The dict of stream `index`.
static bool bp_stream_dict(const uint8_t *plist, size_t plist_len, size_t index,
                           bplist_t *bp, size_t *n, const uint8_t **refs) {
  size_t count = 0;
  const uint8_t *streams = NULL;
  size_t offset = 0;
  uint8_t type = 0;
  return bp_streams(plist, plist_len, bp, &count, &streams) && index < count &&
         bp_offset(bp, bp_ref(bp, streams, index), &offset) &&
         bp_container(bp, offset, &type, n, refs) && type == BPLIST_DICT;
}

bool bplist_get_streams_count(const uint8_t *plist, size_t plist_len,
                              size_t *count) {
  if (!count) {
    return false;
  }
  *count = 0;

  bplist_t bp;
  size_t n = 0;
  const uint8_t *refs = NULL;
  if (!bp_streams(plist, plist_len, &bp, &n, &refs)) {
    return false;
  }
  *count = n;
  return true;
}

bool bplist_get_stream_info(const uint8_t *plist, size_t plist_len,
                            size_t index, int64_t *type, size_t *ekey_len,
                            size_t *eiv_len, size_t *shk_len) {
  if (!type) {
    return false;
  }

  *type = -1;
  if (ekey_len) {
    *ekey_len = 0;
  }
  if (eiv_len) {
    *eiv_len = 0;
  }
  if (shk_len) {
    *shk_len = 0;
  }

  bplist_t bp;
  size_t n = 0;
  const uint8_t *refs = NULL;
  if (!bp_stream_dict(plist, plist_len, index, &bp, &n, &refs)) {
    return false;
  }

  for (size_t j = 0; j < n; j++) {
    size_t key_offset = 0;
    size_t val_offset = 0;
    char key[32];
    if (!bp_offset(&bp, bp_ref(&bp, refs, j), &key_offset) ||
        !bp_read_string(&bp, key_offset, key, sizeof(key)) ||
        !bp_offset(&bp, bp_ref(&bp, refs, n + j), &val_offset)) {
      continue;
    }

    if (strcmp(key, "type") == 0) {
      int64_t type_val = 0;
      if (bp_read_int(&bp, val_offset, &type_val)) {
        *type = type_val;
      }
    } else if (strcmp(key, "ekey") == 0 && ekey_len) {
      bp_read_data(&bp, val_offset, NULL, 0, ekey_len);
    } else if (strcmp(key, "eiv") == 0 && eiv_len) {
      bp_read_data(&bp, val_offset, NULL, 0, eiv_len);
    } else if (strcmp(key, "shk") == 0 && shk_len) {
      bp_read_data(&bp, val_offset, NULL, 0, shk_len);
    }
  }

  return (*type != -1);
}

bool bplist_get_stream_kv_info(const uint8_t *plist, size_t plist_len,
                               size_t index, bplist_kv_info_t *out,
                               size_t out_capacity, size_t *out_count) {
  if (!out || out_capacity == 0 || !out_count) {
    return false;
  }

  *out_count = 0;

  bplist_t bp;
  size_t n = 0;
  const uint8_t *refs = NULL;
  if (!bp_stream_dict(plist, plist_len, index, &bp, &n, &refs)) {
    return false;
  }

  for (size_t j = 0; j < n && *out_count < out_capacity; j++) {
    size_t key_offset = 0;
    char key[64];
    if (!bp_offset(&bp, bp_ref(&bp, refs, j), &key_offset) ||
        !bp_read_string(&bp, key_offset, key, sizeof(key))) {
      continue;
    }

    bplist_kv_info_t *info = &out[*out_count];
    memset(info, 0, sizeof(*info));
    strlcpy(info->key, key, sizeof(info->key));
    (*out_count)++;

    size_t val_offset = 0;
    if (!bp_offset(&bp, bp_ref(&bp, refs, n + j), &val_offset)) {
      continue; // BPLIST_VALUE_UNKNOWN
    }
    const uint8_t val_type = bp.data[val_offset] & 0xF0;

    if (val_type == BPLIST_INT) {
      info->value_type = BPLIST_VALUE_INT;
      int64_t int_val = 0;
      if (bp_read_int(&bp, val_offset, &int_val)) {
        info->int_value = int_val;
      }
    } else if (val_type == BPLIST_DATA) {
      info->value_type = BPLIST_VALUE_DATA;
      size_t len = 0;
      if (bp_read_data(&bp, val_offset, NULL, 0, &len)) {
        info->value_len = len;
      }
    } else if (val_type == BPLIST_STRING || val_type == BPLIST_UNICODE) {
      info->value_type = BPLIST_VALUE_STRING;
      size_t len = 0;
      if (bp_read_string_len(&bp, val_offset, &len)) {
        info->value_len = len;
      }
    } else if (val_type == BPLIST_UID) {
      info->value_type = BPLIST_VALUE_UID;
    } else if (val_type == BPLIST_ARRAY) {
      info->value_type = BPLIST_VALUE_ARRAY;
    } else if (val_type == BPLIST_DICT) {
      info->value_type = BPLIST_VALUE_DICT;
    }
  }

  return (*out_count > 0);
}

bool bplist_find_stream_crypto(const uint8_t *plist, size_t plist_len,
                               int64_t stream_type, uint8_t *ekey,
                               size_t ekey_capacity, size_t *ekey_len,
                               uint8_t *eiv, size_t eiv_capacity,
                               size_t *eiv_len, uint8_t *shk,
                               size_t shk_capacity, size_t *shk_len) {
  bool found = false;

  if (ekey_len) {
    *ekey_len = 0;
  }
  if (eiv_len) {
    *eiv_len = 0;
  }
  if (shk_len) {
    *shk_len = 0;
  }

  size_t stream_count = 0;
  if (!bplist_get_streams_count(plist, plist_len, &stream_count)) {
    return false;
  }

  for (size_t i = 0; i < stream_count; i++) {
    int64_t type = -1;
    size_t local_ekey_len = 0;
    size_t local_eiv_len = 0;
    size_t local_shk_len = 0;

    if (!bplist_get_stream_info(plist, plist_len, i, &type, &local_ekey_len,
                                &local_eiv_len, &local_shk_len)) {
      continue;
    }

    if (type != stream_type) {
      continue;
    }

    uint8_t temp_buf[512];
    size_t temp_len = 0;

    if (ekey && local_ekey_len > 0 && ekey_len) {
      if (bplist_find_data(plist, plist_len, "ekey", temp_buf, sizeof(temp_buf),
                           &temp_len)) {
        size_t copy_len = temp_len < ekey_capacity ? temp_len : ekey_capacity;
        memcpy(ekey, temp_buf, copy_len);
        *ekey_len = copy_len;
        found = true;
      }
    }

    if (eiv && local_eiv_len > 0 && eiv_len) {
      if (bplist_find_data(plist, plist_len, "eiv", temp_buf, sizeof(temp_buf),
                           &temp_len)) {
        size_t copy_len = temp_len < eiv_capacity ? temp_len : eiv_capacity;
        memcpy(eiv, temp_buf, copy_len);
        *eiv_len = copy_len;
        found = true;
      }
    }

    if (shk && local_shk_len > 0 && shk_len) {
      if (bplist_find_data(plist, plist_len, "shk", temp_buf, sizeof(temp_buf),
                           &temp_len)) {
        size_t copy_len = temp_len < shk_capacity ? temp_len : shk_capacity;
        memcpy(shk, temp_buf, copy_len);
        *shk_len = copy_len;
        found = true;
      }
    }

    break;
  }

  return found;
}

// ---------------------------------------------------------------------------
// IPv4 addresses anywhere under a key: SETUP timingPeerInfo (a dict whose
// "Addresses" is an array of strings), timingPeerList (an array of such
// dicts), SETPEERS (a top-level array of strings) and SETPEERSX (a top-level
// array of dicts).  One walk covers all four shapes.  Strings that are not a
// dotted quad -- IPv6 addresses, the peer "ID" -- are skipped.
// ---------------------------------------------------------------------------

typedef struct {
  bplist_t bp;
  uint32_t *out;
  size_t max;
  size_t count;
  unsigned visits;
} ipv4_walk_t;

// Bounds on the walk, so a malformed plist whose references fan out or loop
// cannot keep the RTSP task busy.
#define IPV4_WALK_MAX_DEPTH  5
#define IPV4_WALK_MAX_VISITS 256U

static bool parse_dotted_quad(const char *s, uint8_t octets[4]) {
  for (int i = 0; i < 4; i++) {
    if (*s < '0' || *s > '9') {
      return false;
    }
    unsigned value = 0;
    int digits = 0;
    while (*s >= '0' && *s <= '9') {
      value = value * 10U + (unsigned)(*s - '0');
      if (++digits > 3 || value > 255U) {
        return false;
      }
      s++;
    }
    octets[i] = (uint8_t)value;
    if (i < 3) {
      if (*s != '.') {
        return false;
      }
      s++;
    }
  }
  return *s == '\0';
}

static void walk_ipv4(ipv4_walk_t *w, uint64_t index, int depth) {
  const bplist_t *bp = &w->bp;
  size_t offset = 0;
  if (depth > IPV4_WALK_MAX_DEPTH || ++w->visits > IPV4_WALK_MAX_VISITS ||
      !bp_offset(bp, index, &offset)) {
    return;
  }
  const uint8_t type = bp->data[offset] & 0xF0;
  if (type == BPLIST_STRING || type == BPLIST_UNICODE) {
    char text[64];
    uint8_t octets[4];
    if (!bp_read_string(bp, offset, text, sizeof(text)) ||
        !parse_dotted_quad(text, octets)) {
      return;
    }
    uint32_t addr = 0;
    memcpy(&addr, octets, sizeof(addr)); // network byte order in memory
    for (size_t i = 0; i < w->count; i++) {
      if (w->out[i] == addr) {
        return;
      }
    }
    if (w->count < w->max) {
      w->out[w->count++] = addr;
    }
    return;
  }
  if (type != BPLIST_ARRAY && type != BPLIST_DICT) {
    return;
  }
  uint8_t container = 0;
  size_t n = 0;
  const uint8_t *refs = NULL;
  if (!bp_container(bp, offset, &container, &n, &refs)) {
    return;
  }
  // A dict's values follow its keys; only the values can hold addresses.
  if (container == BPLIST_DICT) {
    refs += n * bp->ref_size;
  }
  for (size_t i = 0; i < n; i++) {
    walk_ipv4(w, bp_ref(bp, refs, i), depth + 1);
  }
}

bool bplist_collect_ipv4(const uint8_t *plist, size_t plist_len,
                         const char *key, uint32_t *out, size_t max,
                         size_t *count) {
  if (count) {
    *count = 0;
  }
  ipv4_walk_t w = {.out = out, .max = max};
  if (!out || !count || !bplist_open(plist, plist_len, &w.bp)) {
    return false;
  }
  uint64_t start = w.bp.top_object;
  if (key && !bp_dict_lookup(&w.bp, w.bp.top_object, key, &start)) {
    return false;
  }
  walk_ipv4(&w, start, 0);
  *count = w.count;
  return true;
}

bool bplist_find_nested_int(const uint8_t *plist, size_t plist_len,
                            const char *dict_key, const char *key,
                            int64_t *out_value) {
  if (!dict_key || !key || !out_value) {
    return false;
  }
  bplist_t bp;
  uint64_t inner = 0;
  uint64_t value = 0;
  size_t offset = 0;
  return bplist_open(plist, plist_len, &bp) &&
         bp_dict_lookup(&bp, bp.top_object, dict_key, &inner) &&
         bp_dict_lookup(&bp, inner, key, &value) &&
         bp_offset(&bp, value, &offset) && bp_read_int(&bp, offset, out_value);
}

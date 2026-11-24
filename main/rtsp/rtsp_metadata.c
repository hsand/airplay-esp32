// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtsp_metadata.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "audio_receiver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rtsp_policy.h"

static const char *TAG = "rtsp_metadata";

#define DMAP_MAX_NESTING 8

// ============================================================================
// DMAP
// ============================================================================

static uint32_t read_be(const uint8_t *p, size_t len) {
  uint32_t value = 0;
  for (size_t i = 0; i < len && i < 4; i++) {
    value = (value << 8) | p[i];
  }
  return value;
}

static void copy_text(char *dst, const uint8_t *src, uint32_t len) {
  const size_t n =
      len < METADATA_STRING_MAX - 1 ? len : METADATA_STRING_MAX - 1;
  memcpy(dst, src, n);
  dst[n] = '\0';
}

static bool dmap_walk(const uint8_t *data, size_t len, airplay_dmap_t *out,
                      int depth) {
  bool found = false;
  size_t pos = 0;
  if (depth > DMAP_MAX_NESTING) {
    return false;
  }
  while (pos + 8 <= len) {
    char tag[5] = {0};
    memcpy(tag, data + pos, 4);
    const uint32_t item_len = read_be(data + pos + 4, 4);
    pos += 8;
    // Subtraction, not pos + item_len: that addition wraps on 32-bit size_t
    // for a hostile length and would let a malformed item read out of bounds.
    if (item_len > len - pos) {
      break;
    }
    const uint8_t *item = data + pos;
    if (strcmp(tag, "minm") == 0) {
      copy_text(out->title, item, item_len);
      found = true;
    } else if (strcmp(tag, "asar") == 0) {
      copy_text(out->artist, item, item_len);
      found = true;
    } else if (strcmp(tag, "asal") == 0) {
      copy_text(out->album, item, item_len);
      found = true;
    } else if (strcmp(tag, "asgn") == 0) {
      copy_text(out->genre, item, item_len);
      found = true;
    } else if (strcmp(tag, "astm") == 0 && item_len >= 1 && item_len <= 4) {
      out->duration_ms = read_be(item, item_len);
      out->has_duration = true;
      found = true;
    } else if (strcmp(tag, "caps") == 0 && item_len >= 1) {
      out->caps = item[item_len - 1];
      out->has_caps = true;
      found = true;
    } else if (strcmp(tag, "mlit") == 0 || strcmp(tag, "cmst") == 0 ||
               strcmp(tag, "mdst") == 0) {
      found = dmap_walk(item, item_len, out, depth + 1) || found;
    }
    pos += item_len;
  }
  return found;
}

bool airplay_dmap_parse(const uint8_t *data, size_t len, airplay_dmap_t *out) {
  memset(out, 0, sizeof(*out));
  return data && dmap_walk(data, len, out, 0);
}

bool airplay_dmap_caps_paused(uint8_t caps) {
  // Apple's table: 2 and 3 are rate 0; 4 and 5 fast-forward and rewind;
  // anything else plays at rate 1.
  return caps == 2 || caps == 3;
}

// ============================================================================
// The merged now-playing
// ============================================================================

static playback_metadata_t s_np;
static uint32_t s_dmap_duration_secs;
static uint32_t s_progress_duration_secs;
// The position the last progress gave, and when: the position reported now
// counts on from it while the sender is playing.
static bool s_have_position;
static uint32_t s_position_secs;
static int64_t s_position_us;

void rtsp_metadata_reset(void) {
  memset(&s_np, 0, sizeof(s_np));
  s_dmap_duration_secs = 0;
  s_progress_duration_secs = 0;
  s_have_position = false;
  s_position_secs = 0;
  s_position_us = 0;
}

static uint32_t position_now(void) {
  if (!s_have_position) {
    return 0;
  }
  uint32_t position = s_position_secs;
  if (!s_np.paused && audio_receiver_is_playing()) {
    const int64_t since = esp_timer_get_time() - s_position_us;
    if (since > 0) {
      position += (uint32_t)(since / 1000000);
    }
  }
  if (s_np.duration_secs > 0 && position > s_np.duration_secs) {
    position = s_np.duration_secs;
  }
  return position;
}

static void emit(void) {
  s_np.duration_secs = s_progress_duration_secs ? s_progress_duration_secs
                                                : s_dmap_duration_secs;
  playback_event_data_t data;
  data.metadata = s_np;
  data.metadata.position_secs = position_now();
  playback_events_emit(PLAYBACK_SOURCE_AIRPLAY, PLAYBACK_EVENT_METADATA, &data);
}

static void set_position(uint32_t seconds) {
  s_have_position = true;
  s_position_secs = seconds;
  s_position_us = esp_timer_get_time();
}

static void keep_text(char *dst, const char *src) {
  if (src[0]) {
    strlcpy(dst, src, METADATA_STRING_MAX);
  }
}

void rtsp_metadata_dmap(const uint8_t *data, size_t len) {
  airplay_dmap_t dmap;
  if (!airplay_dmap_parse(data, len, &dmap)) {
    ESP_LOGI(TAG, "DMAP metadata (%u bytes) with no known tags", (unsigned)len);
    return;
  }
  if (dmap.title[0] &&
      strncmp(dmap.title, s_np.title, METADATA_STRING_MAX) != 0) {
    // Another track: none of the old one's text, length or position carries
    // over; its progress follows.
    s_np.artist[0] = s_np.album[0] = s_np.genre[0] = '\0';
    s_np.has_artwork = false;
    s_dmap_duration_secs = 0;
    s_progress_duration_secs = 0;
    s_have_position = false;
  }
  keep_text(s_np.title, dmap.title);
  keep_text(s_np.artist, dmap.artist);
  keep_text(s_np.album, dmap.album);
  keep_text(s_np.genre, dmap.genre);
  if (dmap.has_duration) {
    s_dmap_duration_secs = dmap.duration_ms / 1000U;
  }
  if (dmap.has_caps) {
    const bool paused = airplay_dmap_caps_paused(dmap.caps);
    if (paused && !s_np.paused) {
      // Freeze where the playhead got to.
      set_position(position_now());
    } else if (!paused && s_np.paused && s_have_position) {
      s_position_us = esp_timer_get_time();
    }
    s_np.paused = paused;
  }
  ESP_LOGI(TAG, "Now playing: \"%s\" / %s / %s (%" PRIu32 " s%s)", s_np.title,
           s_np.artist, s_np.album, s_dmap_duration_secs,
           s_np.paused ? ", paused" : "");
  emit();
}

void rtsp_metadata_progress(const char *text, uint32_t sample_rate) {
  unsigned long start = 0;
  unsigned long current = 0;
  unsigned long end = 0;
  // NOLINTNEXTLINE(bugprone-unchecked-string-to-number-conversion)
  if (!text || sscanf(text, "%lu/%lu/%lu", &start, &current, &end) != 3) {
    return;
  }
  if (sample_rate == 0) {
    sample_rate = 44100;
  }
  uint32_t rendered = 0;
  const bool have_rendered = audio_receiver_get_rendered_rtp(&rendered);
  const uint32_t used =
      airplay_progress_current((uint32_t)start, (uint32_t)end,
                               (uint32_t)current, have_rendered, rendered);
  uint32_t elapsed = 0;
  uint32_t duration = 0;
  if (!airplay_progress_seconds((uint32_t)start, used, (uint32_t)end,
                                sample_rate, &elapsed, &duration)) {
    return;
  }
  s_progress_duration_secs = duration;
  set_position(elapsed);
  ESP_LOGI(TAG,
           "Progress: %" PRIu32 ":%02" PRIu32 " / %" PRIu32 ":%02" PRIu32
           " (%lu/%lu/%lu%s)",
           elapsed / 60, elapsed % 60, duration / 60, duration % 60, start,
           current, end, used != (uint32_t)current ? ", as played" : "");
  emit();
}

void rtsp_metadata_artwork(void) {
  s_np.has_artwork = true;
  emit();
}

void rtsp_metadata_update(const playback_metadata_t *partial,
                          bool has_position) {
  if (!partial) {
    return;
  }
  keep_text(s_np.title, partial->title);
  keep_text(s_np.artist, partial->artist);
  keep_text(s_np.album, partial->album);
  keep_text(s_np.genre, partial->genre);
  if (partial->duration_secs) {
    s_progress_duration_secs = partial->duration_secs;
  }
  if (has_position) {
    set_position(partial->position_secs);
  }
  emit();
}

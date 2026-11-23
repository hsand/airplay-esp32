// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "audio_plc.h"

#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif

#define H     AUDIO_PLC_HISTORY_FRAMES
#define X     AUDIO_PLC_XFADE_FRAMES
#define Q     32768 // unity gain, Q15
#define QBITS 15

_Static_assert(AUDIO_PLC_REPEAT_FRAMES + AUDIO_PLC_XFADE_FRAMES <= H,
               "the repeat and its lead-in must fit the history");
_Static_assert(AUDIO_PLC_REPEAT_FRAMES > AUDIO_PLC_XFADE_FRAMES,
               "the repeat must be longer than its crossfade");

static inline int16_t clamp16(int32_t v) {
  return v > INT16_MAX ? INT16_MAX : v < INT16_MIN ? INT16_MIN : (int16_t)v;
}

// Weight of the incoming side at step j of an X-frame crossfade, Q15.
static inline int32_t xfade_weight(uint32_t j) {
  return (int32_t)(((j + 1U) * (uint32_t)Q) / (X + 1U));
}

static inline int32_t mix(int32_t from, int32_t to, int32_t w) {
  return (from * (Q - w) + to * w) >> QBITS;
}

bool audio_plc_init(audio_plc_t *plc) {
  memset(plc, 0, sizeof(*plc));
  const size_t bytes = H * AUDIO_PLC_MAX_CHANNELS * sizeof(int16_t);
#ifdef ESP_PLATFORM
  plc->history = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
  if (!plc->history) {
    plc->history = malloc(bytes);
  }
  return plc->history != NULL;
}

void audio_plc_deinit(audio_plc_t *plc) {
  free(plc->history);
  plc->history = NULL;
}

void audio_plc_reset(audio_plc_t *plc) {
  plc->write = 0;
  plc->filled = 0;
  plc->concealing = false;
  plc->resume = 0;
  memset(plc->last, 0, sizeof(plc->last));
}

static inline const int16_t *hist(const audio_plc_t *plc, uint32_t index) {
  return &plc->history[(index % H) * plc->channels];
}

/* The next frame of the hole, before the fade.  The last X frames of each
 * pass over the segment are crossfaded into the X frames that led into its
 * start, so the jump back to the start lands on audio that follows on. */
static void next_repeat(audio_plc_t *plc, int32_t *frame) {
  const uint32_t pos = plc->loop_pos;
  const int16_t *s = hist(plc, plc->loop_start + pos);
  const uint32_t tail = plc->loop_len - X;
  if (pos >= tail) {
    const uint32_t j = pos - tail;
    const int16_t *lead = hist(plc, plc->loop_start + H - X + j);
    const int32_t w = xfade_weight(j);
    for (uint8_t ch = 0; ch < plc->channels; ch++) {
      frame[ch] = mix(s[ch], lead[ch], w);
    }
  } else {
    for (uint8_t ch = 0; ch < plc->channels; ch++) {
      frame[ch] = s[ch];
    }
  }
  plc->loop_pos = pos + 1U == plc->loop_len ? 0U : pos + 1U;
}

/* The next concealed frame as it is put out: the repeat, eased in from the
 * last sample played, under the fade. */
static void next_concealed(audio_plc_t *plc, int32_t *frame) {
  const uint32_t c = plc->concealed;
  if (plc->loop_len == 0U || c >= AUDIO_PLC_FADE_FRAMES) {
    for (uint8_t ch = 0; ch < plc->channels; ch++) {
      frame[ch] = 0;
    }
    plc->concealed = c + 1U;
    return;
  }
  next_repeat(plc, frame);
  const int32_t gain = (int32_t)(((AUDIO_PLC_FADE_FRAMES - c) * (uint32_t)Q) /
                                 AUDIO_PLC_FADE_FRAMES);
  const int32_t w = c < X ? xfade_weight(c) : Q;
  for (uint8_t ch = 0; ch < plc->channels; ch++) {
    const int32_t v = w < Q ? mix(plc->last[ch], frame[ch], w) : frame[ch];
    frame[ch] = (v * gain) >> QBITS;
  }
  plc->concealed = c + 1U;
}

static bool usable(audio_plc_t *plc, uint8_t channels) {
  if (!plc->history || channels == 0U || channels > AUDIO_PLC_MAX_CHANNELS) {
    return false;
  }
  if (plc->channels != channels) {
    audio_plc_reset(plc);
    plc->channels = channels;
  }
  return true;
}

void audio_plc_conceal(audio_plc_t *plc, int16_t *out, size_t frames,
                       uint8_t channels) {
  if (!usable(plc, channels)) {
    memset(out, 0, frames * channels * sizeof(int16_t));
    return;
  }
  if (!plc->concealing) {
    plc->concealing = true;
    plc->concealed = 0;
    plc->loop_pos = 0;
    plc->resume = 0;
    plc->events++;
    if (plc->filled < AUDIO_PLC_MIN_HISTORY) {
      plc->loop_len = 0;
    } else {
      uint32_t len = plc->filled - X;
      if (len > AUDIO_PLC_REPEAT_FRAMES) {
        len = AUDIO_PLC_REPEAT_FRAMES;
      }
      plc->loop_len = len;
      plc->loop_start = plc->write + H - len;
    }
  }
  int32_t frame[AUDIO_PLC_MAX_CHANNELS];
  for (size_t k = 0; k < frames; k++) {
    next_concealed(plc, frame);
    for (uint8_t ch = 0; ch < channels; ch++) {
      out[k * channels + ch] = clamp16(frame[ch]);
    }
  }
  plc->frames += (uint32_t)frames;
}

void audio_plc_good(audio_plc_t *plc, int16_t *pcm, size_t frames,
                    uint8_t channels) {
  if (!usable(plc, channels) || frames == 0U) {
    return;
  }
  if (plc->concealing) {
    plc->concealing = false;
    plc->resume = X;
  }
  // Out of the hole: the concealment carries on underneath and hands over.
  size_t k = 0;
  for (; k < frames && plc->resume > 0U; k++) {
    int32_t frame[AUDIO_PLC_MAX_CHANNELS];
    next_concealed(plc, frame);
    const int32_t w = xfade_weight(X - plc->resume);
    for (uint8_t ch = 0; ch < channels; ch++) {
      pcm[k * channels + ch] =
          clamp16(mix(frame[ch], pcm[k * channels + ch], w));
    }
    plc->resume--;
  }

  // Remember the tail of what was played.
  size_t from = 0;
  if (frames > H) {
    from = frames - H;
  }
  for (size_t i = from; i < frames;) {
    const uint32_t at = plc->write % H;
    size_t n = H - at;
    if (n > frames - i) {
      n = frames - i;
    }
    memcpy(&plc->history[at * channels], &pcm[i * channels],
           n * channels * sizeof(int16_t));
    plc->write = (at + (uint32_t)n) % H;
    i += n;
  }
  plc->filled += (uint32_t)(frames - from);
  if (plc->filled > H) {
    plc->filled = H;
  }
  for (uint8_t ch = 0; ch < channels; ch++) {
    plc->last[ch] = pcm[(frames - 1U) * channels + ch];
  }
}

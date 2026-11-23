// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Packet-loss concealment for holes in the PCM timeline.
 *
 * A hole used to play as silence: a lost 8 ms ALAC packet became an 8 ms
 * dropout with a click at each end.  Apple's receiver instead repeats the
 * audio it played last, fading it to nothing over about 50 ms, so a single
 * lost packet is hard to hear and a long gap still ends in silence.  This
 * does the same:
 *
 *  - the hole is filled by looping the last AUDIO_PLC_REPEAT_FRAMES of good
 *    audio (one ALAC packet), under a gain that falls linearly to zero over
 *    AUDIO_PLC_FADE_FRAMES, and with silence after that;
 *  - the joins are crossfaded over AUDIO_PLC_XFADE_FRAMES: from the last
 *    sample played into the repeat, at each loop, and from the concealment
 *    back into the real audio when it resumes.  Apple has none of these, and
 *    its first millisecond of concealment is silence; each is an audible
 *    step.
 *
 * Integer arithmetic only, a few multiplies per sample, and only while a
 * hole is being filled or just after: the ESP32-C5 has no FPU, and this runs
 * on the playback task.  Used by one consumer at a time (the timeline's
 * reader), so it takes no lock. */

// Good audio kept to repeat from.
#define AUDIO_PLC_HISTORY_FRAMES 512U
// The segment repeated: one ALAC packet.
#define AUDIO_PLC_REPEAT_FRAMES 352U
// Crossfade length at every join.
#define AUDIO_PLC_XFADE_FRAMES 32U
// The repeat fades to silence over this many frames (50 ms at 44.1 kHz).
#define AUDIO_PLC_FADE_FRAMES 2205U
// Less good audio than this and a hole is simply silence.
#define AUDIO_PLC_MIN_HISTORY (AUDIO_PLC_XFADE_FRAMES * 4U)

#define AUDIO_PLC_MAX_CHANNELS 2U

typedef struct {
  int16_t *history; // AUDIO_PLC_HISTORY_FRAMES x AUDIO_PLC_MAX_CHANNELS ring
  uint32_t write;   // next frame written
  uint32_t filled;  // good frames held, up to AUDIO_PLC_HISTORY_FRAMES
  uint8_t channels;
  int16_t last[AUDIO_PLC_MAX_CHANNELS]; // the last sample put out

  // The concealment under way, if any.
  bool concealing;
  uint32_t concealed;  // frames put out so far
  uint32_t loop_start; // ring index of the repeated segment's first frame
  uint32_t loop_len;
  uint32_t loop_pos;
  // Frames of real audio still to crossfade in after a hole.
  uint32_t resume;

  // Totals for diagnostics.
  uint32_t events;
  uint32_t frames;
} audio_plc_t;

// The history comes from PSRAM.  Returns false if it could not be allocated;
// holes are then silence, as before.
bool audio_plc_init(audio_plc_t *plc);
void audio_plc_deinit(audio_plc_t *plc);
// Forgets the history, e.g. on a flush: the next stream's holes must not
// repeat the last one's audio.
void audio_plc_reset(audio_plc_t *plc);

/* Real audio, just put out: crossfaded in place out of a concealment that
 * has just ended, then remembered. */
void audio_plc_good(audio_plc_t *plc, int16_t *pcm, size_t frames,
                    uint8_t channels);

// Fills `frames` frames of a hole.
void audio_plc_conceal(audio_plc_t *plc, int16_t *out, size_t frames,
                       uint8_t channels);

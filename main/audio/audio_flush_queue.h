// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* AirPlay 2 FLUSHBUFFERED, keyed on the buffered stream's 24-bit sequence
 * numbers the way Apple's receiver keys it.  Timestamps are carried for the
 * logs only.
 *
 * Each request becomes a record in a FIFO:
 *   - immediate (no flushFromSeq): drop every packet with seq < untilSeq.  It
 *     replaces the whole queue, so deferred flushes still pending are gone.
 *   - deferred (flushFromSeq present): drop [fromSeq, untilSeq) and keep
 *     everything around it.  Appended behind any record still pending.
 * A record is retired by the first packet with seq >= untilSeq, which is
 * itself kept.  Records apply both to what is already buffered (the caller
 * sweeps that when the record is pushed) and to what is still arriving
 * (audio_flush_queue_should_drop() on every packet, in stream order).
 *
 * Not thread-safe: the caller serialises pushes from the RTSP task against
 * the decode worker's filtering. */

/* Records held at once.  A sender pushes one per skip and the next packet
 * past untilSeq retires it, so more than two pending means rapid skipping;
 * the overflow merges into the newest record instead of being lost. */
#define AUDIO_FLUSH_QUEUE_DEPTH 8U

#define AUDIO_SEQ24_MASK 0xFFFFFFU

typedef struct {
  bool has_from;
  uint32_t from_seq;
  uint32_t until_seq;
  uint32_t from_ts;
  uint32_t until_ts;
} audio_flush_record_t;

typedef struct {
  audio_flush_record_t rec[AUDIO_FLUSH_QUEUE_DEPTH];
  uint8_t head;
  uint8_t count;
  /* Packets the queue has told the caller to drop, for the status line. */
  uint32_t dropped;
  uint32_t retired;
} audio_flush_queue_t;

/* Signed distance a - b between two 24-bit sequence numbers (serial number
 * arithmetic: sign bit 23). */
static inline int32_t audio_seq24_diff(uint32_t a, uint32_t b) {
  const uint32_t d = (a - b) & AUDIO_SEQ24_MASK;
  return (d & 0x800000U) ? (int32_t)d - 0x1000000 : (int32_t)d;
}

/* True when `seq` lies in the range the record removes. */
bool audio_flush_record_covers(const audio_flush_record_t *rec, uint32_t seq);

void audio_flush_queue_reset(audio_flush_queue_t *q);

/* Queue a FLUSHBUFFERED.  Returns true when the record is processed at once
 * (immediate, or deferred onto an empty queue), which is when Apple's reply
 * carries an RTP-Info time; false when it was appended behind another. */
bool audio_flush_queue_push(audio_flush_queue_t *q,
                            const audio_flush_record_t *rec);

/* Decide one packet, in stream order: retire every record the packet is past,
 * then report whether the record left at the head drops it. */
bool audio_flush_queue_should_drop(audio_flush_queue_t *q, uint32_t seq);

size_t audio_flush_queue_pending(const audio_flush_queue_t *q);

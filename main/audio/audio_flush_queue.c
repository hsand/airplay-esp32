// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "audio_flush_queue.h"

#include <string.h>

bool audio_flush_record_covers(const audio_flush_record_t *rec, uint32_t seq) {
  if (!rec || audio_seq24_diff(seq, rec->until_seq) >= 0) {
    return false;
  }
  return !rec->has_from || audio_seq24_diff(seq, rec->from_seq) >= 0;
}

void audio_flush_queue_reset(audio_flush_queue_t *q) {
  if (q) {
    memset(q, 0, sizeof(*q));
  }
}

static audio_flush_record_t *slot_at(audio_flush_queue_t *q, size_t i) {
  return &q->rec[(q->head + i) % AUDIO_FLUSH_QUEUE_DEPTH];
}

bool audio_flush_queue_push(audio_flush_queue_t *q,
                            const audio_flush_record_t *rec) {
  if (!q || !rec) {
    return false;
  }
  audio_flush_record_t r = *rec;
  r.from_seq &= AUDIO_SEQ24_MASK;
  r.until_seq &= AUDIO_SEQ24_MASK;

  if (!r.has_from || q->count == 0U) {
    /* An immediate flush supersedes everything still pending: whatever those
     * records would have removed lies below its untilSeq anyway, or belongs
     * to audio the sender has just abandoned. */
    if (!r.has_from) {
      q->count = 0U;
    }
    q->head = 0U;
    q->rec[0] = r;
    q->count = 1U;
    return true;
  }

  if (q->count < AUDIO_FLUSH_QUEUE_DEPTH) {
    *slot_at(q, q->count) = r;
    q->count++;
    return false;
  }
  /* Full: widen the newest record to the new end.  The packets between the
   * two ranges belong to a track that was skipped past again within a few
   * packets, so dropping them costs nothing audible, whereas forgetting this
   * record would play the outgoing track over the new one. */
  slot_at(q, q->count - 1U)->until_seq = r.until_seq;
  slot_at(q, q->count - 1U)->until_ts = r.until_ts;
  return false;
}

bool audio_flush_queue_should_drop(audio_flush_queue_t *q, uint32_t seq) {
  if (!q) {
    return false;
  }
  seq &= AUDIO_SEQ24_MASK;
  while (q->count > 0U &&
         audio_seq24_diff(seq, q->rec[q->head].until_seq) >= 0) {
    q->head = (uint8_t)((q->head + 1U) % AUDIO_FLUSH_QUEUE_DEPTH);
    q->count--;
    q->retired++;
  }
  if (q->count == 0U) {
    q->head = 0U;
    return false;
  }
  if (audio_flush_record_covers(&q->rec[q->head], seq)) {
    q->dropped++;
    return true;
  }
  return false;
}

size_t audio_flush_queue_pending(const audio_flush_queue_t *q) {
  return q ? q->count : 0U;
}

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rtp_sync.h"

#include <string.h>

#define NS_PER_S 1000000000LL
// Re-base the anchor before RTP distances near the int32 range (~6 h).
#define REBASE_SAMPLES (1L << 30)

static int64_t floor_div(int64_t num, int64_t den) {
  const int64_t q = num / den;
  return (num % den != 0 && (num < 0) != (den < 0)) ? q - 1 : q;
}

static int64_t div_round(int64_t num, int64_t den) {
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

void rtp_sync_reset(rtp_sync_t *s) {
  memset(s, 0, sizeof(*s));
}

bool rtp_sync_current(const rtp_sync_t *s, uint32_t rtp,
                      rtp_sync_anchor_t *out) {
  if (!s->valid || s->sample_rate == 0U) {
    return false;
  }
  const int64_t samples =
      (int64_t)(int32_t)(rtp - s->anchor_rtp) - (int64_t)s->average;
  out->rtp = rtp;
  out->network_ns = s->anchor_network_ns +
                    (uint64_t)div_round(samples * NS_PER_S, s->sample_rate);
  return true;
}

static void anchor_at(rtp_sync_t *s, uint32_t rtp, uint64_t network_ns) {
  s->anchor_rtp = rtp;
  s->anchor_network_ns = network_ns;
  s->average = 0;
}

rtp_sync_action_t rtp_sync_announce(rtp_sync_t *s, uint32_t sample_rate,
                                    uint64_t timeline, uint8_t flags,
                                    uint32_t rtp, uint64_t network_ns,
                                    uint32_t apply_rtp,
                                    rtp_sync_anchor_t *out) {
  if (sample_rate == 0U) {
    return RTP_SYNC_KEEP;
  }
  s->announces++;
  s->last_rtp = rtp;

  /* The first packet of a segment, or one on another timeline (a new PTP
   * grandmaster: the old map means nothing there), anchors at once. */
  if (!s->valid || timeline != s->timeline || sample_rate != s->sample_rate) {
    s->valid = true;
    s->pending = false;
    s->sample_rate = sample_rate;
    s->timeline = timeline;
    anchor_at(s, rtp, network_ns);
    s->last_error = 0;
    return rtp_sync_current(s, rtp, out) ? RTP_SYNC_PUBLISH : RTP_SYNC_KEEP;
  }

  if ((int32_t)(rtp - s->anchor_rtp) > REBASE_SAMPLES ||
      (int32_t)(rtp - s->anchor_rtp) < -REBASE_SAMPLES) {
    // Same map, nearer anchor.
    rtp_sync_anchor_t here;
    (void)rtp_sync_current(s, rtp, &here);
    anchor_at(s, here.rtp, here.network_ns);
  }

  // Where this packet puts the anchor, against where the map has it.
  const int64_t elapsed = floor_div(
      (int64_t)(network_ns - s->anchor_network_ns) * (int64_t)sample_rate,
      NS_PER_S);
  const int64_t measured = (int64_t)(int32_t)(rtp - s->anchor_rtp) - elapsed;
  const int64_t error = measured - s->average;
  s->last_error = error > INT32_MAX   ? INT32_MAX
                  : error < INT32_MIN ? INT32_MIN
                                      : (int32_t)error;

  const bool flushed = (flags & RTP_SYNC_FLAG_FIRST_AFTER_FLUSH) != 0;
  if (flushed || error > RTP_SYNC_RESET_SAMPLES ||
      error < -RTP_SYNC_RESET_SAMPLES) {
    if (flushed) {
      s->resets++;
    } else {
      s->jumps++;
    }
    /* A new map, measured from this packet from now on, but the engine keeps
     * the old one until playback reaches apply_rtp. */
    anchor_at(s, rtp, network_ns);
    s->pending = true;
    s->apply_rtp = apply_rtp;
    return RTP_SYNC_DEFER;
  }

  const int64_t magnitude = error < 0 ? -error : error;
  if (magnitude > s->max_error) {
    s->max_error = (int32_t)magnitude;
  }
  const int32_t step = (int32_t)(magnitude >> RTP_SYNC_SHIFT);
  if (step == 0) {
    return s->pending ? RTP_SYNC_DEFER : RTP_SYNC_KEEP;
  }
  s->average += error < 0 ? -step : step;
  s->nudges++;
  if (s->pending) {
    return RTP_SYNC_DEFER;
  }
  return rtp_sync_current(s, rtp, out) ? RTP_SYNC_PUBLISH : RTP_SYNC_KEEP;
}

bool rtp_sync_poll(rtp_sync_t *s, bool cursor_valid, uint32_t cursor_rtp,
                   rtp_sync_anchor_t *out) {
  if (!s->pending) {
    return false;
  }
  const int32_t ahead = (int32_t)(s->apply_rtp - cursor_rtp);
  if (cursor_valid && ahead > 0) {
    if (ahead <= RTP_SYNC_FORCE_APPLY_SAMPLES) {
      return false;
    }
    s->forced++;
  }
  s->pending = false;
  s->applied++;
  return rtp_sync_current(s, s->last_rtp, out);
}

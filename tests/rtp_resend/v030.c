// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* The v0.3.0 resend tracker (baseline/), compiled in its own translation
 * unit under other names, so test.c can run it beside the current one. */

#define rtp_resend_reset       v030_rtp_resend_reset
#define rtp_resend_on_packet   v030_rtp_resend_on_packet
#define rtp_resend_collect     v030_rtp_resend_collect
#define rtp_resend_outstanding v030_rtp_resend_outstanding
#include "baseline/rtp_resend.c"

#include "v030.h"

static rtp_resend_t tracker;

void v030_reset(uint32_t rtp_step) {
  rtp_resend_reset(&tracker, rtp_step);
}

bool v030_on_packet(uint16_t seq, uint32_t rtp, bool retransmit) {
  return rtp_resend_on_packet(&tracker, seq, rtp, retransmit) !=
         RTP_RESEND_UNWANTED;
}

size_t v030_collect(uint32_t now_ms, bool cursor_valid, uint32_t cursor_rtp,
                    uint16_t *first, uint16_t *count, size_t max) {
  rtp_resend_range_t ranges[16];
  if (max > 16) {
    max = 16;
  }
  const size_t n = rtp_resend_collect(&tracker, now_ms, cursor_valid,
                                      cursor_rtp, ranges, max);
  for (size_t i = 0; i < n; i++) {
    first[i] = ranges[i].first;
    count[i] = ranges[i].count;
  }
  return n;
}

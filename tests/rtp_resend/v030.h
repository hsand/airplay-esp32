// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The v0.3.0 tracker, one instance, for the A/B simulation in test.c.
void v030_reset(uint32_t rtp_step);
bool v030_on_packet(uint16_t seq, uint32_t rtp, bool retransmit);
size_t v030_collect(uint32_t now_ms, bool cursor_valid, uint32_t cursor_rtp,
                    uint16_t *first, uint16_t *count, size_t max);

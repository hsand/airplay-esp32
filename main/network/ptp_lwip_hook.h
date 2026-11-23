// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

// lwIP hook file, injected into the lwip component by main/CMakeLists.txt
// through ESP_IDF_LWIP_HOOK_FILENAME (ESP-IDF's documented way to add a
// custom hook).  It makes ip4_input() hand every IPv4 packet to
// ptp_clock.c before any socket sees it, so a SYNC is timestamped in the
// tcpip thread rather than after the socket mailbox and a task wake-up.
// The hook never consumes a packet.
#pragma once

struct pbuf;
struct netif;

int ptp_lwip_ip4_input_hook(struct pbuf *p, struct netif *inp);

#define LWIP_HOOK_IP4_INPUT(p, inp) ptp_lwip_ip4_input_hook((p), (inp))

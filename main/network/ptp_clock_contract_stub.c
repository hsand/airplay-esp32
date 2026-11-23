// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* TODO(WP1): temporary weak fallbacks for the PTP <-> audio contract in
 * ptp_clock.h.  The PTP rework defines the real functions in ptp_clock.c,
 * which override these at link time; delete this file (and its line in
 * main/CMakeLists.txt) when it lands.
 *
 * They are built on the offset-only API, so they model no rate: the network
 * time of any local instant is that instant plus the current filtered offset,
 * and the "grandmaster" is the master the anchor pinned. */

#include "ptp_clock.h"

#include "esp_log.h"

static const char *TAG = "ptp_contract";

__attribute__((weak)) void ptp_clock_set_peers(const uint32_t *ipv4_addrs,
                                               size_t count) {
  (void)ipv4_addrs;
  ESP_LOGI(TAG, "TODO(WP1): %u timing peer(s) not applied (stub)",
           (unsigned)count);
}

__attribute__((weak)) uint64_t ptp_clock_get_grandmaster_id(void) {
  return ptp_clock_get_master_clock_id();
}

__attribute__((weak)) bool ptp_clock_local_to_network(int64_t local_ns,
                                                      uint64_t *network_ns,
                                                      uint64_t *gm_id) {
  if (!ptp_clock_is_locked()) {
    return false;
  }
  if (network_ns) {
    *network_ns = (uint64_t)(local_ns + ptp_clock_get_offset_ns());
  }
  if (gm_id) {
    *gm_id = ptp_clock_get_grandmaster_id();
  }
  return true;
}

__attribute__((weak)) bool ptp_clock_network_to_local(uint64_t network_ns,
                                                      uint64_t gm_id,
                                                      int64_t *local_ns) {
  if (!ptp_clock_is_locked() || gm_id != ptp_clock_get_grandmaster_id()) {
    return false;
  }
  if (local_ns) {
    *local_ns = (int64_t)network_ns - ptp_clock_get_offset_ns();
  }
  return true;
}

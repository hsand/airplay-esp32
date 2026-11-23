// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * NTP-style timing for AirPlay 1 (realtime) streams.
 *
 * The client asks the sender's timing port for its time and feeds the answers
 * to a software clock that runs on the local crystal and is disciplined like
 * a kernel PLL: steered in rate and slewed in phase, and stepped only on the
 * first answer or an error of more than 100 ms.  Network time is therefore a
 * continuous function of local time, offset plus rate, rather than an offset
 * that jumps whenever a better measurement turns up.
 *
 * Network time is in nanoseconds since the NTP epoch (1900), the same scale
 * the sync packets' NTP timestamps convert to.
 */

/**
 * Start the timing client against the sender's timing port.  The first three
 * exchanges are made back to back, so the clock is usually locked a few round
 * trips after this returns.
 * @param remote_ip Remote IP address (network byte order)
 * @param remote_port Remote timing port
 * @return ESP_OK on success
 */
esp_err_t ntp_clock_start_client(uint32_t remote_ip, uint16_t remote_port);

/**
 * Stop the timing client and forget the clock.
 */
void ntp_clock_stop(void);

/**
 * Whether the startup exchanges are done and network time can be used.
 */
bool ntp_clock_is_locked(void);

/**
 * Network time minus local time at this moment, from the disciplined clock.
 * remote_time = local_time + offset.  0 until the first answer.
 */
int64_t ntp_clock_get_offset_ns(void);

/**
 * Local (esp_timer, ns) to network time, using the clock's current offset and
 * rate.  Returns false while the clock is not locked.  Safe on any task; it
 * takes a short spinlock and never blocks.
 */
bool ntp_clock_local_to_network(int64_t local_ns, uint64_t *network_ns);

/**
 * The inverse of ntp_clock_local_to_network().  Returns false while the clock
 * is not locked.
 */
bool ntp_clock_network_to_local(uint64_t network_ns, int64_t *local_ns);

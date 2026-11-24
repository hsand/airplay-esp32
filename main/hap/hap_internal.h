// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

// The event channel keys: the receiver encrypts what it sends with write_key
// and decrypts what it reads with read_key.
void hap_derive_events_keys(const uint8_t *ikm, size_t ikm_len,
                            uint8_t write_key[32], uint8_t read_key[32]);

int hap_hkdf_sha512(const uint8_t *salt, size_t salt_len, const uint8_t *ikm,
                    size_t ikm_len, const uint8_t *info, size_t info_len,
                    uint8_t *okm, size_t okm_len);

// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* The platform under tests/airplay_remote: ESP-IDF logging and errors, a
 * settable clock, and the few libsodium calls hap_crypto.c makes.  Every shim
 * header run.py generates (esp_log.h, sodium.h, ...) includes only this file.
 *
 * HMAC-SHA512 is implemented here from FIPS 180-4 / RFC 2104 so the Events
 * key derivation can be checked against Python's hashlib on a host without
 * libsodium; the AEAD calls are never reached by the code under test. */

#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef int esp_err_t;
#define ESP_OK                0
#define ESP_FAIL              (-1)
#define ESP_ERR_NO_MEM        0x101
#define ESP_ERR_INVALID_ARG   0x102
#define ESP_ERR_INVALID_STATE 0x103

extern int64_t test_now_us;
#define esp_timer_get_time() (test_now_us)

#define ESP_LOGE(tag, ...) ((void)(tag))
#define ESP_LOGW(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) ((void)(tag))
#define ESP_LOGD(tag, ...) ((void)(tag))

// ---------------------------------------------------------------------------
// SHA-512 and HMAC-SHA512, the libsodium API shape.
// ---------------------------------------------------------------------------

#define crypto_auth_hmacsha512_BYTES 64U

typedef struct {
  uint64_t h[8];
  uint8_t block[128];
  size_t used;
  uint64_t total;
} test_sha512_t;

typedef struct {
  test_sha512_t inner;
  test_sha512_t outer;
} crypto_auth_hmacsha512_state;

static const uint64_t test_sha512_k[80] = {
    0x428a2f98d728ae22ULL, 0x7137449123ef65cdULL, 0xb5c0fbcfec4d3b2fULL,
    0xe9b5dba58189dbbcULL, 0x3956c25bf348b538ULL, 0x59f111f1b605d019ULL,
    0x923f82a4af194f9bULL, 0xab1c5ed5da6d8118ULL, 0xd807aa98a3030242ULL,
    0x12835b0145706fbeULL, 0x243185be4ee4b28cULL, 0x550c7dc3d5ffb4e2ULL,
    0x72be5d74f27b896fULL, 0x80deb1fe3b1696b1ULL, 0x9bdc06a725c71235ULL,
    0xc19bf174cf692694ULL, 0xe49b69c19ef14ad2ULL, 0xefbe4786384f25e3ULL,
    0x0fc19dc68b8cd5b5ULL, 0x240ca1cc77ac9c65ULL, 0x2de92c6f592b0275ULL,
    0x4a7484aa6ea6e483ULL, 0x5cb0a9dcbd41fbd4ULL, 0x76f988da831153b5ULL,
    0x983e5152ee66dfabULL, 0xa831c66d2db43210ULL, 0xb00327c898fb213fULL,
    0xbf597fc7beef0ee4ULL, 0xc6e00bf33da88fc2ULL, 0xd5a79147930aa725ULL,
    0x06ca6351e003826fULL, 0x142929670a0e6e70ULL, 0x27b70a8546d22ffcULL,
    0x2e1b21385c26c926ULL, 0x4d2c6dfc5ac42aedULL, 0x53380d139d95b3dfULL,
    0x650a73548baf63deULL, 0x766a0abb3c77b2a8ULL, 0x81c2c92e47edaee6ULL,
    0x92722c851482353bULL, 0xa2bfe8a14cf10364ULL, 0xa81a664bbc423001ULL,
    0xc24b8b70d0f89791ULL, 0xc76c51a30654be30ULL, 0xd192e819d6ef5218ULL,
    0xd69906245565a910ULL, 0xf40e35855771202aULL, 0x106aa07032bbd1b8ULL,
    0x19a4c116b8d2d0c8ULL, 0x1e376c085141ab53ULL, 0x2748774cdf8eeb99ULL,
    0x34b0bcb5e19b48a8ULL, 0x391c0cb3c5c95a63ULL, 0x4ed8aa4ae3418acbULL,
    0x5b9cca4f7763e373ULL, 0x682e6ff3d6b2b8a3ULL, 0x748f82ee5defb2fcULL,
    0x78a5636f43172f60ULL, 0x84c87814a1f0ab72ULL, 0x8cc702081a6439ecULL,
    0x90befffa23631e28ULL, 0xa4506cebde82bde9ULL, 0xbef9a3f7b2c67915ULL,
    0xc67178f2e372532bULL, 0xca273eceea26619cULL, 0xd186b8c721c0c207ULL,
    0xeada7dd6cde0eb1eULL, 0xf57d4f7fee6ed178ULL, 0x06f067aa72176fbaULL,
    0x0a637dc5a2c898a6ULL, 0x113f9804bef90daeULL, 0x1b710b35131c471bULL,
    0x28db77f523047d84ULL, 0x32caab7b40c72493ULL, 0x3c9ebe0a15c9bebcULL,
    0x431d67c49c100d4cULL, 0x4cc5d4becb3e42b6ULL, 0x597f299cfc657e2aULL,
    0x5fcb6fab3ad6faecULL, 0x6c44198c4a475817ULL};

static inline uint64_t test_ror64(uint64_t x, unsigned n) {
  return (x >> n) | (x << (64U - n));
}

static inline void test_sha512_block(test_sha512_t *s, const uint8_t *p) {
  uint64_t w[80];
  for (int i = 0; i < 16; i++) {
    w[i] = 0;
    for (int j = 0; j < 8; j++) {
      w[i] = (w[i] << 8) | p[i * 8 + j];
    }
  }
  for (int i = 16; i < 80; i++) {
    const uint64_t s0 = test_ror64(w[i - 15], 1) ^ test_ror64(w[i - 15], 8) ^
                        (w[i - 15] >> 7);
    const uint64_t s1 = test_ror64(w[i - 2], 19) ^ test_ror64(w[i - 2], 61) ^
                        (w[i - 2] >> 6);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint64_t a = s->h[0], b = s->h[1], c = s->h[2], d = s->h[3];
  uint64_t e = s->h[4], f = s->h[5], g = s->h[6], h = s->h[7];
  for (int i = 0; i < 80; i++) {
    const uint64_t t1 = h +
                        (test_ror64(e, 14) ^ test_ror64(e, 18) ^
                         test_ror64(e, 41)) +
                        ((e & f) ^ (~e & g)) + test_sha512_k[i] + w[i];
    const uint64_t t2 =
        (test_ror64(a, 28) ^ test_ror64(a, 34) ^ test_ror64(a, 39)) +
        ((a & b) ^ (a & c) ^ (b & c));
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  s->h[0] += a;
  s->h[1] += b;
  s->h[2] += c;
  s->h[3] += d;
  s->h[4] += e;
  s->h[5] += f;
  s->h[6] += g;
  s->h[7] += h;
}

static inline void test_sha512_init(test_sha512_t *s) {
  static const uint64_t iv[8] = {
      0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL, 0x3c6ef372fe94f82bULL,
      0xa54ff53a5f1d36f1ULL, 0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL,
      0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL};
  memcpy(s->h, iv, sizeof(iv));
  s->used = 0;
  s->total = 0;
}

static inline void test_sha512_update(test_sha512_t *s, const uint8_t *p,
                                      size_t len) {
  s->total += len;
  while (len > 0) {
    size_t n = 128 - s->used;
    if (n > len) {
      n = len;
    }
    memcpy(s->block + s->used, p, n);
    s->used += n;
    p += n;
    len -= n;
    if (s->used == 128) {
      test_sha512_block(s, s->block);
      s->used = 0;
    }
  }
}

static inline void test_sha512_final(test_sha512_t *s, uint8_t out[64]) {
  const uint64_t bits = s->total * 8;
  const uint8_t one = 0x80;
  const uint8_t zero = 0;
  test_sha512_update(s, &one, 1);
  while (s->used != 112) {
    test_sha512_update(s, &zero, 1);
  }
  uint8_t length[16] = {0};
  for (int i = 0; i < 8; i++) {
    length[15 - i] = (uint8_t)(bits >> (8 * i));
  }
  test_sha512_update(s, length, 16);
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      out[i * 8 + j] = (uint8_t)(s->h[i] >> (56 - 8 * j));
    }
  }
}

static inline int crypto_auth_hmacsha512_init(
    crypto_auth_hmacsha512_state *st, const unsigned char *key,
    size_t keylen) {
  uint8_t k[128] = {0};
  if (keylen > 128) {
    test_sha512_t t;
    test_sha512_init(&t);
    test_sha512_update(&t, key, keylen);
    test_sha512_final(&t, k);
  } else {
    memcpy(k, key, keylen);
  }
  uint8_t pad[128];
  for (int i = 0; i < 128; i++) {
    pad[i] = k[i] ^ 0x36;
  }
  test_sha512_init(&st->inner);
  test_sha512_update(&st->inner, pad, 128);
  for (int i = 0; i < 128; i++) {
    pad[i] = k[i] ^ 0x5c;
  }
  test_sha512_init(&st->outer);
  test_sha512_update(&st->outer, pad, 128);
  return 0;
}

static inline int crypto_auth_hmacsha512_update(
    crypto_auth_hmacsha512_state *st, const unsigned char *in,
    unsigned long long inlen) {
  test_sha512_update(&st->inner, in, (size_t)inlen);
  return 0;
}

static inline int crypto_auth_hmacsha512_final(
    crypto_auth_hmacsha512_state *st, unsigned char *out) {
  uint8_t inner[64];
  test_sha512_final(&st->inner, inner);
  test_sha512_update(&st->outer, inner, sizeof(inner));
  test_sha512_final(&st->outer, out);
  return 0;
}

static inline void sodium_memzero(void *p, size_t len) {
  memset(p, 0, len);
}

// Present in hap_crypto.c but outside what is tested.
static inline int crypto_aead_chacha20poly1305_ietf_encrypt(
    unsigned char *c, unsigned long long *clen, const unsigned char *m,
    unsigned long long mlen, const unsigned char *ad, unsigned long long adlen,
    const unsigned char *nsec, const unsigned char *npub,
    const unsigned char *k) {
  (void)c, (void)m, (void)mlen, (void)ad, (void)adlen, (void)nsec, (void)npub,
      (void)k;
  *clen = 0;
  return -1;
}

static inline int crypto_aead_chacha20poly1305_ietf_decrypt(
    unsigned char *m, unsigned long long *mlen, unsigned char *nsec,
    const unsigned char *c, unsigned long long clen, const unsigned char *ad,
    unsigned long long adlen, const unsigned char *npub,
    const unsigned char *k) {
  (void)m, (void)nsec, (void)c, (void)clen, (void)ad, (void)adlen, (void)npub,
      (void)k;
  *mlen = 0;
  return -1;
}

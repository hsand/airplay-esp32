// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/*
 * Runs every binary plist lookup on each record read from stdin and prints
 * one line of results per record, for run.py to compare with plistlib.
 *
 * A record is a 4-byte little-endian length and that many bytes.  Each one is
 * copied into a heap block of exactly its size, so AddressSanitizer reports
 * any read past the end.
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "plist.h"

static void put_hex(const uint8_t *p, size_t n) {
  if (n == 0) {
    fputs("=", stdout); // distinguishes an empty value from a missing one
  }
  for (size_t i = 0; i < n; i++) {
    printf("%02x", p[i]);
  }
}

static void put_addrs(bool ok, const uint32_t *addrs, size_t n) {
  if (!ok) {
    fputs("-", stdout);
    return;
  }
  fputs("[", stdout);
  for (size_t i = 0; i < n; i++) {
    uint8_t o[4];
    memcpy(o, &addrs[i], 4);
    printf("%s%u.%u.%u.%u", i ? "," : "", o[0], o[1], o[2], o[3]);
  }
  fputs("]", stdout);
}

static void run(const uint8_t *p, size_t len) {
  int64_t i64 = 0;
  double real = 0;
  char str[32];
  uint8_t data[64];
  size_t n = 0;

  fputs("int=", stdout);
  if (bplist_find_int(p, len, "int", &i64)) {
    printf("%" PRId64, i64);
  } else {
    fputs("-", stdout);
  }

  fputs(" real=", stdout);
  if (bplist_find_real(p, len, "real", &real)) {
    printf("%a", real);
  } else {
    fputs("-", stdout);
  }

  fputs(" str=", stdout);
  if (bplist_find_string(p, len, "str", str, sizeof(str))) {
    put_hex((const uint8_t *)str, strlen(str));
  } else {
    fputs("-", stdout);
  }

  fputs(" data=", stdout);
  if (bplist_find_data(p, len, "data", data, sizeof(data), &n)) {
    put_hex(data, n);
  } else {
    fputs("-", stdout);
  }

  fputs(" deep=", stdout);
  if (bplist_find_data_deep(p, len, "deep", data, sizeof(data), &n)) {
    put_hex(data, n);
  } else {
    fputs("-", stdout);
  }

  fputs(" nested=", stdout);
  if (bplist_find_nested_int(p, len, "params", "value", &i64)) {
    printf("%" PRId64, i64);
  } else {
    fputs("-", stdout);
  }

  size_t streams = 0;
  bool have_streams = bplist_get_streams_count(p, len, &streams);
  printf(" streams=");
  if (have_streams) {
    printf("%zu", streams);
  } else {
    fputs("-", stdout);
  }

  for (size_t i = 0; i < 3; i++) {
    int64_t type = 0;
    size_t ekey = 0;
    size_t eiv = 0;
    size_t shk = 0;
    printf(" si%zu=", i);
    if (bplist_get_stream_info(p, len, i, &type, &ekey, &eiv, &shk)) {
      printf("%" PRId64 "/%zu/%zu/%zu", type, ekey, eiv, shk);
    } else {
      fputs("-", stdout);
    }

    bplist_kv_info_t kv[8];
    size_t kv_count = 0;
    printf(" kv%zu=", i);
    if (bplist_get_stream_kv_info(p, len, i, kv, 8, &kv_count)) {
      for (size_t k = 0; k < kv_count; k++) {
        printf("%s", k ? "," : "");
        put_hex((const uint8_t *)kv[k].key, strlen(kv[k].key));
        printf(":%u:%zu:%" PRId64, kv[k].value_type, kv[k].value_len,
               kv[k].int_value);
      }
    } else {
      fputs("-", stdout);
    }
  }

  uint8_t ekey[16];
  uint8_t eiv[16];
  uint8_t shk[64];
  size_t ekey_len = 0;
  size_t eiv_len = 0;
  size_t shk_len = 0;
  fputs(" crypto=", stdout);
  if (bplist_find_stream_crypto(p, len, 96, ekey, sizeof(ekey), &ekey_len, eiv,
                                sizeof(eiv), &eiv_len, shk, sizeof(shk),
                                &shk_len)) {
    put_hex(ekey, ekey_len);
    fputs("/", stdout);
    put_hex(eiv, eiv_len);
    fputs("/", stdout);
    put_hex(shk, shk_len);
  } else {
    fputs("-", stdout);
  }

  uint32_t addrs[8];
  bool ok = bplist_collect_ipv4(p, len, "peers", addrs, 8, &n);
  fputs(" ipv4k=", stdout);
  put_addrs(ok, addrs, n);
  ok = bplist_collect_ipv4(p, len, NULL, addrs, 8, &n);
  fputs(" ipv4=", stdout);
  put_addrs(ok, addrs, n);

  fputs("\n", stdout);
  fflush(stdout);
}

int main(void) {
  for (;;) {
    uint8_t header[4];
    size_t got = fread(header, 1, sizeof(header), stdin);
    if (got == 0) {
      return 0;
    }
    if (got != sizeof(header)) {
      fprintf(stderr, "truncated record header\n");
      return 2;
    }
    size_t len = (size_t)header[0] | (size_t)header[1] << 8 |
                 (size_t)header[2] << 16 | (size_t)header[3] << 24;
    uint8_t *buf = malloc(len ? len : 1);
    if (!buf || fread(buf, 1, len, stdin) != len) {
      fprintf(stderr, "truncated record\n");
      return 2;
    }
    run(buf, len);
    free(buf);
  }
}

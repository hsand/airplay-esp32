// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Build the real stream SETUP reply: argv is the stream type, the data and
 * control ports, audioBufferSize and arrivalToRenderLatencyMs (negative to
 * leave it out).  The Python runner decodes it with plistlib.  Every smaller
 * output capacity must fail cleanly, which the sanitizers watch. */
#include "plist.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
  if (argc != 6) {
    return 1;
  }
  const int64_t type = strtoll(argv[1], NULL, 10);
  const uint16_t data_port = (uint16_t)strtoul(argv[2], NULL, 10);
  const uint16_t control_port = (uint16_t)strtoul(argv[3], NULL, 10);
  const uint32_t buffer_size = (uint32_t)strtoul(argv[4], NULL, 10);
  const int64_t arrival_ms = strtoll(argv[5], NULL, 10);
  uint8_t reference[256];
  const size_t length =
      bplist_build_stream_setup(reference, sizeof(reference), type, data_port,
                                control_port, buffer_size, arrival_ms);
  if (length == 0) {
    return 2;
  }
  for (size_t capacity = 0; capacity <= length + 1; capacity++) {
    uint8_t *out = malloc(capacity ? capacity : 1);
    if (!out) {
      return 1;
    }
    const size_t actual =
        bplist_build_stream_setup(out, capacity, type, data_port, control_port,
                                  buffer_size, arrival_ms);
    const bool valid =
        capacity < length
            ? actual == 0
            : actual == length && memcmp(out, reference, length) == 0;
    free(out);
    if (!valid) {
      fprintf(stderr, "Wrong result at capacity %zu: %zu (need %zu)\n",
              capacity, actual, length);
      return 1;
    }
  }
  return fwrite(reference, 1, length, stdout) == length ? 0 : 1;
}

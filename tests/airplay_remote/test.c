// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for the receiver's remote-control, advertisement and
 * now-playing logic: rtsp_policy.c (feature bits, TXT records, the pairing
 * identity, the preemption decision, command routing, progress and buffer
 * arithmetic), rtsp_metadata.c (DMAP and the merged now-playing state),
 * hap_derive_events_keys() and the sendMediaRemoteCommand plist.
 *
 * Checks that need an independent implementation (plist decoding, HKDF,
 * base64, the identity hash) are printed as "KEY ..." lines and written as
 * files for run.py. */

#include "platform.h"

#include "hap_internal.h"
#include "plist.h"
#include "rtsp_metadata.h"
#include "rtsp_policy.h"

int64_t test_now_us;

static int failures;

#define CHECK(cond, ...)                            \
  do {                                              \
    if (!(cond)) {                                  \
      failures++;                                   \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                          \
      printf("\n");                                 \
    }                                               \
  } while (0)

// ---------------------------------------------------------------------------
// Mocks for rtsp_metadata.c
// ---------------------------------------------------------------------------

static bool mock_playing;
static bool mock_have_rendered;
static uint32_t mock_rendered;
static playback_metadata_t last_meta;
static int meta_events;

bool audio_receiver_is_playing(void) {
  return mock_playing;
}

bool audio_receiver_get_rendered_rtp(uint32_t *rtp) {
  if (!mock_have_rendered) {
    return false;
  }
  *rtp = mock_rendered;
  return true;
}

void playback_events_emit(playback_source_t source, playback_event_t event,
                          const playback_event_data_t *data) {
  if (source == PLAYBACK_SOURCE_AIRPLAY && event == PLAYBACK_EVENT_METADATA &&
      data) {
    last_meta = data->metadata;
    meta_events++;
  }
}

static const char *out_dir;

static void write_file(const char *name, const uint8_t *data, size_t len) {
  char path[512];
  snprintf(path, sizeof(path), "%s/%s", out_dir, name);
  FILE *f = fopen(path, "wb");
  if (!f || fwrite(data, 1, len, f) != len) {
    printf("  FAIL cannot write %s\n", path);
    failures++;
  }
  if (f) {
    fclose(f);
  }
}

static void print_hex(const char *label, const uint8_t *p, size_t n) {
  printf("%s ", label);
  for (size_t i = 0; i < n; i++) {
    printf("%02x", p[i]);
  }
  printf("\n");
}

// ---------------------------------------------------------------------------

static void test_preemption(void) {
  // Every combination of the eight inputs against the rule written out.
  int cases = 0;
  for (int bits = 0; bits < 256; bits++) {
    const rtsp_preempt_input_t in = {
        .has_owner = bits & 1,
        .owner_is_self = (bits & 1) && (bits & 2),
        .owner_closed = bits & 4,
        .owner_ended = bits & 8,
        .same_sender = bits & 16,
        .deny_interruptions = bits & 32,
        .self_closing = bits & 64,
        .remote_control_only = bits & 128,
    };
    rtsp_preempt_t want;
    if (in.self_closing) {
      want = RTSP_PREEMPT_CLOSING;
    } else if (in.owner_is_self) {
      want = RTSP_PREEMPT_PROCEED;
    } else if (in.remote_control_only) {
      want = RTSP_PREEMPT_REMOTE_ONLY;
    } else if (!in.has_owner) {
      want = RTSP_PREEMPT_PROCEED;
    } else if (in.owner_closed || in.owner_ended || in.same_sender) {
      want = RTSP_PREEMPT_TAKEOVER;
    } else {
      want =
          in.deny_interruptions ? RTSP_PREEMPT_REFUSE : RTSP_PREEMPT_TAKEOVER;
    }
    const rtsp_preempt_t got = rtsp_preempt_decide(&in);
    CHECK(got == want, "preempt case 0x%02x", bits);
    // Only a SETUP that goes ahead gets 0; anything else is answered.
    const int status = rtsp_preempt_status(got);
    CHECK((status == 0) ==
              (got == RTSP_PREEMPT_PROCEED || got == RTSP_PREEMPT_TAKEOVER),
          "status for case 0x%02x", bits);
    cases++;
  }
  CHECK(rtsp_preempt_status(RTSP_PREEMPT_REFUSE) == 453, "453");
  CHECK(rtsp_preempt_status(RTSP_PREEMPT_CLOSING) == 455, "455");
  CHECK(rtsp_preempt_status(RTSP_PREEMPT_REMOTE_ONLY) == 403, "403");
  // The cases that matter, spelled out.
  const rtsp_preempt_input_t idle = {0};
  CHECK(rtsp_preempt_decide(&idle) == RTSP_PREEMPT_PROCEED, "idle");
  const rtsp_preempt_input_t second_phone = {.has_owner = true};
  CHECK(rtsp_preempt_decide(&second_phone) == RTSP_PREEMPT_TAKEOVER,
        "last sender wins");
  const rtsp_preempt_input_t denied = {.has_owner = true,
                                       .deny_interruptions = true};
  CHECK(rtsp_preempt_decide(&denied) == RTSP_PREEMPT_REFUSE, "453");
  const rtsp_preempt_input_t reconnect = {
      .has_owner = true, .same_sender = true, .deny_interruptions = true};
  CHECK(rtsp_preempt_decide(&reconnect) == RTSP_PREEMPT_TAKEOVER,
        "the same phone reconnecting");
  // Denied interruptions protect a session, not only a playing one: the
  // decision has no "playing" input, as Apple's has none.
  const rtsp_preempt_input_t paused_owner = {.has_owner = true,
                                             .deny_interruptions = true};
  CHECK(rtsp_preempt_decide(&paused_owner) == RTSP_PREEMPT_REFUSE,
        "a paused session is protected too");
  // The old connection, its session taken over, may not claim it back -- not
  // even as the same sender, which would otherwise always win.
  const rtsp_preempt_input_t taken_over = {
      .self_closing = true, .has_owner = true, .same_sender = true};
  CHECK(rtsp_preempt_decide(&taken_over) == RTSP_PREEMPT_CLOSING,
        "a taken-over connection gets 455");
  // A remote-control-only SETUP neither takes over nor gets 453, and does
  // not claim an idle speaker either.
  const rtsp_preempt_input_t rc_busy = {.remote_control_only = true,
                                        .has_owner = true};
  const rtsp_preempt_input_t rc_denied = {.remote_control_only = true,
                                          .has_owner = true,
                                          .deny_interruptions = true};
  const rtsp_preempt_input_t rc_idle = {.remote_control_only = true};
  CHECK(rtsp_preempt_decide(&rc_busy) == RTSP_PREEMPT_REMOTE_ONLY,
        "remote-control-only does not take over");
  CHECK(rtsp_preempt_decide(&rc_denied) == RTSP_PREEMPT_REMOTE_ONLY,
        "remote-control-only is not refused as audio");
  CHECK(rtsp_preempt_decide(&rc_idle) == RTSP_PREEMPT_REMOTE_ONLY,
        "remote-control-only does not claim an idle speaker");
  printf("preemption decision table: %d cases\n", cases);
}

static void test_route(void) {
  const uint32_t modern = airplay_source_version_parse("940.18.1");
  const uint32_t old = airplay_source_version_parse("353.1");
  CHECK(modern == 9401801U, "parse %u", modern);
  CHECK(airplay_source_version_parse("354.54.6") ==
            AIRPLAY_SRCVERS_MEDIA_REMOTE,
        "threshold");
  CHECK(airplay_source_version_parse("377.40.00") == 3774000U, "srcvers");
  CHECK(airplay_source_version_parse(NULL) == 0 &&
            airplay_source_version_parse("") == 0,
        "empty");

  // AirPlay 2, modern sender: the event channel, even with DACP around.
  CHECK(airplay_remote_route(2, modern, true, true) == AIRPLAY_ROUTE_EVENTS,
        "modern");
  CHECK(airplay_remote_route(2, 0, true, false) == AIRPLAY_ROUTE_EVENTS,
        "unknown version");
  // Old sender: DACP first, events only as the fallback.
  CHECK(airplay_remote_route(2, old, true, true) == AIRPLAY_ROUTE_DACP, "old");
  CHECK(airplay_remote_route(2, old, true, false) == AIRPLAY_ROUTE_EVENTS,
        "old, no DACP");
  // AirPlay 1: DACP or nothing.
  CHECK(airplay_remote_route(1, 0, false, true) == AIRPLAY_ROUTE_DACP, "v1");
  CHECK(airplay_remote_route(1, 0, true, false) == AIRPLAY_ROUTE_NONE,
        "v1 never uses events");
  CHECK(airplay_remote_route(2, modern, false, false) == AIRPLAY_ROUTE_NONE,
        "unreachable");

  const char *want_4cc[] = {"play", "paus", "plps", "stop", "nitm", "pitm"};
  const char *want_dacp[] = {"play", "pause",    "playpause",
                             "stop", "nextitem", "previtem"};
  for (int c = 0; c <= AIRPLAY_MR_PREVIOUS; c++) {
    CHECK(strcmp(airplay_mr_fourcc((airplay_mr_command_t)c), want_4cc[c]) == 0,
          "4cc %d", c);
    CHECK(strcmp(airplay_mr_dacp_word((airplay_mr_command_t)c), want_dacp[c]) ==
              0,
          "dacp %d", c);
  }
  CHECK(airplay_mr_pauses_locally(AIRPLAY_MR_PAUSE) &&
            airplay_mr_pauses_locally(AIRPLAY_MR_STOP) &&
            airplay_mr_pauses_locally(AIRPLAY_MR_TOGGLE) &&
            !airplay_mr_pauses_locally(AIRPLAY_MR_PLAY) &&
            !airplay_mr_pauses_locally(AIRPLAY_MR_NEXT),
        "local pause");
  CHECK(airplay_volume_db_to_slider(-144.0f) == 0.0f &&
            airplay_volume_db_to_slider(-30.0f) == 0.0f &&
            airplay_volume_db_to_slider(0.0f) == 1.0f &&
            airplay_volume_db_to_slider(-15.0f) == 0.5f,
        "slider");
}

static void test_commands(void) {
  uint8_t body[192];
  const airplay_mr_command_plist_t toggle = {
      .fourcc = airplay_mr_fourcc(AIRPLAY_MR_TOGGLE),
      .mr_command = AIRPLAY_MR_TOGGLE};
  size_t len = bplist_build_mr_command(body, sizeof(body), &toggle);
  CHECK(len > 0, "toggle");
  write_file("cmd_toggle.bplist", body, len);

  const airplay_mr_command_plist_t next = {
      .fourcc = airplay_mr_fourcc(AIRPLAY_MR_NEXT),
      .mr_command = AIRPLAY_MR_NEXT};
  len = bplist_build_mr_command(body, sizeof(body), &next);
  CHECK(len > 0, "next");
  write_file("cmd_next.bplist", body, len);

  const airplay_mr_command_plist_t dvlc = {.fourcc = "dvlc",
                                           .mr_command = -1,
                                           .has_volume = true,
                                           .volume = 0.75,
                                           .is_muted = false};
  len = bplist_build_mr_command(body, sizeof(body), &dvlc);
  CHECK(len > 0, "dvlc");
  write_file("cmd_dvlc.bplist", body, len);

  // Too small a buffer fails cleanly at every size.
  for (size_t cap = 0; cap < len; cap++) {
    uint8_t small[192];
    CHECK(bplist_build_mr_command(small, cap, &dvlc) == 0, "cap %zu", cap);
  }
}

static void test_events_keys(void) {
  uint8_t ikm[64];
  for (int i = 0; i < 64; i++) {
    ikm[i] = (uint8_t)(i * 7 + 3);
  }
  uint8_t write_key[32];
  uint8_t read_key[32];
  hap_derive_events_keys(ikm, 32, write_key, read_key);
  print_hex("EVENTS32_WRITE", write_key, 32);
  print_hex("EVENTS32_READ", read_key, 32);
  hap_derive_events_keys(ikm, 64, write_key, read_key);
  print_hex("EVENTS64_WRITE", write_key, 32);
  print_hex("EVENTS64_READ", read_key, 32);
  CHECK(memcmp(write_key, read_key, 32) != 0, "directions differ");
}

static void test_advertisement(void) {
  const uint64_t v1 = airplay_features_compute(true, true);
  const uint64_t v2 = airplay_features_compute(false, false);
  const uint64_t v2_art = airplay_features_compute(false, true);
  printf("FEATURES_V1 %llx\n", (unsigned long long)v1);
  printf("FEATURES_V2 %llx\n", (unsigned long long)v2);
  printf("FEATURES_V2_ART %llx\n", (unsigned long long)v2_art);
  char ex[24];
  CHECK(airplay_features_ex(v2_art, ex, sizeof(ex)), "featuresEx");
  printf("FEATURES_EX %s\n", ex);
  char tiny[4];
  CHECK(!airplay_features_ex(v2_art, tiny, sizeof(tiny)), "small buffer");
  CHECK(strcmp(airplay_metadata_types(true), "0,1,2") == 0 &&
            strcmp(airplay_metadata_types(false), "0,2") == 0,
        "md");
  CHECK(strcmp(AIRPLAY_MODEL, "AudioAccessory5,1") == 0, "model");

  const uint8_t macs[3][6] = {{0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56},
                              {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x57},
                              {0, 0, 0, 0, 0, 0}};
  for (int m = 0; m < 3; m++) {
    char pi[37];
    char again[37];
    airplay_pairing_identity(macs[m], pi);
    airplay_pairing_identity(macs[m], again);
    CHECK(strcmp(pi, again) == 0, "stable");
    CHECK(strcmp(pi, "00000000-0000-0000-0000-000000000000") != 0, "not zero");
    printf("PI %02x%02x%02x%02x%02x%02x %s\n", macs[m][0], macs[m][1],
           macs[m][2], macs[m][3], macs[m][4], macs[m][5], pi);
  }

  const airplay_txt_input_t in = {
      .device_id = "24:0A:C4:12:34:56",
      .features = "0x405C4A00,0x14340",
      .model = AIRPLAY_MODEL,
      .pk_hex = "00112233",
      .pi = "PI-VALUE",
      .metadata_types = "0,2",
  };
  airplay_txt_item_t items[16];
  size_t n = airplay_txt_airplay(&in, items, 16);
  CHECK(n > 0, "airplay TXT");
  for (size_t i = 0; i < n; i++) {
    printf("TXT_AIRPLAY %s=%s\n", items[i].key, items[i].value);
  }
  CHECK(airplay_txt_airplay(&in, items, n - 1) == 0, "airplay TXT overflow");
  n = airplay_txt_raop(&in, items, 16);
  CHECK(n > 0, "raop TXT");
  for (size_t i = 0; i < n; i++) {
    printf("TXT_RAOP %s=%s\n", items[i].key, items[i].value);
  }
}

static void test_buffer_size(void) {
  // Realtime: omitted, as Apple's reply has none.
  CHECK(airplay_audio_buffer_size(96, 2, 44100) == 0, "realtime");
  // AAC: five seconds at the budgeted rate.
  CHECK(airplay_audio_buffer_size(103, 4, 44100) ==
            AIRPLAY_AAC_BYTES_PER_SEC * AIRPLAY_AUDIO_BUFFER_SECONDS,
        "aac %u", airplay_audio_buffer_size(103, 4, 44100));
  CHECK(airplay_audio_buffer_size(103, 8, 48000) ==
            airplay_audio_buffer_size(103, 4, 44100),
        "eld");
  // Lossless: PCM rate, capped.
  CHECK(airplay_audio_buffer_size(103, 2, 44100) == AIRPLAY_AUDIO_BUFFER_MAX,
        "alac");
  CHECK(airplay_audio_buffer_size(103, 2, 4000) == AIRPLAY_AUDIO_BUFFER_MIN,
        "floor");
  CHECK(airplay_audio_buffer_size(103, -1, -5) == AIRPLAY_AUDIO_BUFFER_MAX,
        "unknown");
  printf("audioBufferSize: AAC %u, ALAC %u, realtime omitted\n",
         airplay_audio_buffer_size(103, 4, 44100),
         airplay_audio_buffer_size(103, 2, 44100));
}

static void test_progress(void) {
  uint32_t elapsed = 0;
  uint32_t duration = 0;
  // A track that starts 10 s before the 32-bit wrap and runs 200 s.
  const uint32_t start = 0xFFFFFFFFU - 44100U * 10U + 1U;
  const uint32_t current = start + 44100U * 61U; // wrapped
  const uint32_t end = start + 44100U * 200U;    // wrapped
  CHECK(current < start && end < start, "the test wraps");
  CHECK(
      airplay_progress_seconds(start, current, end, 44100, &elapsed, &duration),
      "ok");
  CHECK(elapsed == 61 && duration == 200, "wrap: %u/%u", elapsed, duration);
  // The stream's rate, not 44100.
  CHECK(airplay_progress_seconds(1000, 1000 + 48000 * 30, 1000 + 48000 * 90,
                                 48000, &elapsed, &duration),
        "48k");
  CHECK(elapsed == 30 && duration == 90, "48k: %u/%u", elapsed, duration);
  // The playhead before the start reads 0, not four billion.
  CHECK(airplay_progress_seconds(5000, 4000, 5000 + 44100, 44100, &elapsed,
                                 &duration) &&
            elapsed == 0 && duration == 1,
        "before start");
  CHECK(!airplay_progress_seconds(0, 0, 0, 0, &elapsed, &duration), "rate 0");

  // What is being played wins when it lies in the track...
  CHECK(airplay_progress_current(start, end, current, true,
                                 start + 44100U * 58U) == start + 44100U * 58U,
        "rendered");
  // ...not when it does not, or is unknown.
  CHECK(airplay_progress_current(start, end, current, true, start - 10U) ==
            current,
        "rendered before start");
  CHECK(airplay_progress_current(start, end, current, true, end + 44100U) ==
            current,
        "rendered past end");
  CHECK(airplay_progress_current(start, end, current, false, 0) == current,
        "no render");
  printf("progress across an RTP wrap: %u s of %u s\n", 61U, 200U);
}

// DMAP: tag, 32-bit big-endian length, value.
static size_t dmap_item(uint8_t *p, const char *tag, const void *value,
                        uint32_t len) {
  memcpy(p, tag, 4);
  p[4] = (uint8_t)(len >> 24);
  p[5] = (uint8_t)(len >> 16);
  p[6] = (uint8_t)(len >> 8);
  p[7] = (uint8_t)len;
  memcpy(p + 8, value, len);
  return 8 + len;
}

static size_t dmap_track(uint8_t *out, const char *title, const char *artist,
                         uint32_t duration_ms, int caps) {
  uint8_t inner[512];
  size_t n = 0;
  n += dmap_item(inner + n, "minm", title, (uint32_t)strlen(title));
  n += dmap_item(inner + n, "asar", artist, (uint32_t)strlen(artist));
  n += dmap_item(inner + n, "asal", "Album", 5);
  n += dmap_item(inner + n, "xxxx", "skip", 4); // unknown: skipped
  const uint8_t ms[4] = {(uint8_t)(duration_ms >> 24),
                         (uint8_t)(duration_ms >> 16),
                         (uint8_t)(duration_ms >> 8), (uint8_t)duration_ms};
  n += dmap_item(inner + n, "astm", ms, 4);
  if (caps >= 0) {
    const uint8_t c = (uint8_t)caps;
    n += dmap_item(inner + n, "caps", &c, 1);
  }
  return dmap_item(out, "mlit", inner, (uint32_t)n);
}

static void test_dmap_and_merge(void) {
  uint8_t buf[600];
  airplay_dmap_t d;
  size_t n = dmap_track(buf, "Song", "Artist", 245000, 1);
  CHECK(airplay_dmap_parse(buf, n, &d), "parse");
  CHECK(strcmp(d.title, "Song") == 0 && strcmp(d.artist, "Artist") == 0 &&
            strcmp(d.album, "Album") == 0,
        "strings");
  CHECK(d.has_duration && d.duration_ms == 245000, "astm");
  CHECK(d.has_caps && d.caps == 1, "caps");
  // Truncated: no read past the end (ASan), and whatever came before stands.
  for (size_t cut = 0; cut < n; cut++) {
    (void)airplay_dmap_parse(buf, cut, &d);
  }
  // A hostile length.
  uint8_t bad[16] = {'m', 'l', 'i', 't', 0xFF, 0xFF, 0xFF, 0xF0};
  CHECK(!airplay_dmap_parse(bad, sizeof(bad), &d), "hostile");
  CHECK(airplay_dmap_caps_paused(2) && airplay_dmap_caps_paused(3) &&
            !airplay_dmap_caps_paused(1) && !airplay_dmap_caps_paused(4),
        "caps table");

  // The merged state: text, then progress, then text again for the same
  // track -- every event carries everything.
  rtsp_metadata_reset();
  mock_playing = true;
  mock_have_rendered = false;
  test_now_us = 1000000;
  n = dmap_track(buf, "Song", "Artist", 245000, 1);
  rtsp_metadata_dmap(buf, n);
  CHECK(strcmp(last_meta.title, "Song") == 0 && last_meta.duration_secs == 245,
        "text: %s %u", last_meta.title, last_meta.duration_secs);
  CHECK(last_meta.position_secs == 0, "no position yet");

  const uint32_t start = 0xFFFF0000U;
  char progress[64];
  snprintf(progress, sizeof(progress), "%u/%u/%u", start, start + 48000U * 20U,
           start + 48000U * 240U);
  rtsp_metadata_progress(progress, 48000);
  CHECK(strcmp(last_meta.title, "Song") == 0 &&
            strcmp(last_meta.artist, "Artist") == 0,
        "progress keeps the text");
  CHECK(last_meta.position_secs == 20 && last_meta.duration_secs == 240,
        "progress: %u/%u", last_meta.position_secs, last_meta.duration_secs);

  // What is audible, not what the sender claims.
  mock_have_rendered = true;
  mock_rendered = start + 48000U * 18U;
  rtsp_metadata_progress(progress, 48000);
  CHECK(last_meta.position_secs == 18, "as played: %u",
        last_meta.position_secs);

  // Time passes while playing: a text update reports the position on.
  test_now_us += 5000000;
  n = dmap_track(buf, "Song", "Artist", 245000, -1);
  rtsp_metadata_dmap(buf, n);
  CHECK(last_meta.position_secs == 23 && last_meta.duration_secs == 240,
        "counting on: %u/%u", last_meta.position_secs, last_meta.duration_secs);

  // Paused (caps 2): the position freezes.
  n = dmap_track(buf, "Song", "Artist", 245000, 2);
  rtsp_metadata_dmap(buf, n);
  CHECK(last_meta.paused, "paused");
  const uint32_t frozen = last_meta.position_secs;
  test_now_us += 30000000;
  rtsp_metadata_dmap(buf, n);
  CHECK(last_meta.position_secs == frozen, "frozen: %u vs %u",
        last_meta.position_secs, frozen);

  // Another track: the old one's artist and length do not carry over.
  n = dmap_track(buf, "Next", "Other", 100000, 1);
  rtsp_metadata_dmap(buf, n);
  CHECK(strcmp(last_meta.title, "Next") == 0 &&
            strcmp(last_meta.artist, "Other") == 0 && !last_meta.paused,
        "next track");
  CHECK(last_meta.duration_secs == 100 && last_meta.position_secs == 0,
        "next track starts clean: %u/%u", last_meta.duration_secs,
        last_meta.position_secs);

  rtsp_metadata_reset();
  CHECK(meta_events > 0, "events");
  printf("DMAP and merged now-playing: %d events\n", meta_events);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    return 2;
  }
  out_dir = argv[1];
  test_preemption();
  test_route();
  test_commands();
  test_events_keys();
  test_advertisement();
  test_buffer_size();
  test_progress();
  test_dmap_and_merge();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("C checks: PASS\n");
  return 0;
}

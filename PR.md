# Fix AirPlay 2 multi-room sync and audio pops

Fixes #54.

## The main fix

Realtime streams (type 96 — what iPhones send) schedule audio on the
sender's source timeline: receivers are expected to play each frame
**latencyMin samples (11025 = 250 ms) after** its anchor time. This
firmware played at the anchor directly, so it ran ~250 ms ahead of every
other speaker in a group. Solo playback was unaffected, which is why many
users never saw it — and the device's own timing logs are measured against
the same anchor, so they always looked perfect.

Three independent measurements agree on the constant: steady buffer depth
(measured 1716 ms ≈ 88200−11025 samples), first-frame earliness at stream
start (~1750 ms), and the empirical +300 ms offset found by hand in #54
(net +254 ms after that build's hardware compensation).

`latencyMin` is parsed from SETUP when present, defaulting to 11025.
Buffered streams (type 103) are unchanged.

## Supporting fixes

- **Position servo** — corrects small standing offsets (post-stall
  residuals, clock drift) by trimming one sample per 4 frames at the
  frame's quietest point. Engages above 5 ms error, disengages below
  1.5 ms, immune to one-sided measurement spikes.
- **Precise starts** — playout begins within ±4 ms of schedule instead of
  up to the full jitter threshold early; stale packet islands at the
  buffer head (leftover pre-flush retransmissions) are skipped instead of
  played as a blip.
- **Packet loss concealment** — unrecovered losses insert exact-length
  silence at the right time instead of skipping (which popped and shifted
  position 8 ms per lost packet). UDP receive queue deepened 6 → 32
  packets so the device's own TCP traffic can't starve the audio socket.
- **Pop fixes** — volume changes ramp over ~3 ms instead of stepping
  (zipper clicks); I2S `auto_clear` so underruns play silence instead of
  looping stale DMA contents.
- **Correctness** — output writes bounded to the stereo buffer size
  (overrun with a malformed channel count); buffered-stream late threshold
  10 → 25 ms (was below the DMA ring's own jitter, causing drop storms);
  DMA latency modeled as ring midpoint instead of full ring (~3 ms bias).
- **Web log viewer** — never worked on IDF 5.x (the WS handshake no longer
  invokes the URI handler, so clients were never registered); the
  broadcaster now discovers WebSocket sessions each tick.
- **Diagnostics** — once-per-second `Playout:` line (err / buffered /
  depth / ptp_gap / gaps) used to find and validate all of the above.

## Validation

Tested on ESP32-S3 + PCM5102A against other AirPlay 2 receivers:
`err` holds 0 ± 4 ms across track changes, buffer depth sits at the
predicted ~1966 ms, `gaps=0` in normal sessions, and multi-room sync is
audibly correct. Logs are in #54.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

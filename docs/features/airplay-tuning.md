# AirPlay tuning

Advanced timing and metadata options live under **AirPlay Receiver → AirPlay Protocol** in
`menuconfig`. The defaults suit most setups — change these only if you hear drop-outs or
want to control what metadata is received.

## Cover art

Album cover art is **disabled by default**. Most receivers have no screen, or only a small
OLED, and pulling artwork over the RTSP connection can stall the audio pipeline and cause
drop-outs — especially on unbuffered AirPlay 1 / realtime streams.

When disabled, the receiver drops artwork type `1` from its advertised `md` txt record so
senders do not transmit cover art at all, and ignores any that arrives anyway. Track title,
artist, album and progress metadata are always received.

To enable it, for instance when you have a [TFT display](tft-display.md):

```bash
idf.py menuconfig
# AirPlay Receiver → AirPlay Protocol
# Enable "Enable cover-art / artwork reception"
```

Or in your sdkconfig defaults:

```ini
CONFIG_ENABLE_AIRPLAY_ARTWORK=y
```

## Early and late timing thresholds

The timing engine holds frames that arrive early, outputting silence until their scheduled
play time, and drops frames that arrive late. The threshold controls how much slack is
allowed before a frame is held or dropped.

Buffered AirPlay 2 streams (AAC) have a deep jitter buffer and can use a tight threshold
for precise sync. Unbuffered realtime streams (ALAC over UDP) have almost no buffer, so a
tight threshold causes audible drop-outs whenever the pipeline stalls — when metadata
arrives, for example.

| Option | Default | Applies to |
| --- | --- | --- |
| `CONFIG_AIRPLAY_TIMING_THRESHOLD_MS` | 10 ms | Buffered streams (AAC) |
| `CONFIG_AIRPLAY_RT_TIMING_THRESHOLD_MS` | 50 ms | Unbuffered realtime streams (ALAC) |

If you still hear drop-outs on AirPlay 1 / realtime playback, increase
`CONFIG_AIRPLAY_RT_TIMING_THRESHOLD_MS`, at the cost of slightly looser sync.

## Forcing AirPlay v1

Classic AirPlay v1 makes the receiver advertise itself as a plain RAOP receiver. There are
two reasons to do this:

- **Hardware buttons.** iOS only sends DACP headers in v1 mode, so this is what makes
  [hardware buttons](buttons.md#read-this-first-the-airplay-v1-requirement) work.
- **Apple Music for Windows.** It is a RAOP-only sender and refuses any device that
  advertises the `_airplay._tcp` service, reporting that the device "is not compatible
  with this version of AMPLibraryAgent". The device still appears in its picker and it
  still opens an RTSP connection — it sends `OPTIONS` with an `Apple-Challenge` and then
  walks away.

The mode lives in NVS, so no rebuild is needed. Open the web interface and pick
**Configuration → AirPlay → Mode**, then restart the device. The mDNS records and the RTSP
listening port are both built at startup, so the change only takes effect on the next boot.

In v1 mode the receiver mirrors a classic RAOP advertisement: no `_airplay._tcp`
service, a shairport-sync-style `_raop._tcp` TXT record, `Server: AirTunes/105.1` on RTSP
responses, and RTSP on port 5000 instead of 7000.

It costs you AirPlay 2 features: HomeKit pairing, encrypted transport and multi-room sync.

There is no setting that keeps both senders happy. The deciding factor for Apple Music is
the presence of `_airplay._tcp` alone — a receiver publishing a fully classic `_raop._tcp`
TXT record on port 5000 is still refused while that service is up, and starts working the
moment it is withdrawn. iOS needs the same service to negotiate AirPlay 2, and a device
publishes one advertisement, so the mode is a real either/or.

## Volume curve

The sender's volume slider runs from 0 dB at the top to −30 dB at the bottom, and sends
−144 dB for mute. By default the receiver turns it into a gain the way Apple's receivers
do: 1:1 in decibels (−10 dB on the slider is 10 dB quieter), with the bottom of the slider
and −144 both silent. This holds in software and on DACs with their own volume control, so
a speaker grouped with HomePods plays at their level for the same slider position. Changes
under 0.3 dB, 1 % of the slider, are ignored, as Apple ignores them. The stored level is
kept within −30…0 dB, so a muted session does not start the next one silent.

The curves of earlier firmware are still available as
**DAC Configuration → Volume curve → Earlier firmware's** in `menuconfig`
(`CONFIG_DAC_VOLUME_LAW_LEGACY`): the square of the slider position in software, and 2 dB
of DAC attenuation per slider dB on DACs with volume control. They are louder than Apple's
in the upper half of the slider and much quieter near the bottom.

## Starts and stops

Every start fades in over 30 ms from the first sample: a new stream, a resume, after a seek
or a skip, and after a dropout. Apple's receiver does the same.

Every stop fades out over 8 ms: a pause, a seek, a skip and the end of a session. Apple's
receiver cuts off hard here. This one does not, because many boards have no mute line —
a PCM5102A board with XSMT bridged high among them — and a hard cut to silence clicks. A
board's amplifier enable or mute line is only switched off 80 ms after the stop, once the
fade has left the DMA queue, and is switched on again before the first sample of the next
start.

See also [Output latency](output-latency.md).

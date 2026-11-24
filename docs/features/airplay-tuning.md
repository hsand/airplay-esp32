# AirPlay tuning

Advanced timing and metadata options live under **AirPlay Receiver → AirPlay Protocol** in
`menuconfig`. The defaults suit most setups — change these only if you hear drop-outs or
want to control what metadata is received.

## Cover art

Album cover art is **disabled by default**. Most receivers have no screen, or only a small
OLED, and pulling artwork over the RTSP connection can stall the audio pipeline and cause
drop-outs — especially on unbuffered AirPlay 1 / realtime streams.

When disabled, the receiver drops artwork type `1` from its advertised `md` txt record and
leaves out feature bit 15 (artwork), so senders do not transmit cover art at all, and it
ignores any that arrives anyway. Track title, artist, album, track length, play state and
progress are always received (feature bits 16 and 17, `md` 0 and 2).

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

## When a second device plays

Several phones can see and probe the speaker at once without disturbing each other: a
phone opening the AirPlay picker, pairing, or asking for the speaker's details never stops
what is playing. Only a device that actually **starts playing** to the speaker takes it
over, at which point the first one is disconnected. This is what Apple's receivers do; the
last device to press play wins.

To keep the current listener in charge instead, open the web interface and pick
**Configuration → When another device plays → Refuse while another is connected**. A second
device is then turned away (RTSP status 453, which iOS shows as the speaker being in use)
for as long as the first one holds the speaker, playing or paused, as on Apple's receivers.
It applies at once. A phone that lost its Wi-Fi and comes back is still
let in, since it is taking back its own session.

## Device model

The speaker presents itself to Apple devices as a HomePod mini (`AudioAccessory5,1`), as
it always has. Buffered audio, multi-room timing and pairing follow the advertised
features, not the model.

## Buffer the sender may queue

For a buffered AirPlay 2 stream the receiver tells the sender how much audio it may queue
ahead (`audioBufferSize`). Apple's receivers allow 120 seconds. Here it is about five
seconds for AAC (200 kB) and capped at 512 kB (about three seconds) for lossless, because
everything already queued has to cross the network again after a seek or a skip before the
new position can play. On an ESP32 sharing its radio with Bluetooth that drain is slow, and
1 MB was measured to cost about two seconds per seek. The queued audio waits in the
sender and in TCP, not in the speaker's memory, so a smaller value costs no PSRAM and five
seconds still rides out a Wi-Fi stall. Realtime streams are paced by the sender and get no
value, as on Apple's receivers.

## Forcing AirPlay v1

Classic AirPlay v1 makes the receiver advertise itself as a plain RAOP receiver. The main
reason to do this:

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

!!! note "Changed after v0.3.0"
    Apple's curve is now the default on every board, and the bottom of the slider gets
    louder. The lowest step of an iPhone's volume buttons (−28 dB from the sender) used to
    play at about −48 dB in software and about −83 dB on a DAC with its own volume control;
    it now plays at −28 dB, and anything at or below −30 dB is silent. To keep the old
    behaviour, build with `CONFIG_DAC_VOLUME_LAW_LEGACY=y`, as below.

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

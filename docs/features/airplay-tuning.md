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
**Configuration → When another device plays → Refuse while playing**. A second device is
then turned away (RTSP status 453, which iOS shows as the speaker being in use) until the
first one stops. It applies at once. A phone that lost its Wi-Fi and comes back is still
let in, since it is taking back its own session.

## Device model

Apple senders decide a few things from the model a receiver claims. The setting is under
**Configuration → Appears to Apple devices as**, and takes effect after a restart.

| Setting | Model string | What it means |
| --- | --- | --- |
| **HomePod mini** (default) | `AudioAccessory5,1` | What this firmware has always claimed. iOS treats the speaker as an Apple HomePod. |
| **Generic AirPlay speaker** | `AirPlayGeneric1,1` | Apple's own name for a receiver of unknown make. iOS treats it as a third-party speaker. |

Buffered audio, multi-room timing and pairing are decided by the advertised features, not
by the model, so both settings play the same way. The differences:

- **HomePod mini** makes iOS expect HomePod-only behaviour this firmware does not have,
  such as full MediaRemote control and HomePod cluster logic. iOS also skips its volume
  safety clamp for Apple speakers, so a session starts at exactly the speaker's last
  volume.
- **Generic** is the honest answer and avoids those HomePod paths. iOS applies its
  third-party volume clamp: a session that would start very quiet is raised to −27 dB, and
  one at full volume is lowered to −20 dB.
- **Risk with Generic:** iOS's pairing code consults whether the receiver is an Apple
  audio accessory when deciding whether to ask for a PIN. This has not been seen on a
  device. If your iPhone asks for a PIN after switching to Generic, switch back to HomePod
  mini; this firmware has no PIN to give.

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

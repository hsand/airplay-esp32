# Output latency

An AirPlay sender tells every speaker in a group *when* each sample should be heard. Each
receiver then has to start a sample early by however long its own output path takes, so
that the sound — not the data — lands on time. Apple's receivers add up the latency of
every stage after their buffer; this receiver does the same.

Getting it right matters most in a group with Apple speakers in one room. A few
milliseconds of error is far too little to see against video, but it is enough to pull the
stereo image towards one speaker and to colour the sound where two speakers overlap.

## What is compensated

Every block of audio is scheduled for the moment its first sample will be heard:

| Part | Where the figure comes from |
| --- | --- |
| DMA queue | Measured live from the I2S driver's completion cursor |
| I2S FIFO | The 32–64 frames the transmit FIFO still holds once the DMA counts a buffer as sent |
| Resampler | Half its filter length, only while it converts 44.1 kHz to 48 kHz |
| Per-output delay | The part common to both outputs, when one is set |
| DAC | The DAC's own filter delay, from its driver or the preset below |
| Trim | Your correction, for anything after the DAC |

Earlier firmware assumed a fixed 5 ms after the DMA queue, which was never measured.
On a PCM5102A board the real figure is about 1.2 ms, so those builds played about 3.8 ms
early.

When a stream starts, the log shows the breakdown, for example:

```text
Output latency: DMA 43537 us (modelled; live depth used) + FIFO 726 + resampler 0 +
output delay 0 + DAC 499 (preset: PCM510xA normal filter) + trim 0 = 44762 us at 44100 Hz
```

The **Configuration** page shows the same parts under the trim control.

## The DAC's delay

A DAC with a driver reports its own delay where the datasheet gives one. The PCM5122 (the
HiFi-ESP32-Plus and Amped-ESP32-Plus boards) reads back which interpolation filter it is
running and reports that filter's delay: 20 samples for the normal filter, 3.5 for the
low-latency one (TI SLAS763C §7.5).

The TAS57xx, TAS58xx, ES8311, ES8388 and MAX98357A datasheets give no figure, so those
boards report the DAC as unknown and count it as 0. Their filters typically take well
under a millisecond. Calibrate with the trim if that matters to you.

### Plain I2S DACs: the preset

A DAC wired straight to the I2S pins, with no driver, cannot say what it is. Pick it under
**DAC Configuration → Plain I2S DAC output delay** in `menuconfig`:

| Preset | Delay | Source |
| --- | --- | --- |
| TI PCM5100A/5101A/5102A, normal filter (FLT low) | 22 samples | TI SLAS859C, Tables 4–6 |
| TI PCM5100A/5101A/5102A, low-latency filter (FLT high) | 3.5 samples | TI SLAS859C, Tables 7–9 |
| Cirrus CS4344/CS4345/CS4348 | 10 samples (5 above 50 kHz) | Cirrus DS613F2, page 6 |
| Custom | `CONFIG_DAC_PLAIN_DELAY_CUSTOM_Q4`, in 1/16 samples | your own measurement |
| Unknown | 0 | — |

The PCM510xA datasheet gives 20 samples in its filter summary (§8.5) but 22 in the
per-mode tables. The preset uses 22.

The preset applies only when no driver reports a figure. It defaults to the PCM510xA
normal filter on every build that is documented with one: the ESP32-S3, ESP32-S2, WROVER
and XIAO ESP32-C5 builds, and the PCM5100A boards (HiFi-ESP32, HiFi-Esparagus,
Amped-ESP32, Amped-Esparagus), whose schematics tie FLT to ground. Everything else
defaults to Unknown.

### PCM5102A boards

The common purple PCM5102A board has four solder bridges. Their expected positions are in
the photo under [Check the DAC board](../getting-started/shopping-list.md#check-the-dac-board).
Two of them matter here:

- **FLT** selects the interpolation filter, which sets the latency: low (the photo) is the
  normal filter, 22 samples; high is the low-latency filter, 3.5 samples. If you change it,
  change the preset to match.
- **XSMT** is the soft-mute input. Bridged high, as in the photo, the DAC is never muted,
  so the receiver's own fade-out at every pause is the only thing between you and a click.

## The trim

The trim covers anything the receiver cannot know about: an amplifier with its own DSP, an
AV receiver behind S/PDIF or USB, a soundbar, an unknown DAC.

Set it on the **Configuration** page under **Output latency trim**, in milliseconds, from
−50 to +50. **Positive** means the speaker sounds late, so it starts playing that much
earlier. The value is stored and survives a restart.

It applies at once, to every output (I2S, S/PDIF and USB), without a restart and without a
click:

- under 10 ms, the drift servo eases playout onto the new time with sub-sample trims,
  far too slow to hear as a pitch change. A 2 ms change takes about 10 seconds to settle;
- from 10 ms up, playout moves in one step, hidden by a short fade.

The setting is also available as `GET`/`POST /api/audio/latency` with `{"trim_us": n}`.

## Calibrating

### Against an Apple speaker

This is the accurate method, to about 0.1 ms. You need a HomePod or HomePod mini (or an
Apple TV playing through HomePods), a phone that can record audio, and a computer with
[Audacity](https://www.audacityteam.org/).

1. Group this speaker with the HomePod in the Home app or from Control Center. Do not use
   a TV's own speakers as the reference: a TV adds its own delay.
2. Play a click track: one short, sharp click a second, for about 30 seconds, at a
   moderate level.
3. Hold the phone's microphone the same distance from both speakers. If you cannot,
   measure both distances and correct by 2.9 ms per metre of difference.
4. Record a mono WAV file at 44.1 or 48 kHz.
5. In Audacity, zoom in on each pair of clicks and measure the time from the HomePod's
   click to this speaker's. Take the median of at least ten pairs. A positive figure means
   this speaker is late.
6. Add that figure to the trim. Record again, and repeat until the two clicks are less than
   0.3 ms apart.

When they line up, a steady 1 kHz tone from both speakers should sound even, without the
hollow, phasey colour of two copies slightly apart.

### Against video

A rougher check, to about 5 ms. Play an A/V sync test clip (a white flash and a beep on the
same frame, once a second) on an iPhone, with the sound sent to this speaker. Film the
iPhone's screen and the speaker with a second phone in 240 fps slow motion, the phone close
to the speaker. Step through the video and compare the flash frame with the start of the
beep. At 240 fps a frame is 4.2 ms. Expect an error under 10 ms after calibrating against
an Apple speaker; much more than 20 ms points at the sender or the clip, not the receiver.

### With a scope

For a new DAC preset: play a single full-scale sample after silence, and compare the I2S
data line with the DAC's analogue output on a scope. The difference is the DAC's delay,
which belongs in the Custom preset, or in a pull request adding the part.

## Related

- [Check the DAC board](../getting-started/shopping-list.md#check-the-dac-board)
- [Custom board configuration](../boards/custom.md)
- [AirPlay tuning](airplay-tuning.md)

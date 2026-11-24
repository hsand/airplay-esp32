# Architecture

## Signal flow

Audio reaches the board over the network or over Bluetooth, but from the I2S bus onward
both sources share one path.

```mermaid
flowchart LR
    iOS["iPhone / Mac<br/><small>AirPlay</small>"]
    BT["Phone / Tablet<br/><small>Bluetooth A2DP</small>"]
    ESP["ESP32"]
    DAC["DAC<br/><small>PCM5102A or TAS58xx</small>"]
    AMP["Amplifier<br/>+ Speakers"]

    iOS -->|WiFi or Ethernet| ESP
    BT -->|Bluetooth| ESP
    ESP -->|I2S| DAC
    DAC -->|Analog| AMP

    classDef src stroke:#5c6bc0,stroke-width:2px
    classDef out stroke:#26a69a,stroke-width:2px
    class iOS,BT src
    class DAC,AMP out
```

## Protocol stack

An AirPlay 2 session is really three protocols running side by side. mDNS advertises the
speaker, RTSP carries control, and PTP keeps clocks aligned. Only after HomeKit pairing
completes does audio start flowing.

```mermaid
flowchart TD
    SRC["AirPlay 2 source<br/><small>iPhone, iPad, Mac, Apple TV</small>"]

    MDNS["mDNS<br/><small>discovery</small>"]
    RTSP["RTSP<br/><small>control</small>"]
    PTP["PTP<br/><small>timing</small>"]

    HAP["HAP pairing<br/><small>transient</small>"]

    ALAC["ALAC<br/><small>realtime, UDP</small>"]
    AAC["AAC<br/><small>buffered, TCP</small>"]

    BUF["Audio buffer<br/>+ timing sync"]
    RS["Resampler<br/><small>optional</small>"]
    OUT["I2S output<br/><small>44.1 or 48 kHz</small>"]

    SRC --> MDNS & RTSP & PTP
    MDNS & RTSP & PTP --> HAP
    HAP --> ALAC & AAC
    ALAC & AAC --> BUF
    BUF --> RS --> OUT

    classDef proto stroke:#ffb300,stroke-width:2px
    classDef codec stroke:#ab47bc,stroke-width:2px
    classDef stage stroke:#5c6bc0,stroke-width:2px
    class MDNS,RTSP,PTP proto
    class ALAC,AAC codec
    class BUF,RS,OUT stage
```

## Audio pipeline

`AudioReceiver` (RTSP) → decoder → `AudioBuffer` → `AudioOutput` (I2S, S/PDIF or USB).

Two stream types run through it, and the difference between them is the single most
important thing to understand when debugging audio problems.

| Stream | Codec | Transport | Buffering | Timing threshold |
| --- | --- | --- | --- | --- |
| Buffered (AirPlay 2) | AAC | TCP | Deep jitter buffer | 10 ms |
| Realtime (AirPlay 1) | ALAC | UDP | Almost none | 50 ms |

Buffered streams tolerate a tight early/late threshold because the jitter buffer absorbs
network variation. Realtime streams have almost nothing to absorb with, so the same tight
threshold makes them drop out whenever the pipeline stalls — when metadata arrives, for
instance. That is why the two have separate settings; see
[AirPlay tuning](../features/airplay-tuning.md#early-and-late-timing-thresholds).

```mermaid
flowchart LR
    RX["AudioReceiver<br/><small>RTSP session</small>"]
    DEC["Decoder<br/><small>ALAC / AAC</small>"]
    TIME{"Timing<br/>check"}
    BUF["AudioBuffer"]
    OUT["AudioOutput<br/><small>I2S / SPDIF / USB</small>"]
    HOLD["Hold<br/><small>output silence</small>"]
    DROP["Drop frame"]

    RX --> DEC --> TIME
    TIME -->|on time| BUF --> OUT
    TIME -->|too early| HOLD --> BUF
    TIME -->|too late| DROP

    classDef bad stroke:#e53935,stroke-width:2px
    class DROP bad
```

Every source takes the same three steps on its way to the I2S bus: the channel mode
(stereo, mono sum, left or right), the [software EQ](../features/software-eq.md) when it
is built in, and software volume on a DAC with no volume control of its own. AirPlay and
Sendspin reach them through the playback task, USB audio and Bluetooth through
`audio_output_write_pcm()`.

## I2S signals

| Signal | Function |
| --- | --- |
| BCK | Bit clock — 44100 × 16 × 2 = 1.41 MHz |
| LCK | Word select — toggles at 44.1 kHz |
| DIN | Serial audio data, 16-bit stereo |

MCLK is not used by the PCM5102A, which generates it internally. It is still routed to
GPIO8 by default, which is handy for driving another converter such as a WM8805
I2S-to-S/PDIF bridge.

## Runtime coexistence rules

**AirPlay and Bluetooth are mutually exclusive.** A Bluetooth connection suspends AirPlay;
disconnecting resumes it. While an AirPlay session is active the device stops being
discoverable over Bluetooth, so a stray phone cannot interrupt playback.

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle: Idle
    Idle: discoverable over Bluetooth
    AirPlay: AirPlay session
    AirPlay: Bluetooth discovery off
    Bluetooth: Bluetooth session
    Bluetooth: AirPlay suspended

    Idle --> AirPlay: AirPlay connects
    AirPlay --> Idle: session ends
    Idle --> Bluetooth: device pairs
    Bluetooth --> Idle: device disconnects
    AirPlay --> Bluetooth: Bluetooth connects<br/>(suspends AirPlay)
```

**Ethernet is preferred over WiFi at boot**, and both are hot-swappable at runtime. See
[Ethernet](../features/ethernet.md) for the full failover behaviour.

**Display tasks run on core 0, audio tasks on core 1** on dual-core builds. Letting the
LVGL task migrate to core 1 causes progressive audio buffer backpressure — latency climbs
without recovering until the stream misaligns.

## Source layout

```text
main/
├── main.c                      # Entry point — NVS, WiFi, AirPlay services
├── settings.c                  # NVS persistence
├── audio/
│   ├── audio_receiver.c        # RTSP session manager — buffered or realtime stream
│   ├── audio_stream.c          # Base stream abstraction
│   ├── audio_stream_buffered.c # AirPlay 2 AAC, deep jitter buffer
│   ├── audio_stream_realtime.c # AirPlay 1 ALAC, low-latency UDP
│   ├── rtp_resend.c            # Realtime retransmit requests (RTT-timed, Apple-style)
│   ├── rtp_sync.c              # Realtime sync packets → one smoothed anchor per segment
│   ├── audio_decoder.c         # ALAC and AAC decoders
│   ├── audio_decode_worker.c   # Decoder task fed from the receive path
│   ├── audio_flush_queue.c     # FLUSHBUFFERED records, keyed on 24-bit sequence numbers
│   ├── audio_buffer.c          # Decoder scratch space
│   ├── audio_timeline.c        # RTP-addressed PCM store — holds the decoded audio
│   ├── audio_plc.c             # Packet-loss concealment: fills holes with a faded repeat
│   ├── audio_engine_v2.c       # Owns the epoch, timeline, clock map and scheduler
│   ├── audio_scheduler.c       # Anchor/preroll/play/pause state machine + drift servo
│   ├── audio_clock_map.c       # Sender time ↔ RTP (PTP for AirPlay 2, NTP for AirPlay 1)
│   ├── audio_playout_clock.c   # Local → sender time per render: rate, anchor, freewheel
│   ├── audio_epoch.c           # Generation counter that invalidates stale work
│   ├── audio_timing.c          # Clock plumbing shared by the streams
│   ├── audio_latency.c         # Output latency: DMA depth + FIFO + resampler + DAC + trim
│   ├── audio_volume.c          # AirPlay dB → gain (Apple's law by default)
│   ├── audio_resample.c        # 44.1 → 48 kHz conversion
│   ├── audio_output*.c         # I2S, S/PDIF and USB backends, weak defaults, the DMA queue cursor
│   ├── audio_delay.c           # Per-output delay for multi-way speakers
│   ├── audio_eq.c              # Software EQ for DACs without a DSP
│   ├── eq_ab.c                 # Software/DSP EQ A/B switch on a TAS58xx
│   ├── usb_audio_sink.c        # USB speaker: the host plays into the I2S DAC
│   ├── audio_crypto.c          # AirPlay audio decryption
│   └── a2dp_sink.c             # Bluetooth A2DP sink
├── rtsp/
│   ├── rtsp_server.c           # Connections; one holds the session
│   ├── rtsp_conn.c             # Connection management
│   ├── rtsp_message.c          # Request parsing and response building
│   ├── rtsp_handlers.c         # Method handlers (OPTIONS, SETUP, PLAY, …)
│   ├── rtsp_anchor.c           # SETRATEANCHORTIME / FLUSHBUFFERED request validation
│   ├── rtsp_events.c           # Event channel: commands back to the sender
│   ├── rtsp_metadata.c         # DMAP, progress and artwork as one now-playing state
│   ├── rtsp_policy.c           # Platform-free rules: features, TXT, pi, preemption
│   ├── rtsp_crypto.c           # RTSP-level encryption
│   ├── rtsp_fairplay.c         # FairPlay
│   └── rtsp_rsa.c              # RSA (AirPlay 1)
├── hap/                        # HomeKit pairing — SRP, Ed25519, TLV8
├── plist/                      # Apple property list parsing
├── network/                    # WiFi, Ethernet, mDNS, PTP, NTP, web server, OTA
├── sendspin/                   # Sendspin player role (experimental)
├── usb/                        # TinyUSB composite device (UAC + HID controls)
├── dacp_client.c               # DACP remote commands (AirPlay 1, old senders)
├── playback_control.c          # Unified playback abstraction
├── playback_events.c           # Per-source playback state, fanned out
├── bt_coex.c                   # Frees Bluetooth's DRAM when AirPlay takes over
├── led.c                       # Status LED
└── buttons.c                   # Debounced button input

components/
├── dac/                        # Abstract DAC API, Kconfig-selected
├── dac_tas57xx/                # TAS5756/5754/5751 with hybrid flow DSP
├── dac_tas58xx/                # TAS5825M with on-chip DSP and 15-band EQ
├── display/                    # OLED (u8g2) and ST7789 (LVGL 9) drivers
├── boards/                     # Per-board HAL — GPIOs, SPI bus, init
├── spiffs_storage/             # SPIFFS mount
├── audio-resampler/            # Sinc-based resampler
└── board_utils/                # Board-level utilities
```

## Key components

| Module | Location | Purpose |
| --- | --- | --- |
| RTSP server | `main/rtsp/` | AirPlay control messages; one connection holds the session, others may probe and pair |
| Remote control | `main/rtsp/rtsp_events.c`, `main/dacp_client.c` | Commands back to the sender: MediaRemote on the event channel, or DACP |
| HAP pairing | `main/hap/` | Cryptographic device pairing |
| Audio pipeline | `main/audio/` | Decoding, buffering, timing |
| A2DP sink | `main/audio/` | Bluetooth audio receiver, ESP32 only |
| PTP clock | `main/network/` | Synchronisation with the source |
| WiFi | `main/network/` | AP+STA management, captive portal |
| Ethernet | `main/network/` | W5500 SPI driver |
| Web server | `main/network/` | Configuration interface |
| DAC abstraction | `components/dac/` | Kconfig-selected DAC API |
| Board support | `components/boards/` | Per-board HAL |
| Display | `components/display/` | OLED or ST7789 TFT |
| SPIFFS storage | `components/spiffs_storage/` | Filesystem mount |
| Buttons | `main/buttons.c` | Debounced button input |

## Conventions

- **Kconfig drives board selection.** The DAC driver is chosen automatically via
  `CONFIG_DAC_TAS57XX` or `CONFIG_DAC_TAS58XX`. Display, buttons, Bluetooth and Ethernet
  are all Kconfig-gated and compile out entirely when disabled.
- **Each component has its own `CMakeLists.txt`** with `idf_component_register()`.
- **`u8g2` and `u8g2-hal-esp-idf` are git submodules** — always clone with `--recursive`.

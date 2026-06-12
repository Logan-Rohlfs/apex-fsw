# Apex — Flight Computer, Simulation & Ground Tools

Apex is a student-rocketry airbrake flight computer targeting ~10,000 ft (IREC).
A Teensy 4.1 fuses IMU/baro/GPS, runs the flight state machine and airbrake PID,
logs to NAND, and downlinks 2-GFSK telemetry at 441.480 MHz over an RF4463PRO
(Si4463). The ground side is an RTL-SDR feeding a PyQt5 monitor; a tracking
ground station ("HORIZON") with a 70 cm yagi will use the same SDR pipeline.

## Repository layout

```
fsw/      PlatformIO firmware (Teensy 4.1). src/ is small — read all of it.
          envs: teensy41_debug (monitor), teensy41_hil (HIL), teensy41_flight (flight)
sim/      Python side. .venv is the project venv (Python 3.9 — no match
          statements, no list[int] in evaluated positions outside annotations).
          apex_sim/ = RocketPy HIL/SIL package, scripts/ = ground tools.
bin/      apex-setup — installs/activates the venv.
```

Key files:
- [fsw/src/config.h](fsw/src/config.h) — every tunable: pins, radio params,
  detection thresholds, PID gains, telemetry rates. **The user edits this file
  directly between your turns** (rates, power, etc.) — re-read it before
  editing, never revert values you didn't set.
- [fsw/src/radio.cpp](fsw/src/radio.cpp) — Si4463 driver + GFSK downlink TX.
- [sim/scripts/horizon.py](sim/scripts/horizon.py) — the ground GUI (serial +
  RTL-SDR sources, plots, spectrum/waterfall, link stats).
- [sim/scripts/radio_gfsk_rx.py](sim/scripts/radio_gfsk_rx.py) — SDR decoder;
  **the wire-format mirror of radio.cpp**.
- [sim/scripts/radio_diag.py](sim/scripts/radio_diag.py) — link diagnostic
  (expected vs RF-bursts-heard vs decoded; clipping/SNR/offset).

Docs to read before touching the relevant area:
- [sim/docs/sensors/rf4463pro_433_radio.md](sim/docs/sensors/rf4463pro_433_radio.md)
  — radio module, **wire format of record**, bench-validation numbers.
- [fsw/docs/devices.md](fsw/docs/devices.md) — per-device wiring + library
  snippets (IMU, baro, GPS, NAND, radio quirks).
- [sim/README.md](sim/README.md) — HIL/SIL architecture and config YAMLs.

## Build / test commands

```bash
# Firmware (always build after fsw edits — IDE clang diagnostics about
# Arduino.h/SPI1/millis are indexer noise; pio is the truth):
~/.platformio/penv/bin/pio run -e teensy41_debug -d fsw   # also: teensy41_hil, teensy41_flight

# Python: use the project venv, NOT system python:
sim/.venv/bin/python ...

# GUI smoke tests run headless:
QT_QPA_PLATFORM=offscreen sim/.venv/bin/python <test>   # filter the noisy
# "propagateSizeHints" lines from output.
```

There is no Python test suite for the radio tools; the established pattern is
**synthesize GFSK IQ end-to-end** (modulate frames with numpy exactly as the
Si4463 would — Gaussian BT=0.5, MSB-first, bit 1 = +deviation — add noise and a
realistic +10 kHz carrier offset, round-trip through u8 IQ) and assert the
decoder and monitor pipeline recover everything. Do this for any wire-format or
decoder change before claiming it works.

## The radio link (most active area)

2-GFSK, 10 kbps, ±25 kHz deviation at 441.480 MHz (125 kHz allocation; Carson
BW ≈ 60 kHz). Frame: `0xAA`×8 preamble, `0x2D 0xD4` sync, type byte, body,
CRC-16-CCITT (poly 0x1021, init 0xFFFF) over type+body, big-endian CRC.
Types: `0x01` test burst, `0x02` FLIGHT telemetry (every beat), `0x03`
HOUSEKEEPING (replaces one beat per second). Bodies are packed little-endian
structs with scaled-int16 channels; lat/lon stay f32.

Non-negotiables:
- **fsw structs and `radio_gfsk_rx.py` structs must change in the same
  commit**, with `static_assert(sizeof(...))` on the C side and
  `STRUCT.size` asserts in tests. A half-migrated flash = every frame fails
  CRC and the ground goes silent with no error (this happened; see missteps).
- **`KG5LDI`** (the user's callsign) leads every FLIGHT frame in ASCII —
  FCC Part 97 §97.119 station ID. Never remove it; 433 MHz here is the 70 cm
  amateur band.
- TX is **non-blocking** from the flight loop (skip the beat if still on air).
  Only the *TX-side* Si4463 modem properties are programmed by hand (datasheet
  formulas); a future RF4463PRO ground RX will need a WDS-generated config —
  do not try to hand-write RX modem properties.
- The CW marker (`RADIO_MARKER`) reprograms the modem; `_gfsk_ready` must be
  invalidated so GFSK reconfigures (already handled — keep it that way).

Bench facts (June 2026): link validated on hardware — 10/10 frames, CRC OK.
This bench's combined crystal offset is **+10.4 kHz ≈ +24 ppm** (stable ±40 Hz);
decode quality 0.64 is the clean-signal ceiling. A wildly different offset
means config/hardware drift, not a decoder bug.

## Monitor architecture rules

- **Never block the SDR read loop.** rtl_sdr's pipe backs up within ~100 ms
  and the spectrum freezes. Heavy work (frame decode) runs on its own thread
  against a snapshot; the FFT is cheap enough inline. Spawn-and-skip via a
  busy flag, never queue.
- Read with `stdout.read1()` and pass `-b 8192` to rtl_sdr. Plain `read(n)`
  waits for n bytes and rtl_sdr writes 256 KiB blocks — at 240 kS/s that's one
  update per ~550 ms regardless of any FPS constant.
- Decoded frames carry `sample_index`; plot points are stamped with the
  back-computed **true reception time**, not batch arrival time. Plots scroll
  the x-window against wall-clock at the 25 Hz UI tick (freeze after 5 s of
  silence), so smoothness never depends on data cadence.
- Two plot layouts share the window: `PLOT_GROUPS` (USB serial, full sensor
  set) and `RADIO_PLOT_GROUPS` (ground-station view). Routing is per-source
  via `_key_to_group_serial` / `_key_to_group_radio`.
- Line protocol from firmware and from the radio worker is identical:
  `>key:value` numeric → plots+values, `!key:value` → state panel,
  `#LEVEL: msg` → log. New state keys go through `StatePanel.update_state`.
- Theme: all colors come from the module-level palette constants
  (ACCENT/BG/SURFACE/INSET/...) and `badge_style()` / `mono_font()` helpers.
  No hand-written hex in widget code. Qt stylesheets are f-strings — escape
  literal braces as `{{ }}`.
- Performance idioms already in place (preserve them): dirty-key tracking in
  `PlotGroupWidget.refresh`, values-table batching via `flush_values()` at the
  UI tick, curve downsampling, `np.searchsorted` over boolean masks.

## User preferences & working style

- **Consult before committing to a protocol/packet design.** The user
  interrupted mid-implementation to design the telemetry packet properly —
  present the layout (struct snippet + byte counts + airtime math) and get a
  go-ahead for wire-format changes. Small fixes don't need asking.
- The user flashes and tests on real hardware quickly — leave both ends of the
  link consistent at the end of every turn, even mid-feature.
- Explain RF/DSP concepts plainly when asked (gain staging, dB, AGC) — the
  user is sharp but learning the radio domain; correct mental models matter
  to them more than just answers.
- Likes the dark cohesive look of the monitor (keep the palette), cares about
  perceived smoothness/FPS, and will notice visual regressions immediately.
- Temporary artifacts (temp dirs, pip installs used for one task, /tmp files)
  must be cleaned up; diagnostics tools should use `tempfile` + cleanup with a
  `--keep` escape hatch.
- The user edits config values (rates, FPS, waterfall depth) directly while
  testing. Treat current values in config.h / monitor constants as intentional.

## Key findings & missteps (do not repeat)

1. **Half-migrated wire format flashed to hardware** — firmware TX struct was
   updated before the Python decoder; user rebuilt for an unrelated change and
   the ground went silent (every CRC failed, no error shown). Change both ends
   atomically; if interrupted mid-migration, say so explicitly.
2. **OOK was a dead end** — bit-banged CW keying at 20 bps, brute-force
   envelope decode. Retired; don't resurrect. GFSK via the packet FIFO is the
   chip's native path and ~500× faster.
3. **Blocking the acquisition thread** — the old decoder ran in the SDR read
   loop and chewed seconds of CPU → ~3 fps spectrum. Architecture rule above.
4. **`rtl_sdr -g 0` means tuner AGC, not 0 dB.** Gain is dB (logarithmic),
   snapped to ~29 discrete hardware steps. Tune by gain-staging: raise until
   the noise floor starts rising dB-for-dB, back off one step, confirm 0% ADC
   clipping with radio_diag.
5. **Display dB is dBFS** (relative to the 8-bit ADC), not dBm.
6. **`find_frames(max_frames=32)` silently dropped ~14% of frames at 10 Hz**
   (4 s buffer holds ~40). Caps and limits must scale with rate × buffer;
   when "loss" appears, check software stages before blaming RF —
   `radio_diag.py` separates expected / bursts-heard / decoded.
7. **Batch-stamped plot points** made 10 Hz telemetry look like 2 Hz. True
   per-frame timestamps fixed it; view motion is wall-clock-driven.
8. **Si4463 hardware quirks** (hard-won, in radio.cpp comments): CTS polling
   after every command; SDO pad conditioning before SPI1 init on this board;
   power-cycle the radio rail in setup() so the chip cold-boots after firmware
   uploads; keep bench PA at 0x08 so the SDR next to it doesn't overload;
   FRAC/2^19 ∈ [1,2) in the PLL formula.
9. **Decoder design that works**: quadrature discriminator → boxcar bit filter
   (cumsum) → ±1 template correlation on the DC-balanced preamble+sync (the
   template mean doubles as the slicer threshold = free carrier-offset
   estimate) → CRC as the final arbiter. No AFC needed at ±25 kHz deviation.
10. **Python 3.9 venv** — `struct.Struct` mirrors, no 3.10+ syntax in
    evaluated positions. `from __future__ import annotations` is present where
    needed; keep it.

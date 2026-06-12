# Apex Binary Flight Logging

This document records the logging architecture decisions that are safety- and
project-critical. A rocket can physically fly without storage, but Apex cannot
validate an airbrake flight without a trustworthy log. For this project,
logging failure is therefore launch-fatal.

## Fatal Decisions

- **No log, no arm.** `flight_state_arm()` refuses to enter `ARMED` unless both
  QSPI flash and microSD are mounted, writable, and the boot log file is open.
- **Do not let logging stop control after launch.** Once `BOOST` is detected,
  write failures are recorded as fault bits, but the state machine and airbrake
  controller keep running.
- **HIL is a flight to the firmware.** HIL uses the same arming and logging path
  as flight. A HIL run should produce a normal binary flight log from the board.
- **QSPI and SD are both required for launch.** QSPI is the soldered black-box
  log. SD is a removable mirror and later offload target. Missing either medium
  blocks arming in this phase.
- **Binary on board, one wide CSV off board.** The board writes compact typed
  binary records. The ground tool will decode those records into one
  Excel-friendly CSV per flight.
- **Flight numbering is in the records, not filenames.** Each boot gets a
  `boot_id`. Each launch/HIL launch detection gets a `flight_id`. The decoder
  splits logs by `flight_id`, so multiple restarts do not require erasing memory.

## Current Firmware Slice

At boot, storage creates one binary file per boot on both media:

```text
APEX/BOOT_00001.APXLOG
APEX/BOOT_00002.APXLOG
...
```

The files contain typed records with this header:

```cpp
struct LogHeader {
    uint32_t magic;      // "APXL" little-endian
    uint8_t  version;    // APEX_LOG_VERSION
    uint8_t  type;       // BOOT, EVENT, SAMPLE
    uint16_t length;     // payload bytes
    uint32_t seq;        // monotonic per boot
    uint32_t boot_id;    // increments every boot
    uint32_t flight_id;  // 0 before launch, assigned at BOOST
    uint32_t time_ms;    // millis() at record creation
    uint16_t crc;        // header-with-zero-crc + payload
};
```

This header makes the log scan-resynchronizable: if power dies or a write is
partially corrupt, the decoder can search for the next magic word and continue.

## Data Flow

```text
sensor/fusion/control state
          |
          v
   storage_log_update()
          |
          +--> prelaunch RAM ring @ LOG_PRELAUNCH_RING_HZ
          |
          +--> binary SAMPLE records to QSPI + SD

BOOST detected
          |
          v
 storage_begin_flight()
          |
          +--> assign flight_id
          +--> write LAUNCH_DETECTED event
          +--> flush prelaunch ring into the flight log
```

## Initial Rates

```text
Pad file log:       2 Hz
Prelaunch RAM ring: 50 Hz for LOG_RING_BUF_SECONDS
BOOST / COAST:      100 Hz
DESCENT:            25 Hz
LANDED / IDLE:      2 Hz
```

100 Hz is the first flight rate because it matches the state/control loop and
HIL packet loop. A later benchmark may add a narrower 200 Hz IMU/fusion record,
but the first useful log should not record faster than the main flight decisions
are being made.

## Decoder Target

The ground tool exports one folder per flight and one wide CSV:

```text
Flight_02_2026-06-12T18-44-03Z/
    raw_BOOT_00042.APXLOG
    Flight_02_2026-06-12T18-44-03Z.csv
    metadata.json
```

Example CSV shape:

```csv
time_ms,boot_id,flight_id,phase,event,alt_m,vel_mps,pred_apogee_m,ax,ay,az,gx,gy,gz,baro_pa,gps_fix,gps_sats,deploy,pid_p,pid_i,pid_d,storage_faults
10020,42,0,ARMED,,0.3,0.0,0.3,9.81,0.02,-0.01,0.001,0.000,0.003,91343.0,3,12,0.000,0.0,0.0,0.0,0
10170,42,2,BOOST,LAUNCH_DETECTED,2.1,8.7,4180.2,71.4,0.8,-0.4,0.010,0.001,0.004,91315.8,3,12,0.000,0.0,0.0,0.0,0
17230,42,2,COAST,,1320.8,238.4,3260.5,-4.3,0.1,0.0,0.002,0.000,0.002,74231.9,3,13,0.083,278.9,-1.2,-31.3,0
```

The current CLI lives in `sim/scripts/decode_logs.py`:

```bash
sim/.venv/bin/python sim/scripts/decode_logs.py /Volumes/APEX-FLASH/APEX --out sim/output/log_exports
sim/.venv/bin/python sim/scripts/decode_logs.py /Volumes/APEX-SD/APEX --out sim/output/log_exports
```

It accepts individual `.APXLOG` files or directories, scans/resynchronizes by
record magic, verifies CRCs, splits by `flight_id`, copies the raw source log
into the export folder, and writes `metadata.json` alongside the single wide
CSV. Ground/no-launch records can be suppressed with `--no-ground`.

## Open Follow-Ups

- Benchmark QSPI and SD write latency on the actual board with the actual SD
  card. If SD latency spikes are too large, keep QSPI real-time and mirror to SD
  after landing.
- Add a buzzer pin and alarm patterns. Logging-not-ready should get a loud pad
  alarm and continue refusing arm.
- Integrate offload/export into the ground app so launch-day use is one click.
- Add explicit firmware git/build hash records in addition to the current
  config hash and payload-size metadata.
- Add a storage write-latency benchmark command/log record for QSPI vs SD.

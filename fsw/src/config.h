#pragma once

// ─── Pins ─────────────────────────────────────────────────────────────────────

// Barometer / high-g accel (I2C0)
#define PIN_BAR_INT1        7
#define PIN_ACC2_INT1       8

// IMU (SPI0)
#define PIN_ACC1_INT1       9
#define PIN_ACC1_CS         10
#define PIN_MOSI0           11
#define PIN_MISO0           12
#define PIN_SCK0            13

// Magnetometer (I2C1)
#define PIN_SDA1            17
#define PIN_SCL1            16

// GPS — MAX-M10S (I2C2 primary, UART7 available)
// Wire2: SDA2=25, SCL2=24 (Teensy 4.1 defaults)
// Serial7: TX7=28, RX7=29
#define PIN_GPS_PPS         30   // 1 Hz pulse — attach interrupt for time sync

// GPS configuration. GPS is useful for UTC, recovery, telemetry, and post-flight
// reconstruction, but it is intentionally not required for control or flight
// state progression; high-G boost can still cause temporary fix loss.
#define GPS_I2C_CLOCK_HZ    400000UL
#define GPS_NAV_RATE_HZ     10
#define GPS_DYNAMIC_MODEL   DYN_MODEL_AIRBORNE4g

// GPS trust model. AIRBORNE4g tracks up to 4 g of dynamics — boost is ~14 g,
// so a fix loss at launch is EXPECTED (logged as such, not a fault). After a
// loss the first epochs can be garbage while the tracking loops re-converge:
// require N consecutive fresh fixes before gps_trusted() returns true again.
// A solution older than GPS_STALE_MS clears gps.valid — a frozen module must
// not present stale coordinates as live.
#define GPS_STALE_MS              1500     // > 10 Hz nominal + I2C hiccups
#define GPS_REACQUIRE_EPOCHS      5        // consecutive fresh fixes to re-trust
#define GPS_ALT_SANITY_M          300.0f   // |gps AGL − fused AGL| divergence warn

// Radio — RF4463PRO / Si4463 (SPI1)
// SPI1: MOSI1=26, MISO1=1, SCK1=27
#define PIN_RAD_MOSI        26
#define PIN_RAD_MISO        1
#define PIN_RAD_SCK         27
#define PIN_RAD_CS          0
#define PIN_RAD_INT1        2    // RF4463PRO nIRQ
#define PIN_RAD_GPIO0       5    // Si4463 GPIO0
#define PIN_RAD_GPIO1       4    // Si4463 GPIO1

// Power enables
#define PIN_3V3_2_EN        23   // AP2112K-3.3 enable — drives radio 3.3V rail
#define PIN_12V_EN          20   // A6  — 12 V rail enable, powers an external
                                 //       video TX. Follows PIN_RADIO_SWITCH.
#define PIN_SRV_EN          38   // A14 — servo power high-side switch (NPN+PMOS),
                                 //       on at ARM / off in IDLE (no pad chatter)

// ── Switches (screw switches on the rod; see docs/flight_readiness.md) ────────
// Two independent purely-software switches — simple bridges to GND, read via
// INPUT_PULLUP (closed = GND = LOW, open/broken floats HIGH = safe default).
// PIN_ARM_SWITCH:   closed = arming interlock satisfied (one of several AND'd
//                   conditions in flight_state_arm()/IDLE auto-arm — closing
//                   it does not arm by itself).
// PIN_RADIO_SWITCH: closed = radio transmissions enabled. While open the FC is
//                   radio-silent: firmware withholds onboard Si4463 TX and
//                   drives PIN_12V_EN low (external video TX off).
#define PIN_ARM_SWITCH      32   // D32
#define PIN_RADIO_SWITCH    31   // D31
#define SWITCH_CLOSED_LEVEL LOW  // pin level when a switch is closed (to GND).

// ── Buzzer (on-pad status; the only feedback to an operator with no uplink) ───
#define PIN_BUZZER          6    // D6 — passive piezo driven with tone()
#define BUZZER_ACTIVE       0    // 1 = self-driving (active) buzzer → digitalWrite

// ─── Radio ────────────────────────────────────────────────────────────────────
// RF4463PRO-433 / Si4463 crystal frequency used by POWER_UP and PLL math.
// NiceRF material confirms a 10 ppm crystal but is inconsistent/sparse on the
// frequency; 30 MHz matches Silicon Labs reference configs and the SDR marker.
// GPIO2 = RXEN, GPIO3 = TXEN (antenna TX/RX switch internal to module).
// RF4463PRO SDN is active-high shutdown; hardware must hold it low for SPI.
#define RADIO_XTAL_HZ       30000000UL
#define RADIO_FREQ_HZ       441480000UL  // allocated center frequency: 441.480 MHz
#define RADIO_CHANNEL_BW_HZ 125000UL     // allocated channel bandwidth: 125 kHz
#define RADIO_MARKER_PA_PWR 0x7F         // low bench marker power to avoid SDR overload

// 2-GFSK downlink parameters. Carson bandwidth = 2*(dev + bitrate/2) = 60 kHz,
// inside the 125 kHz allocation. Deviation is large vs. the ±4.4 kHz worst-case
// crystal offset (10 ppm at 441 MHz) so the SDR can slice without AFC.
#define RADIO_GFSK_BITRATE_BPS  10000UL
#define RADIO_GFSK_DEV_HZ       25000UL

// FCC Part 97 station identification (§97.119): the callsign is embedded in
// ASCII in every telemetry frame, satisfying the 10-minute ID requirement.
#define RADIO_CALLSIGN          "KG5LDI"

// Telemetry beacon rates by phase. FLIGHT frame = 51 B / ~41 ms airtime at
// 10 kbps; one beat per second is replaced by the HOUSEKEEPING frame (33 B).
// At 20 Hz that's ~82% airtime duty.
#define RADIO_TELEM_IDLE_HZ     20   // IDLE / LANDED
#define RADIO_TELEM_FLIGHT_HZ   20   // ARMED / BOOST / COAST / DESCENT

// ─── I2C Addresses ────────────────────────────────────────────────────────────
#define BMP581_ADDR         0x46    // ADR tied to GND
#define ADXL375_ADDR        0x53    // SDO tied to GND

// ─── Sensor ODR / FSR ─────────────────────────────────────────────────────────
#define IMU_ACCEL_ODR_HZ    800
#define IMU_ACCEL_FSR_G     16
#define IMU_GYRO_ODR_HZ     800
#define IMU_GYRO_FSR_DPS    2000
#define HIGHG_ODR_HZ        800

// ─── Task Rates ───────────────────────────────────────────────────────────────
#define RATE_FUSION_HZ      200
#define RATE_STATE_HZ       100
#define RATE_CONTROL_HZ     100
#define RATE_BARO_HZ        50
#define RATE_MAG_HZ         25

// ─── Arming ───────────────────────────────────────────────────────────────────
// IDLE → ARMED once the pad reference is captured, the boot settle has elapsed,
// AND the arm switch is closed (airbrakes are not pyro — ARMED only enables
// launch detection). Switch-level gated; debug builds can also use ARM/DISARM.
#define AUTO_ARM_DELAY_MS           10000

// ─── Launch Detection ─────────────────────────────────────────────────────────
// Sustained accel starts a launch candidate and powers the servo rail. A
// sustained 30 m barometric rise validates flight before burnout/airbrake
// control may proceed. Baro can also detect and validate launch by itself.
#define LAUNCH_ACCEL_THRESH_MSS     19.62f   // 2g
#define LAUNCH_CONFIRM_MS           150
#define LAUNCH_BARO_BACKUP_M        30.0f    // baro AGL backup gate
#define LAUNCH_BARO_CONFIRM_MS      200
#define LAUNCH_ABORT_CONFIRM_MS     500      // settled handling bump -> ARMED

// ─── Burnout Detection ────────────────────────────────────────────────────────
// Primary: axial specific force flips negative (−0.78 g early coast on the
// Seymour TX recording). Backup: max burn time — N3355 burns ~4.6 s, a stuck
// gate cannot hold BOOST forever.
#define BURNOUT_CONFIRM_MS          200
#define BOOST_MAX_MS                8000

// ─── Airbrake Gates ───────────────────────────────────────────────────────────
#define POST_BURNOUT_LOCKOUT_MS     1000
#define MACH_GATE_MPS               260.0f   // ~0.75 Mach — no transonic actuation
#define MIN_DEPLOY_ALT_M            100.0f

// ─── Apogee Detection ─────────────────────────────────────────────────────────
// Primary: fused (accel-verified) velocity through zero — immune to transonic
// baro spikes, same approach as TeleMetrum. Backup: baro fell below the
// running max; armed only well after burnout so the transonic regime (first
// ~4 s of coast at this rocket's ~M0.85) can never reach it.
#define APOGEE_VEL_THRESH_MPS       2.0f     // velocity below this = apogee
#define APOGEE_CONFIRM_MS           500
#define APOGEE_BACKUP_LOCKOUT_MS    8000     // baro backup armed this long after burnout
#define APOGEE_BARO_FALL_M          10.0f

// ─── Landed Detection ─────────────────────────────────────────────────────────
// Orientation-independent: |baro rate| (1 s baseline) + accel magnitude near
// 1 g. A rocket on its side breaks any axial-axis assumption — the CF
// velocity never settles to zero there (validated on Seymour TX landing).
#define LANDED_ACCEL_THRESH_MSS     2.0f     // ||accel| − 1 g| band
#define LANDED_VEL_MAX_MPS          2.0f     // baro-rate threshold
#define LANDED_CONFIRM_MS           3000

// ─── Control Law ──────────────────────────────────────────────────────────────
#define TARGET_APOGEE_M             3048.0f  // 10,000 ft AGL

// PID gains from MATLAB, rescaled ft → m (divide by 0.3048).
// All three gains rescale — the D-term is Kd × velocity and velocity has
// units too (ft/s in MATLAB, m/s here). Matches apex_sim/sim/airbrakes.py.
// D-term uses velocity directly as proxy — matches sim, do not change.
#define PID_KP                      (0.6f   / 0.3048f)
#define PID_KI                      (-0.004f / 0.3048f)
#define PID_KD                      (-0.05f / 0.3048f)
#define PID_U_MIN                   -15.0f
#define PID_U_MAX                    30.0f

// ─── Servo ────────────────────────────────────────────────────────────────────
// Direction is INVERTED: deployment 0 → SERVO_MAX_US (retracted),
// deployment 1 → SERVO_MIN_US (fully deployed). Endpoints measured from the
// airframe. Full stroke in SERVO_FULL_TRAVEL_S (rate-limited in control.cpp).
#define SERVO_PIN                   37       // D37
#define SERVO_MIN_US                500      // fully deployed
#define SERVO_MAX_US                1833     // retracted
#define SERVO_MIN_DEG               0.0f
#define SERVO_MAX_DEG               180.0f
#define SERVO_FULL_TRAVEL_S         0.24f    // 0% → 100% travel time
#define SERVO_MAX_RATE_DEG_PER_S    (180.0f / SERVO_FULL_TRAVEL_S)
#define SERVO_BOOT_TEST_FRAC        0.05f    // restrained rail-safe wiggle
#define SERVO_BOOT_TEST_HOLD_MS     250

// ─── Physical Constants ───────────────────────────────────────────────────────
#define ROCKET_MASS_KG              30.44f
#define REF_AREA_M2                 0.019001f
#define CD_CLEAN                    0.576f
#define ISA_SEA_LEVEL_PA            101325.0f
#define ISA_SEA_LEVEL_TEMP_K        288.15f
#define ISA_LAPSE_RATE              0.0065f

// ─── Sensor Fusion ────────────────────────────────────────────────────────────
// Mahony convergence guard — integrator is blocked until attitude is settled.
// Requires FUSION_CONV_MIN_SAMPLES elapsed AND FUSION_CONV_CONFIRM_COUNT
// consecutive samples with |vert_accel| < FUSION_CONV_VERT_ACCEL_MAX.
#define FUSION_CONV_MIN_SAMPLES      300    // 1.5 s at 200 Hz
#define FUSION_CONV_CONFIRM_COUNT    100    // 0.5 s of stable residual
#define FUSION_CONV_VERT_ACCEL_MAX   0.3f  // m/s² residual threshold

// Complementary filter gains.
// alpha: altitude baro weight per tick (dimensionless).
// beta:  velocity correction gain (1/s). Kept separate from alpha/dt to
//        prevent the dt-cancellation that made the old gain 1.0–4.0×.
// Validated against Seymour TX flight data (max coast baro spike: 101 Pa = 10.9 m).
// CF_COAST_BETA=1.0 → 10.9 m spike injects 10.9 m/s; acceptable because the
// clamp limits alt_err to CF_ALT_ERR_CLAMP_M before it reaches the gain.
#define CF_BOOST_ALPHA               0.005f
#define CF_COAST_ALPHA               0.02f
#define CF_BOOST_BETA                0.10f  // mild — baro noisy under motor vibration
#define CF_COAST_BETA                1.00f  // aggressive — baro reliable post-burnout
#define CF_ALT_ERR_CLAMP_M           5.0f  // max baro correction before clamping spike

// Baro-dead GPS altitude fallback. Engages only when the baro has produced
// no sample for BARO_DEAD_MS AND gps_trusted() — keeps the CF bounded instead
// of dead-reckoning on accel alone. Gains are weak: GPS altitude is 10 Hz,
// σ ≈ 3 m, with >100 ms solution latency. Never used while baro is alive.
#define BARO_DEAD_MS                 500
#define CF_GPS_ALPHA                 0.01f
#define CF_GPS_BETA                  0.20f
#define CF_GPS_ERR_CLAMP_M           20.0f

// ─── Pad Re-Zero ──────────────────────────────────────────────────────────────
// Periodically refreshes the ground pressure reference while stationary on pad.
// Only fires in IDLE or ARMED. Motion check must pass for REZERO_STABLE_MS
// before a re-zero is allowed. Safe at launch edge — control loop activates
// 2.5s+ post-burnout, well after the complementary filter reconverges.

#define REZERO_INTERVAL_MS          30000   // attempt re-zero every 30s
#define REZERO_STABLE_MS            10000   // must be stable for 10s before re-zero
#define REZERO_ACCEL_MIN_MSS        9.3f    // 0.95g — lower bound of "stationary"
#define REZERO_ACCEL_MAX_MSS        10.3f   // 1.05g — upper bound
#define REZERO_GYRO_MAX_RADS        0.05f   // not rotating
#define REZERO_BARO_SAMPLES         50      // rolling average depth (~1s at 50Hz)

// ─── Logging ──────────────────────────────────────────────────────────────────
#define LOG_RATE_DESCENT_HZ         25
#define LOG_RING_BUF_SECONDS        15      // high-rate pre-launch context depth

// Binary flight logger. QSPI flash is launch-critical; microSD is a redundant
// mirror. Runtime writes are sampled in the main loop (never sensor ISRs). The
// RAM ring keeps compact recent prelaunch samples so the log can explain launch
// detection without allowing old pad context to compete with flight data.
#define LOG_PRELAUNCH_RING_HZ       20
#define LOG_PAD_FILE_HZ             2
#define LOG_FLIGHT_FILE_HZ          100     // fastest useful state/control rate
// QSPI logging uses PJRC's supported LittleFS_QPINAND file path. The storage
// layer still batches records into NAND-page-sized writes so the firmware does
// not feed LittleFS a stream of tiny appends.
// HIL-only escape hatch. Flight/debug builds still enforce "no log, no arm";
// HIL can continue so sim/control tests are not blocked by storage bring-up.
#define HIL_ALLOW_STORAGE_FAULT_ARM 1
// QSPI NAND writes are staged into page-sized chunks before LittleFS writes.
// This bounds the power-loss window for records that have not filled a page yet.
#define LOG_FLASH_PAGE_BUFFER_BYTES 2048
#define LOG_FLASH_FLUSH_INTERVAL_MS 5000
#define LOG_FLASH_MAX_FAULTS        3        // suspend QSPI after this many consecutive failures
// SD is only a mirror, and its FAT sync can stall 100s of ms (occasionally
// >1 s) — flushing it every second was freezing the loop. Flush it rarely; the
// post-landing QSPI→SD dump still commits the full log.
#define LOG_SD_FLUSH_INTERVAL_MS    30000
#define LOG_SD_MAX_FAULTS           1        // suspend live SD mirror after first fault
#define LOG_SD_SLOW_DISABLE_US      250000   // or after one very slow write/flush

// Delay the one-shot post-landing QSPI->SD dump so a couple more seconds of
// settling/impact high-g samples land in the QSPI log before it's copied.
// The dump is a single blocking pass (storage.cpp dump_qspi_file_to_sd) —
// fine since the FC isn't used for recovery.
#define LOG_LANDING_DUMP_DELAY_MS   3000

// Debug-only storage diagnostics. The flight logger still runs normally; debug
// builds only add a low-rate timing summary so SD/QSPI stalls are visible.
#define DEBUG_STORAGE_SUMMARY_MS    1000
#define DEBUG_STORAGE_SLOW_US       5000

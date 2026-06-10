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

// ─── Radio ────────────────────────────────────────────────────────────────────
// RF4463PRO-433 / Si4463 crystal frequency used by POWER_UP and PLL math.
// NiceRF material confirms a 10 ppm crystal but is inconsistent/sparse on the
// frequency; 30 MHz matches Silicon Labs reference configs and the SDR marker.
// GPIO2 = RXEN, GPIO3 = TXEN (antenna TX/RX switch internal to module).
// RF4463PRO SDN is active-high shutdown; hardware must hold it low for SPI.
#define RADIO_XTAL_HZ       30000000UL
#define RADIO_FREQ_HZ       441480000UL  // allocated center frequency: 441.480 MHz
#define RADIO_CHANNEL_BW_HZ 125000UL     // allocated channel bandwidth: 125 kHz
#define RADIO_MARKER_PA_PWR 0x08         // low bench marker power to avoid SDR overload

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
#define RADIO_TELEM_IDLE_HZ     30   // IDLE / LANDED
#define RADIO_TELEM_FLIGHT_HZ   10   // ARMED / BOOST / COAST / DESCENT

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

// ─── Launch Detection ─────────────────────────────────────────────────────────
#define LAUNCH_ACCEL_THRESH_MSS     19.62f   // 2g
#define LAUNCH_CONFIRM_MS           150

// ─── Burnout Detection ────────────────────────────────────────────────────────
#define BURNOUT_CONFIRM_MS          200

// ─── Airbrake Gates ───────────────────────────────────────────────────────────
#define POST_BURNOUT_LOCKOUT_MS     2500
#define MACH_GATE_MPS               240.0f   // 0.7 Mach
#define MIN_DEPLOY_ALT_M            100.0f

// ─── Apogee Detection ─────────────────────────────────────────────────────────
#define APOGEE_VEL_THRESH_MPS       2.0f     // velocity below this = apogee
#define APOGEE_CONFIRM_MS           500

// ─── Landed Detection ─────────────────────────────────────────────────────────
#define LANDED_ACCEL_THRESH_MSS     2.0f     // low accel variance
#define LANDED_CONFIRM_MS           3000

// ─── Control Law ──────────────────────────────────────────────────────────────
#define TARGET_APOGEE_M             3048.0f  // 10,000 ft AGL

// PID gains from MATLAB, rescaled ft → m (divide by 0.3048)
// D-term uses velocity directly as proxy — matches sim, do not change.
#define PID_KP                      (0.4f   / 0.3048f)
#define PID_KI                      (-0.004f / 0.3048f)
#define PID_KD                      -0.04f
#define PID_U_MIN                   -15.0f
#define PID_U_MAX                    30.0f

// ─── Servo ────────────────────────────────────────────────────────────────────
#define SERVO_PIN                   6
#define SERVO_MIN_US                1000     // parameterized — measure from hardware
#define SERVO_MAX_US                2000
#define SERVO_MIN_DEG               0.0f
#define SERVO_MAX_DEG               180.0f
#define SERVO_MAX_RATE_DEG_PER_S    (180.0f / 0.24f)   // 0.24s full travel

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
#define LOG_RATE_IDLE_HZ            1
#define LOG_RATE_ARMED_HZ           25
#define LOG_RATE_BOOST_HZ           200
#define LOG_RATE_COAST_HZ           100
#define LOG_RATE_DESCENT_HZ         25
#define LOG_RING_BUF_SECONDS        60      // pre-launch ring buffer depth

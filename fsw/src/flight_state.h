#pragma once
#include <Arduino.h>

// ─── Flight Phase ─────────────────────────────────────────────────────────────

enum class FlightPhase : uint8_t {
    IDLE     = 0,
    ARMED    = 1,
    BOOST    = 2,
    COAST    = 3,
    DESCENT  = 4,
    LANDED   = 5,
};

const char* phase_name(FlightPhase p);

// ─── State machine (flight_state.cpp) ─────────────────────────────────────────
// Call at RATE_STATE_HZ from the main loop (both flight and HIL builds).
// Evaluates phase transitions on g_state and advances g_state.phase.
// Each transition has a primary gate with a confirmation window plus an
// independent backup gate — see config.h "Detection" sections.
void flight_state_update(uint32_t now_ms);

// Force IDLE → ARMED (fresh pad reference) / ARMED → IDLE. Used by the HIL
// auto-arm path and the debug-mode ARM/DISARM commands. flight_state_arm()
// resets all detection windows and calls fusion_on_armed(). Returns false if
// launch-critical preflight requirements (currently storage logging) are not met.
bool flight_state_arm(uint32_t now_ms);
void flight_state_disarm();

// Boot-like full reset back to IDLE: detection windows, max altitude, baro
// rate ring, the once-per-boot auto-arm latch, the pad reference (fusion is
// re-initialised so auto pad capture runs again) and the control state.
// Only called at the HIL session boundary — never in flight builds' loop.
void flight_state_reset(uint32_t now_ms);

// ─── Raw Sensor Data ──────────────────────────────────────────────────────────
// Written by sensor layer timer callbacks.
// Read by fusion layer using sensors_get_*() which performs an atomic copy.
// All values in SI units.

struct ImuData {
    float    accel_x_mss;
    float    accel_y_mss;
    float    accel_z_mss;
    float    gyro_x_rads;
    float    gyro_y_rads;
    float    gyro_z_rads;
    uint32_t timestamp_ms;
};

struct HighGData {
    float    accel_x_mss;
    float    accel_y_mss;
    float    accel_z_mss;
    uint32_t timestamp_ms;
};

struct BaroData {
    float    pressure_pa;
    float    temperature_c;
    uint32_t timestamp_ms;
};

struct MagData {
    float    x_gauss;
    float    y_gauss;
    float    z_gauss;
    uint32_t timestamp_ms;
};

struct GpsData {
    // Position / velocity
    float    lat_deg;
    float    lon_deg;
    float    altitude_msl_m;
    float    speed_mps;
    uint8_t  fix_quality;     // 0=no fix, 2=2D, 3=3D, 4=GNSS+DR
    uint8_t  satellites;
    bool     valid;

    // UTC time — only valid when time_valid == true
    bool     time_valid;
    uint16_t utc_year;
    uint8_t  utc_month;
    uint8_t  utc_day;
    uint8_t  utc_hour;
    uint8_t  utc_minute;
    uint8_t  utc_second;
    uint16_t utc_ms;

    // micros() timestamp of last PPS rising edge — use to anchor UTC to system clock
    volatile uint32_t pps_micros;

    uint32_t timestamp_ms;
};

// ─── Fused State ──────────────────────────────────────────────────────────────
// Written by fusion layer at RATE_FUSION_HZ.

struct FusedState {
    float    altitude_agl_m;
    float    velocity_mps;        // vertical, upward positive
    float    accel_mps2;          // vertical
    float    predicted_apogee_m;
    float    attitude_q[4];       // quaternion [w, x, y, z]
    float    air_density_kgm3;    // ISA model from altitude
    uint32_t timestamp_ms;
};

// ─── Control State ────────────────────────────────────────────────────────────
// Written by control loop at RATE_CONTROL_HZ.

struct ControlState {
    float    deployment_frac;     // 0.0–1.0
    float    servo_angle_deg;     // 0–180
    float    pid_error_m;
    float    pid_integral;
    float    pid_p_term;
    float    pid_i_term;
    float    pid_d_term;
    bool     active;              // true when airbrake control loop is running
    uint32_t timestamp_ms;
};

// ─── Flight State (shared bus) ────────────────────────────────────────────────
// Single global. ISR-written fields are accessed via sensors_get_*() atomic
// copies. Multi-byte reads from fusion/control fields in the main loop should
// use a brief noInterrupts() / interrupts() section.

struct FlightState {
    // Latest raw sensor values — written by fusion layer after each read.
    // Safe to read from the main loop for logging/plotting without competing
    // with the sensor staging buffers.
    ImuData   imu;
    HighGData high_g;
    BaroData  baro;
    MagData   mag;

    // Fused estimates — written by fusion layer
    FusedState fused;

    // State machine
    FlightPhase phase;
    uint32_t    phase_entry_ms;
    uint32_t    burnout_time_ms;    // recorded when burnout detected
    bool        airbrakes_enabled;  // true only after 30 m baro launch validation

    // Control output
    ControlState control;

    // GPS (async, not in critical path)
    GpsData gps;

    // Pad altitude reference — set on arm, used for AGL calculation
    float pad_pressure_pa;
    float pad_altitude_msl_m;
};

extern FlightState g_state;

#include "flight_state.h"
#include "config.h"
#include "control.h"
#include "debug.h"
#include "fusion.h"
#include "storage.h"

#include <Arduino.h>
#include <math.h>

FlightState g_state = {};

const char* phase_name(FlightPhase p) {
    switch (p) {
        case FlightPhase::IDLE:    return "IDLE";
        case FlightPhase::ARMED:   return "ARMED";
        case FlightPhase::BOOST:   return "BOOST";
        case FlightPhase::COAST:   return "COAST";
        case FlightPhase::DESCENT: return "DESCENT";
        case FlightPhase::LANDED:  return "LANDED";
        default:                   return "UNKNOWN";
    }
}

// ─── State machine ────────────────────────────────────────────────────────────
// Reference implementation (validated against the Seymour TX 2026-05-24
// recording end-to-end): sim/apex_sim/hil/fake_teensy.py::FlightLogic.
// Keep the two in lockstep — the Python replay tests are the regression
// suite for this logic (sim/tests/test_hil_flight_replay.py).

// Confirmation-window start timestamps. 0 = window not running.
// A single sample that fails the condition resets the window — a spike can
// never advance a phase.
static uint32_t _launch_since      = 0;
static uint32_t _launch_baro_since = 0;
static uint32_t _burnout_since     = 0;
static uint32_t _apogee_since      = 0;
static uint32_t _apogee_baro_since = 0;
static uint32_t _landed_since      = 0;

static float _max_alt_agl = 0.0f;

// Baro vertical rate over a 1 s baseline (10 slots × 100 ms) for the LANDED
// gate. A per-tick derivative spikes on quantised/steppy baro data; the 1 s
// difference is robust (validated on the TeleMega landing recording).
static float    _alt_ring[10] = {0};
static uint8_t  _ring_idx     = 0;
static bool     _ring_full    = false;
static uint32_t _ring_last_ms = 0;
static float    _baro_rate    = 0.0f;

// IDLE auto-arm latch — fires once per boot so a debug-mode DISARM sticks.
// File scope (not function-static) so flight_state_reset() can re-allow it
// at a HIL session boundary, making each session boot-like.
static bool     _auto_armed           = false;
static uint32_t _last_auto_arm_try_ms = 0;

static float pressure_to_alt_msl(float pa) {
    return 44330.0f * (1.0f - powf(pa / ISA_SEA_LEVEL_PA, 1.0f / 5.255f));
}

// Sustained-condition window. Returns true once `cond` has held for
// `window_ms`; resets the window whenever `cond` drops.
static bool confirm(uint32_t now_ms, uint32_t& since_ms, bool cond,
                    uint32_t window_ms) {
    if (!cond) { since_ms = 0; return false; }
    if (since_ms == 0) { since_ms = now_ms; return false; }
    return (now_ms - since_ms) >= window_ms;
}

static void reset_windows() {
    _launch_since = _launch_baro_since = _burnout_since = 0;
    _apogee_since = _apogee_baro_since = _landed_since = 0;
    _max_alt_agl  = 0.0f;
    _ring_idx = 0; _ring_full = false; _ring_last_ms = 0;
    _baro_rate = 0.0f;
}

static void enter(FlightPhase p, uint32_t now_ms, const char* how) {
    g_state.phase          = p;
    g_state.phase_entry_ms = now_ms;
    if (p == FlightPhase::BOOST) {
        storage_begin_flight(now_ms, how);
    } else {
        storage_log_event(LOG_EVENT_PHASE, how);
    }
    LOG_INFO("%s detected (%s)", phase_name(p), how);
    (void)how;
}

bool flight_state_arm(uint32_t now_ms) {
    if (!storage_logging_ready()) {
        storage_log_event(LOG_EVENT_STORAGE_FAULT, "arm refused: logging not ready");
        LOG_ERROR("ARM refused: logging storage not ready");
        return false;
    }
    reset_windows();
    fusion_on_armed();
    control_reset();   // arming = fresh flight: no stale PID integral/deploy
    g_state.phase          = FlightPhase::ARMED;
    g_state.phase_entry_ms = now_ms;
    storage_log_event(LOG_EVENT_ARMED, "armed");
    return true;
}

void flight_state_reset(uint32_t now_ms) {
    // Boot-like reset. Only invoked from the HIL session-gap handler in
    // main.cpp (never during flight): each HIL session must behave exactly
    // like a fresh power-on flight.
    reset_windows();
    _auto_armed           = false;
    _last_auto_arm_try_ms = 0;
    g_state.phase           = FlightPhase::IDLE;
    g_state.phase_entry_ms  = now_ms;
    g_state.burnout_time_ms = 0;
    // Drop the pad reference so fusion's auto pad capture runs again from a
    // fresh 50-sample baro average (fusion_init() preserves a non-zero pad
    // reference for watchdog warm restarts — clear it first).
    g_state.pad_pressure_pa    = 0.0f;
    g_state.pad_altitude_msl_m = 0.0f;
    fusion_init();
    control_reset();
    LOG_INFO("Flight state reset to IDLE");
}

void flight_state_disarm() {
    if (g_state.phase == FlightPhase::ARMED) {
        g_state.phase = FlightPhase::IDLE;
        reset_windows();
        storage_log_event(LOG_EVENT_DISARMED, "disarmed");
    }
}

void flight_state_update(uint32_t now_ms) {
    // Snapshot ISR-written fields (200 Hz fusion timer in flight builds).
    noInterrupts();
    const float ax      = g_state.imu.accel_x_mss;
    const float ay      = g_state.imu.accel_y_mss;
    const float az      = g_state.imu.accel_z_mss;
    const float baro_pa = g_state.baro.pressure_pa;
    const float vel     = g_state.fused.velocity_mps;
    interrupts();

    const bool  pad_ready = (g_state.pad_pressure_pa > 0.0f);
    const float baro_agl  = pad_ready && baro_pa > 50000.0f
        ? pressure_to_alt_msl(baro_pa) - g_state.pad_altitude_msl_m
        : 0.0f;
    const float accel_mag = sqrtf(ax * ax + ay * ay + az * az);

    if (baro_agl > _max_alt_agl) _max_alt_agl = baro_agl;

    // 1 s-baseline baro rate (orientation-independent LANDED signal).
    if (_ring_last_ms == 0 || now_ms - _ring_last_ms >= 100) {
        _ring_last_ms = now_ms;
        if (_ring_full) _baro_rate = baro_agl - _alt_ring[_ring_idx];  // /1.0 s
        _alt_ring[_ring_idx] = baro_agl;
        _ring_idx = (_ring_idx + 1) % 10;
        if (_ring_idx == 0) _ring_full = true;
    }

    switch (g_state.phase) {
        case FlightPhase::IDLE: {
            // Auto-arm: pad reference captured + boot delay elapsed.
            // Airbrakes are not pyro — ARMED only enables launch detection.
            // Fires once per boot so a debug-mode DISARM sticks.
            if (!_auto_armed && pad_ready && now_ms >= AUTO_ARM_DELAY_MS &&
                now_ms - _last_auto_arm_try_ms >= 1000) {
                _last_auto_arm_try_ms = now_ms;
                if (flight_state_arm(now_ms)) {
                    _auto_armed = true;
                    LOG_INFO("Auto-armed");
                }
            }
            break;
        }

        case FlightPhase::ARMED: {
            const bool acc = confirm(now_ms, _launch_since,
                accel_mag > LAUNCH_ACCEL_THRESH_MSS, LAUNCH_CONFIRM_MS);
            const bool bar = confirm(now_ms, _launch_baro_since,
                pad_ready && baro_agl > LAUNCH_BARO_BACKUP_M,
                LAUNCH_BARO_CONFIRM_MS);
            if (acc || bar)
                enter(FlightPhase::BOOST, now_ms, acc ? "accel" : "baro backup");
            break;
        }

        case FlightPhase::BOOST: {
            const bool acc = confirm(now_ms, _burnout_since,
                ax < 0.0f, BURNOUT_CONFIRM_MS);
            const bool timeout =
                (now_ms - g_state.phase_entry_ms) > BOOST_MAX_MS;
            if (acc || timeout) {
                g_state.burnout_time_ms = now_ms;
                enter(FlightPhase::COAST, now_ms, acc ? "accel" : "timeout");
            }
            break;
        }

        case FlightPhase::COAST: {
            const bool v = confirm(now_ms, _apogee_since,
                vel < APOGEE_VEL_THRESH_MPS, APOGEE_CONFIRM_MS);
            const bool backup_armed =
                (now_ms - g_state.burnout_time_ms) > APOGEE_BACKUP_LOCKOUT_MS;
            const bool bar = confirm(now_ms, _apogee_baro_since,
                backup_armed && pad_ready &&
                baro_agl < _max_alt_agl - APOGEE_BARO_FALL_M,
                APOGEE_CONFIRM_MS);
            if (v || bar)
                enter(FlightPhase::DESCENT, now_ms, v ? "velocity" : "baro backup");
            break;
        }

        case FlightPhase::DESCENT: {
            const bool still = confirm(now_ms, _landed_since,
                fabsf(_baro_rate) < LANDED_VEL_MAX_MPS &&
                fabsf(accel_mag - 9.81f) < LANDED_ACCEL_THRESH_MSS,
                LANDED_CONFIRM_MS);
            if (still)
                enter(FlightPhase::LANDED, now_ms, "stillness");
            break;
        }

        case FlightPhase::LANDED:
        default:
            break;
    }
}

#include "control.h"
#include "config.h"
#include "debug.h"
#include "flight_state.h"
#include "storage.h"
#include "board.h"        // servo power high-side switch
#ifdef APEX_HIL
#include "hil.h"          // g_hil_dt_s — sim-authoritative timestep
#endif

#include <Arduino.h>
#include <math.h>

// ─── Servo PWM ────────────────────────────────────────────────────────────────
// Hardware PWM at 50 Hz on SERVO_PIN. 12-bit resolution → 20 ms / 4096 ≈
// 4.9 µs per count, fine for a 1000–2000 µs command range.
// analogWriteResolution() is global to all analogWrite() users — the servo
// is currently the only one (LEDs are digital).

#define SERVO_PWM_HZ        50
#define SERVO_PWM_RES_BITS  12

static void servo_write_us(float us) {
    const float period_us = 1e6f / SERVO_PWM_HZ;
    const float duty = us / period_us * (float)(1 << SERVO_PWM_RES_BITS);
    analogWrite(SERVO_PIN, (int)(duty + 0.5f));
}

// deployment fraction → pulse width. INVERTED: frac 0 → SERVO_MAX_US
// (retracted), frac 1 → SERVO_MIN_US (fully deployed). If the linkage runs the
// other way, swap SERVO_MIN_US / SERVO_MAX_US in config.h.
static float deploy_to_us(float frac) {
    return SERVO_MAX_US - frac * (float)(SERVO_MAX_US - SERVO_MIN_US);
}
static float deploy_to_deg(float frac) {
    return SERVO_MIN_DEG + frac * (SERVO_MAX_DEG - SERVO_MIN_DEG);
}

// ─── Control state ────────────────────────────────────────────────────────────

static float    _integral = 0.0f;
static float    _deploy   = 0.0f;   // rate-limited actual command
static uint32_t _last_ms  = 0;
static bool     _was_active = false;

void control_init() {
    analogWriteFrequency(SERVO_PIN, SERVO_PWM_HZ);
    analogWriteResolution(SERVO_PWM_RES_BITS);
    control_reset();
    LOG_INFO("Control init — servo pin %d, %d–%d us", SERVO_PIN,
             SERVO_MIN_US, SERVO_MAX_US);
}

void control_reset() {
    _integral   = 0.0f;
    _deploy     = 0.0f;
    _last_ms    = 0;        // dt anchor — first tick after reset uses 1/RATE
    _was_active = false;
    g_state.control = ControlState{};
    g_state.control.servo_angle_deg = deploy_to_deg(0.0f);
    servo_write_us(deploy_to_us(0.0f));   // hold retracted command
}

void control_arm() {
    // Power the servo (off through the pad sit), then bring it to retracted
    // with a smooth sweep from the assumed center pulse so it does not snap on
    // power-up. The sweep is a one-time blocking move at the deliberate ARM
    // event; skipped in HIL (no hardware, and it must not stall the packet loop).
    board_servo_power(true);
#ifndef APEX_HIL
    const int steps = SERVO_INIT_SWEEP_MS / 20;
    for (int i = 1; i <= steps; i++) {
        const float frac = 0.5f * (1.0f - (float)i / (float)steps);  // 0.5 → 0
        servo_write_us(deploy_to_us(frac));
        delay(20);
    }
#endif
    control_reset();
}

void control_disarm() {
    control_reset();
    board_servo_power(false);   // unpower servo when not armed
}

void control_update(uint32_t now_ms) {
#ifdef APEX_HIL
    // Sim-authoritative dt in HIL (g_hil_dt_s, from the SimPacket stream — see
    // fusion.cpp): one control tick per packet on simulated time. Keeps the
    // PID integral and servo rate-limit deterministic and immune to USB jitter.
    const float dt = g_hil_dt_s;
    _last_ms = now_ms;
#else
    float dt = 1.0f / RATE_CONTROL_HZ;
    if (_last_ms != 0 && now_ms > _last_ms) {
        dt = (now_ms - _last_ms) * 1e-3f;
        if (dt > 0.05f) dt = 1.0f / RATE_CONTROL_HZ;
    }
    _last_ms = now_ms;
#endif

    // Snapshot ISR-written fused state (200 Hz fusion timer in flight builds).
    noInterrupts();
    const float alt  = g_state.fused.altitude_agl_m;
    const float vel  = g_state.fused.velocity_mps;
    const float pred = g_state.fused.predicted_apogee_m;
    interrupts();

    // ── Deployment gates ──────────────────────────────────────────────────────
    const bool in_coast = (g_state.phase == FlightPhase::COAST);
    const bool active = in_coast
        && (now_ms - g_state.burnout_time_ms) >= POST_BURNOUT_LOCKOUT_MS
        && vel > 0.0f
        && vel < MACH_GATE_MPS
        && alt > MIN_DEPLOY_ALT_M;

    if (active && !_was_active) {
        LOG_INFO("Airbrake control ACTIVE");
        storage_log_event(LOG_EVENT_CONTROL_ACTIVE, "airbrake control active");
    }
    _was_active = active;

    // ── PID (MATLAB port — error in metres, D-term = velocity) ────────────────
    float desired;
    float error = 0.0f, p_term = 0.0f, i_term = 0.0f, d_term = 0.0f;
    if (active) {
        error = pred - TARGET_APOGEE_M;
        _integral += error * dt;
        p_term = PID_KP * error;
        i_term = PID_KI * _integral;
        d_term = PID_KD * vel;
        float u = p_term + i_term + d_term;
        u = fmaxf(PID_U_MIN, fminf(PID_U_MAX, u));
        desired = (u - PID_U_MIN) / (PID_U_MAX - PID_U_MIN);
    } else if (in_coast) {
        desired = _deploy;        // gated mid-coast — hold position
    } else {
        desired = 0.0f;           // retracted in every other phase
    }

    // ── Servo rate limit (full travel in 0.24 s) ──────────────────────────────
    const float max_delta = (1.0f / SERVO_FULL_TRAVEL_S) * dt;
    float delta = desired - _deploy;
    delta = fmaxf(-max_delta, fminf(max_delta, delta));
    _deploy = fmaxf(0.0f, fminf(1.0f, _deploy + delta));

    servo_write_us(deploy_to_us(_deploy));

    // ── Publish ───────────────────────────────────────────────────────────────
    g_state.control.deployment_frac = _deploy;
    g_state.control.servo_angle_deg = deploy_to_deg(_deploy);
    g_state.control.pid_error_m     = error;
    g_state.control.pid_integral    = _integral;
    g_state.control.pid_p_term      = p_term;
    g_state.control.pid_i_term      = i_term;
    g_state.control.pid_d_term      = d_term;
    g_state.control.active          = active;
    g_state.control.timestamp_ms    = now_ms;
}

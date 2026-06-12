#include "fusion.h"
#include "flight_state.h"
#include "sensors.h"
#include "gps.h"
#include "config.h"
#include "debug.h"

#include <MahonyAHRS.h>
#include <math.h>

// ─── Module State ─────────────────────────────────────────────────────────────

static Mahony _ahrs;

static float    _cf_alt_m    = 0.0f;
static float    _cf_vel_mps  = 0.0f;
static uint32_t _last_us     = 0;

// Baro rolling average — used for pad auto-capture and periodic re-zero
static float    _baro_buf[REZERO_BARO_SAMPLES];
static uint8_t  _baro_idx    = 0;
static float    _baro_sum    = 0.0f;
static uint8_t  _baro_count  = 0;

// Periodic re-zero state
static uint32_t _stable_since_ms = 0;
static uint32_t _last_rezero_ms  = 0;
static bool     _was_stable      = false;

// Mahony convergence guard (Bug 2 fix).
// Blocks the CF integrator until attitude has settled.
// Latch: set true once, never cleared — Mahony self-corrects during flight.
static bool     _attitude_converged  = false;
static uint16_t _conv_confirm_count  = 0;
static uint32_t _total_imu_samples   = 0;

// Phase transition tracking for velocity seeding at ground → flight
static FlightPhase _prev_phase = FlightPhase::IDLE;

// ─── Internal Helpers ─────────────────────────────────────────────────────────

static float pressure_to_alt_msl(float pa) {
    return 44330.0f * (1.0f - powf(pa / ISA_SEA_LEVEL_PA, 1.0f / 5.255f));
}

static float isa_density(float alt_msl_m) {
    float T = ISA_SEA_LEVEL_TEMP_K - ISA_LAPSE_RATE * alt_msl_m;
    T = fmaxf(T, 1.0f);
    return 1.225f * powf(T / ISA_SEA_LEVEL_TEMP_K, 4.256f);
}

// Closed-form ballistic apogee prediction — port of MATLAB/Python sim.
// Conservative: uses clean-flight Cd (no brakes), over-predicts → drives
// earlier deployment.
static float predict_apogee(float alt_agl_m, float vel_mps, float density) {
    if (vel_mps <= 0.0f) return alt_agl_m;
    float k   = 0.5f * density * REF_AREA_M2 * CD_CLEAN;
    float arg = (k * vel_mps * vel_mps) / (ROCKET_MASS_KG * 9.81f);
    return alt_agl_m + (ROCKET_MASS_KG / (2.0f * k)) * logf(1.0f + arg);
}

// Periodic pad re-zero while stationary in IDLE/ARMED.
// Handles thermal drift during long pad holds (West Texas sun, 1+ hour).
static void check_rezero(float accel_mag, float gyro_mag, uint32_t now_ms) {
    if (g_state.phase != FlightPhase::IDLE &&
        g_state.phase != FlightPhase::ARMED) return;

    if (_baro_count < REZERO_BARO_SAMPLES) return;
    if (now_ms - _last_rezero_ms < REZERO_INTERVAL_MS) return;

    bool stable = (accel_mag >= REZERO_ACCEL_MIN_MSS &&
                   accel_mag <= REZERO_ACCEL_MAX_MSS &&
                   gyro_mag  <= REZERO_GYRO_MAX_RADS);

    if (stable && !_was_stable) _stable_since_ms = now_ms;
    _was_stable = stable;

    if (!stable || (now_ms - _stable_since_ms) < REZERO_STABLE_MS) return;

    float avg_pa = _baro_sum / _baro_count;

    // Write altitude before pressure — pressure > 0 is the "commit" flag
    // checked by the CF below. Sequenced to avoid torn read in main loop.
    g_state.pad_altitude_msl_m = pressure_to_alt_msl(avg_pa);
    g_state.pad_pressure_pa    = avg_pa;

    _cf_alt_m   = 0.0f;
    _cf_vel_mps = 0.0f;
    _last_rezero_ms = now_ms;

    LOG_INFO("Pad re-zero: %.2f Pa / %.1f m MSL", avg_pa, g_state.pad_altitude_msl_m);
}

// ─── Public API ───────────────────────────────────────────────────────────────

void fusion_init() {
    // In HIL mode, fusion_update() is called at RATE_HIL_HZ (100 Hz) per SimPacket.
    // Mahony hardcodes invSampleFreq = 1/begin(freq) — must match actual call rate
    // or the quaternion integrates at wrong speed (Supervisor Q2 HIGH issue).
#ifdef APEX_HIL
    _ahrs.begin(RATE_HIL_HZ);
#else
    _ahrs.begin(RATE_FUSION_HZ);
#endif
    _last_us = micros();

    _attitude_converged = false;
    _conv_confirm_count = 0;
    _total_imu_samples  = 0;
    _prev_phase         = FlightPhase::IDLE;

    _cf_alt_m   = 0.0f;
    _cf_vel_mps = 0.0f;

    // Baro rolling average + re-zero state. Zero at boot anyway; explicit
    // resets make fusion_init() a full re-init so flight_state_reset() can
    // give a HIL session a boot-like fresh pad capture.
    _baro_idx        = 0;
    _baro_sum        = 0.0f;
    _baro_count      = 0;
    _stable_since_ms = 0;
    _last_rezero_ms  = 0;
    _was_stable      = false;

    // On a cold start pad_pressure_pa is 0 (zero-initialised FlightState).
    // On a warm restart (watchdog reset during pad hold) it may already be
    // set — preserve it rather than reverting to open-loop integration.
    if (g_state.pad_pressure_pa == 0.0f) {
        g_state.pad_altitude_msl_m = 0.0f;
    }

    LOG_INFO("Fusion init @ %d Hz", RATE_FUSION_HZ);
}

void fusion_on_armed() {
    // Arm is a deliberate human action — always take a fresh pad snapshot.
    // Drop the original `== 0.0f` guard: thermal drift over a multi-hour
    // pad hold (West Texas, >30°C) can shift baro by several Pa, and the
    // arm event is the last human-confirmed opportunity for a clean reference.
    //
    // Prefer the rolling average (best noise rejection). Fall back to a
    // single reading only if the buffer isn't full yet (< 1 s after boot).
    // Wrap the fallback sensors_get_baro call in noInterrupts() to prevent
    // a race with the 200 Hz ISR which also calls sensors_get_baro.
    if (_baro_count > 0) {
        float avg_pa = _baro_sum / _baro_count;
        g_state.pad_altitude_msl_m = pressure_to_alt_msl(avg_pa);
        g_state.pad_pressure_pa    = avg_pa;
    } else {
        noInterrupts();
        BaroData baro;
        bool have_baro = sensors_get_baro(baro);
        interrupts();
        if (have_baro && baro.pressure_pa > 50000.0f) {
            g_state.pad_altitude_msl_m = pressure_to_alt_msl(baro.pressure_pa);
            g_state.pad_pressure_pa    = baro.pressure_pa;
        }
    }

    _cf_alt_m   = 0.0f;
    _cf_vel_mps = 0.0f;
    _last_rezero_ms = millis();

    LOG_INFO("Armed: pad ref %.2f Pa / %.1f m MSL",
             g_state.pad_pressure_pa, g_state.pad_altitude_msl_m);
}

void fusion_update() {
    uint32_t now_us = micros();
    float dt = (now_us - _last_us) * 1e-6f;
    if (dt <= 0.0f || dt > 0.05f) dt = 1.0f / RATE_FUSION_HZ;
    _last_us = now_us;

    uint32_t now_ms = millis();

    // ── Read sensors ──────────────────────────────────────────────────────────
    ImuData   imu;
    HighGData hg;
    BaroData  baro;
    MagData   mag;

    bool have_imu  = sensors_get_imu(imu);
    bool have_hg   = sensors_get_highg(hg);
    bool have_baro = sensors_get_baro(baro);
    bool have_mag  = sensors_get_mag(mag);

    if (have_imu)  g_state.imu    = imu;
    if (have_hg)   g_state.high_g = hg;
    if (have_baro) g_state.baro   = baro;
    if (have_mag)  g_state.mag    = mag;

    if (!have_imu) return;

    _total_imu_samples++;

    // ── 1. Attitude — Mahony AHRS ─────────────────────────────────────────────
    // Must run every sample unconditionally — starving Mahony delays convergence.
    float gx_dps = imu.gyro_x_rads * (180.0f / PI);
    float gy_dps = imu.gyro_y_rads * (180.0f / PI);
    float gz_dps = imu.gyro_z_rads * (180.0f / PI);

    if (have_mag) {
        _ahrs.update(gx_dps, gy_dps, gz_dps,
                     imu.accel_x_mss, imu.accel_y_mss, imu.accel_z_mss,
                     mag.x_gauss, mag.y_gauss, mag.z_gauss);
    } else {
        _ahrs.updateIMU(gx_dps, gy_dps, gz_dps,
                        imu.accel_x_mss, imu.accel_y_mss, imu.accel_z_mss);
    }

    float roll  = _ahrs.getRollRadians();
    float pitch = _ahrs.getPitchRadians();
    float yaw   = _ahrs.getYawRadians();

    // Reconstruct quaternion from ZYX Euler for HIL/telemetry consumers.
    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f),   sy = sinf(yaw*0.5f);
    g_state.fused.attitude_q[0] = cr*cp*cy + sr*sp*sy;
    g_state.fused.attitude_q[1] = sr*cp*cy - cr*sp*sy;
    g_state.fused.attitude_q[2] = cr*sp*cy + sr*cp*sy;
    g_state.fused.attitude_q[3] = cr*cp*sy - sr*sp*cy;

    // ── 2. Accel source selection ──────────────────────────────────────────────
    float ax, ay, az;
    float accel_mag = sqrtf(imu.accel_x_mss * imu.accel_x_mss +
                            imu.accel_y_mss * imu.accel_y_mss +
                            imu.accel_z_mss * imu.accel_z_mss);

    if (have_hg && accel_mag > 14.0f * 9.81f) {
        ax = hg.accel_x_mss;
        ay = hg.accel_y_mss;
        az = hg.accel_z_mss;
        accel_mag = sqrtf(ax*ax + ay*ay + az*az);
    } else {
        ax = imu.accel_x_mss;
        ay = imu.accel_y_mss;
        az = imu.accel_z_mss;
    }

    // ── 3. Project accel onto world vertical ──────────────────────────────────
    // Body-frame "up" direction from roll and pitch (yaw doesn't affect vertical).
    float vx = -sinf(pitch);
    float vy =  sinf(roll) * cosf(pitch);
    float vz =  cosf(roll) * cosf(pitch);
    float vert_accel = ax*vx + ay*vy + az*vz - 9.81f;

    // ── 4. Baro buffer + Bug 1 auto-capture + periodic re-zero ────────────────
    if (have_baro) {
        if (_baro_count < REZERO_BARO_SAMPLES) {
            _baro_buf[_baro_idx] = baro.pressure_pa;
            _baro_sum += baro.pressure_pa;
            _baro_count++;
        } else {
            _baro_sum -= _baro_buf[_baro_idx];
            _baro_buf[_baro_idx] = baro.pressure_pa;
            _baro_sum += baro.pressure_pa;
        }
        _baro_idx = (_baro_idx + 1) % REZERO_BARO_SAMPLES;

        // Bug 1 fix: auto-capture pad reference once the rolling average
        // buffer is full (~1 s after boot). Fires once; fusion_on_armed()
        // and check_rezero() will overwrite it with refined values later.
        // Using the full average avoids the ±1.96m noise of a single sample
        // (validated against Seymour TX flight data).
        // Sanity check (> 50000 Pa) prevents capturing 0 if baro isn't ready.
        if (g_state.pad_pressure_pa == 0.0f &&
            _baro_count >= REZERO_BARO_SAMPLES &&
            baro.pressure_pa > 50000.0f) {
            float avg_pa = _baro_sum / _baro_count;
            g_state.pad_altitude_msl_m = pressure_to_alt_msl(avg_pa);
            g_state.pad_pressure_pa    = avg_pa;
            _last_rezero_ms = now_ms;
            LOG_INFO("Auto pad capture: %.2f Pa / %.1f m MSL",
                     avg_pa, g_state.pad_altitude_msl_m);
        }

        float gyro_mag = sqrtf(imu.gyro_x_rads * imu.gyro_x_rads +
                               imu.gyro_y_rads * imu.gyro_y_rads +
                               imu.gyro_z_rads * imu.gyro_z_rads);
        check_rezero(accel_mag, gyro_mag, now_ms);
    }

    // ── 5. Bug 2 fix: Mahony convergence guard ────────────────────────────────
    // Mahony starts at q=[1,0,0,0]. With Kp=0.5 the time constant is ~2 s.
    // During convergence the gravity projection is wrong → non-zero vert_accel
    // at rest → drift if the integrator is running.
    //
    // Guard requires both:
    //   a) FUSION_CONV_MIN_SAMPLES elapsed (time for Kp to act)
    //   b) FUSION_CONV_CONFIRM_COUNT consecutive samples with
    //      |vert_accel| < FUSION_CONV_VERT_ACCEL_MAX AND accel near 1g AND
    //      gyro low — confirms gravity is actually projected correctly.
    //
    // Once set, never cleared — Mahony self-corrects during flight, and in
    // flight phases the baro correction handles residual errors anyway.
    if (!_attitude_converged) {
        float gyro_mag = sqrtf(imu.gyro_x_rads * imu.gyro_x_rads +
                               imu.gyro_y_rads * imu.gyro_y_rads +
                               imu.gyro_z_rads * imu.gyro_z_rads);
        bool near_1g  = (accel_mag >= REZERO_ACCEL_MIN_MSS &&
                         accel_mag <= REZERO_ACCEL_MAX_MSS);
        bool low_gyro = (gyro_mag  <= REZERO_GYRO_MAX_RADS);
        bool low_resid= (fabsf(vert_accel) < FUSION_CONV_VERT_ACCEL_MAX);

        if (_total_imu_samples >= FUSION_CONV_MIN_SAMPLES &&
            near_1g && low_gyro && low_resid) {
            _conv_confirm_count++;
        } else {
            _conv_confirm_count = 0;
        }

        if (_conv_confirm_count >= FUSION_CONV_CONFIRM_COUNT) {
            _attitude_converged = true;
            LOG_INFO("Attitude converged at %lu ms", (unsigned long)now_ms);
        }
    }

    // ── 6. Complementary filter — altitude and velocity ───────────────────────
    //
    // Strategy by phase:
    //
    //   IDLE / ARMED / LANDED:  Bug 3 fix.  Rocket is stationary.  Bypass
    //     the integrator entirely — altitude tracks baro directly, velocity
    //     is clamped to zero.  Prevents any vert_accel residual (from Mahony
    //     transient or sensor bias) from integrating into launch-time error.
    //     When BOOST begins, the integrator starts from a clean AGL=0, vel=0
    //     with no accumulated drift.
    //
    //   BOOST / COAST / DESCENT:  α-β complementary filter.
    //     alpha: altitude baro weight (dimensionless, per tick).
    //     beta:  velocity correction gain (1/s) — SEPARATE from alpha/dt.
    //     The old code used `alpha * (alt_err / dt)` which simplified to
    //     1.0–4.0 × alt_err. The new beta terms (0.10, 1.00) are independent
    //     of dt and were validated against the Seymour TX baro noise
    //     (max spike: 101 Pa = 10.9 m → clamped to 5 m → 0.5/5.0 m/s
    //     correction in BOOST/COAST respectively).

    bool in_flight = (g_state.phase == FlightPhase::BOOST   ||
                      g_state.phase == FlightPhase::COAST   ||
                      g_state.phase == FlightPhase::DESCENT);
    bool pad_ready = (g_state.pad_pressure_pa > 0.0f);

    float baro_agl = (have_baro && pad_ready)
                   ? (pressure_to_alt_msl(baro.pressure_pa) - g_state.pad_altitude_msl_m)
                   : _cf_alt_m;

    // Velocity seeding at ground → flight transition.
    // Supervisor HIGH issue: without seeding, vel=0 at BOOST entry while
    // the rocket is already moving at ~9 m/s (150 ms confirmation window at
    // ~13g). Seed from current accel × half the confirmation window
    // (parabolic rise assumption). This prevents a false apogee trigger in
    // the first tick if apogee detection is ever checked outside COAST phase.
    bool was_ground = (_prev_phase == FlightPhase::IDLE ||
                       _prev_phase == FlightPhase::ARMED);
    if (was_ground && in_flight) {
        _cf_vel_mps = 0.5f * vert_accel * (LAUNCH_CONFIRM_MS * 0.001f);
        LOG_INFO("CF: seeded vel=%.1f m/s at launch transition", _cf_vel_mps);
    }
    _prev_phase = g_state.phase;

    if (!in_flight) {
        // Ground bypass: altitude from baro, velocity = 0.
        if (pad_ready && have_baro) {
            _cf_alt_m = baro_agl;
        }
        _cf_vel_mps = 0.0f;
    } else {
        float alpha, beta;
        switch (g_state.phase) {
            case FlightPhase::BOOST:
                alpha = CF_BOOST_ALPHA;
                beta  = CF_BOOST_BETA;
                break;
            case FlightPhase::COAST:
            default:
                alpha = CF_COAST_ALPHA;
                beta  = CF_COAST_BETA;
                break;
        }

        float vel_pred = _cf_vel_mps + vert_accel * dt;
        float alt_pred = _cf_alt_m   + _cf_vel_mps * dt;

        if (have_baro && pad_ready) {
            float alt_err = baro_agl - alt_pred;
            // Clamp spike outliers before applying correction.
            alt_err = fmaxf(-CF_ALT_ERR_CLAMP_M, fminf(CF_ALT_ERR_CLAMP_M, alt_err));

            _cf_alt_m   = alt_pred + alpha * alt_err;
            _cf_vel_mps = vel_pred + beta  * alt_err;
        } else {
            // No new baro sample this tick — normal at 200 Hz vs 50 Hz baro.
            // Only if the baro has been silent past BARO_DEAD_MS do we fall
            // back to trusted GPS altitude at weak gains (10 Hz, σ≈3 m,
            // >100 ms latency) so the CF stays bounded instead of drifting
            // on accel dead-reckoning. GPS never replaces a live baro.
            const bool baro_dead = pad_ready &&
                (g_state.baro.timestamp_ms == 0 ||
                 now_ms - g_state.baro.timestamp_ms > BARO_DEAD_MS);
            if (baro_dead && gps_trusted()) {
                float gps_agl = g_state.gps.altitude_msl_m
                              - g_state.pad_altitude_msl_m;
                float alt_err = gps_agl - alt_pred;
                alt_err = fmaxf(-CF_GPS_ERR_CLAMP_M,
                                fminf(CF_GPS_ERR_CLAMP_M, alt_err));
                _cf_alt_m   = alt_pred + CF_GPS_ALPHA * alt_err;
                _cf_vel_mps = vel_pred + CF_GPS_BETA  * alt_err;
            } else {
                // Pure dead-reckoning between samples / no aid available.
                _cf_alt_m   = alt_pred;
                _cf_vel_mps = vel_pred;
            }
        }
    }

    // ── 7. ISA density + apogee prediction ────────────────────────────────────
    float alt_msl  = _cf_alt_m + g_state.pad_altitude_msl_m;
    float density  = isa_density(alt_msl);
    float pred_apo = predict_apogee(_cf_alt_m, _cf_vel_mps, density);

    // ── 8. Publish ────────────────────────────────────────────────────────────
    g_state.fused.altitude_agl_m     = _cf_alt_m;
    g_state.fused.velocity_mps       = _cf_vel_mps;
    g_state.fused.accel_mps2         = vert_accel;
    g_state.fused.air_density_kgm3   = density;
    g_state.fused.predicted_apogee_m = pred_apo;
    g_state.fused.timestamp_ms       = now_ms;
}

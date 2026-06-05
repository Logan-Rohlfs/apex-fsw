#include "fusion.h"
#include "flight_state.h"
#include "sensors.h"
#include "config.h"
#include "debug.h"

#include <MahonyAHRS.h>
#include <math.h>

// ─── Module State ─────────────────────────────────────────────────────────────

static Mahony _ahrs;

// Complementary filter state
static float    _cf_alt_m    = 0.0f;
static float    _cf_vel_mps  = 0.0f;
static uint32_t _last_us     = 0;

// Baro rolling average for re-zero stability check
static float    _baro_buf[REZERO_BARO_SAMPLES];
static uint8_t  _baro_idx    = 0;
static float    _baro_sum    = 0.0f;
static uint8_t  _baro_count  = 0;

// Re-zero timing and motion state
static uint32_t _stable_since_ms = 0;
static uint32_t _last_rezero_ms  = 0;
static bool     _was_stable      = false;

// ─── Internal Helpers ─────────────────────────────────────────────────────────

static float pressure_to_alt_msl(float pa) {
    return 44330.0f * (1.0f - powf(pa / ISA_SEA_LEVEL_PA, 1.0f / 5.255f));
}

static float isa_density(float alt_msl_m) {
    float T = ISA_SEA_LEVEL_TEMP_K - ISA_LAPSE_RATE * alt_msl_m;
    T = fmaxf(T, 1.0f);
    // ρ = ρ₀ · (T/T₀)^(g/RL − 1), exponent ≈ 4.256 for dry air
    return 1.225f * powf(T / ISA_SEA_LEVEL_TEMP_K, 4.256f);
}

// Closed-form ballistic apogee prediction — direct port of MATLAB/Python sim.
// Assumes clean-flight Cd (conservative — over-predicts, drives earlier deployment).
static float predict_apogee(float alt_agl_m, float vel_mps, float density) {
    if (vel_mps <= 0.0f) return alt_agl_m;
    float k   = 0.5f * density * REF_AREA_M2 * CD_CLEAN;
    float arg = (k * vel_mps * vel_mps) / (ROCKET_MASS_KG * 9.81f);
    return alt_agl_m + (ROCKET_MASS_KG / (2.0f * k)) * logf(1.0f + arg);
}

// Check and perform pad altitude re-zero if conditions are met.
// Only fires in IDLE or ARMED. Requires sustained stillness and a minimum
// interval between re-zeros. Safe if it fires at T=0 — control loop
// activates 2.5 s post-burnout, by which point the CF has reconverged.
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

    float avg_pa           = _baro_sum / _baro_count;
    g_state.pad_pressure_pa    = avg_pa;
    g_state.pad_altitude_msl_m = pressure_to_alt_msl(avg_pa);

    _cf_alt_m   = 0.0f;
    _cf_vel_mps = 0.0f;
    _last_rezero_ms = now_ms;

    LOG_INFO("Pad re-zero: %.2f Pa  ref=%.1f m MSL", avg_pa, g_state.pad_altitude_msl_m);
}

// ─── Public API ───────────────────────────────────────────────────────────────

void fusion_init() {
    _ahrs.begin(RATE_FUSION_HZ);
    _last_us = micros();
    LOG_INFO("Fusion init @ %d Hz", RATE_FUSION_HZ);
}

void fusion_on_armed() {
    // Snapshot ground reference using current baro rolling average.
    // If the buffer isn't full yet, fall back to the latest single reading.
    BaroData baro;
    if (sensors_get_baro(baro) && g_state.pad_pressure_pa == 0.0f) {
        float pa = (_baro_count > 0) ? (_baro_sum / _baro_count) : baro.pressure_pa;
        g_state.pad_pressure_pa    = pa;
        g_state.pad_altitude_msl_m = pressure_to_alt_msl(pa);
    }
    _cf_alt_m   = 0.0f;
    _cf_vel_mps = 0.0f;
    _last_rezero_ms = millis();
    LOG_INFO("Armed: pad ref %.2f Pa / %.1f m MSL", g_state.pad_pressure_pa, g_state.pad_altitude_msl_m);
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

    // Mirror latest raw values into g_state so logging/plotting can read them
    // without competing with the sensor staging buffers.
    if (have_imu)  g_state.imu    = imu;
    if (have_hg)   g_state.high_g = hg;
    if (have_baro) g_state.baro   = baro;
    if (have_mag)  g_state.mag    = mag;

    if (!have_imu) return;

    // ── 1. Attitude — Mahony AHRS ─────────────────────────────────────────────
    // Library expects gyro in deg/s, accel in any consistent unit (normalised internally).
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

    // Mahony doesn't expose quaternion directly — use Euler angles.
    // Yaw does not affect the vertical projection so only roll/pitch matter.
    float roll  = _ahrs.getRollRadians();
    float pitch = _ahrs.getPitchRadians();
    float yaw   = _ahrs.getYawRadians();

    // Reconstruct quaternion from ZYX Euler for consumers (HIL, telemetry).
    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f),   sy = sinf(yaw*0.5f);
    g_state.fused.attitude_q[0] = cr*cp*cy + sr*sp*sy;  // w
    g_state.fused.attitude_q[1] = sr*cp*cy - cr*sp*sy;  // x
    g_state.fused.attitude_q[2] = cr*sp*cy + sr*cp*sy;  // y
    g_state.fused.attitude_q[3] = cr*cp*sy - sr*sp*cy;  // z

    // ── 2. Accel source selection — switch to ADXL375 near ICM saturation ─────
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
    // "Up" direction in body frame from roll and pitch (yaw does not affect vertical):
    //   vx = -sin(pitch)
    //   vy =  sin(roll) * cos(pitch)
    //   vz =  cos(roll) * cos(pitch)
    float vx = -sinf(pitch);
    float vy =  sinf(roll) * cosf(pitch);
    float vz =  cosf(roll) * cosf(pitch);
    float vert_accel = ax*vx + ay*vy + az*vz - 9.81f;

    // ── 4. Baro rolling average + re-zero ────────────────────────────────────
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

        float gyro_mag = sqrtf(imu.gyro_x_rads * imu.gyro_x_rads +
                               imu.gyro_y_rads * imu.gyro_y_rads +
                               imu.gyro_z_rads * imu.gyro_z_rads);
        check_rezero(accel_mag, gyro_mag, now_ms);
    }

    // ── 5. Complementary filter — altitude and velocity ───────────────────────
    // α blends baro (long-term stable) vs integrated accel (short-term accurate).
    // Reduce baro weight during boost — vibration makes BMP581 noisy.
    float alpha;
    switch (g_state.phase) {
        case FlightPhase::BOOST:   alpha = 0.005f; break;
        case FlightPhase::COAST:   alpha = 0.02f;  break;
        default:                   alpha = 0.05f;  break;
    }

    if (have_baro && g_state.pad_pressure_pa > 0.0f) {
        float baro_alt_agl = pressure_to_alt_msl(baro.pressure_pa) - g_state.pad_altitude_msl_m;

        // Predict forward with accel
        float vel_pred = _cf_vel_mps + vert_accel * dt;
        float alt_pred = _cf_alt_m   + _cf_vel_mps * dt;

        // Baro correction
        float alt_err  = baro_alt_agl - alt_pred;
        _cf_alt_m   = alt_pred  + alpha * alt_err;
        _cf_vel_mps = vel_pred  + alpha * (alt_err / dt);
    } else {
        _cf_vel_mps += vert_accel * dt;
        _cf_alt_m   += _cf_vel_mps * dt;
    }

    // ── 6. ISA density + apogee prediction ────────────────────────────────────
    float alt_msl   = _cf_alt_m + g_state.pad_altitude_msl_m;
    float density   = isa_density(alt_msl);
    float pred_apo  = predict_apogee(_cf_alt_m, _cf_vel_mps, density);

    // ── Write g_state.fused ────────────────────────────────────────────────────
    g_state.fused.altitude_agl_m     = _cf_alt_m;
    g_state.fused.velocity_mps       = _cf_vel_mps;
    g_state.fused.accel_mps2         = vert_accel;
    g_state.fused.air_density_kgm3   = density;
    g_state.fused.predicted_apogee_m = pred_apo;
    g_state.fused.timestamp_ms       = now_ms;
}

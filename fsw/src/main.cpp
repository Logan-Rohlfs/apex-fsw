#include <Arduino.h>
#include "config.h"
#include "debug.h"
#include "flight_state.h"
#include "sensors.h"
#include "fusion.h"
#include "gps.h"
#include "radio.h"

// ─── Timers ───────────────────────────────────────────────────────────────────

static IntervalTimer _timer_fusion;
static IntervalTimer _timer_baro;
static IntervalTimer _timer_mag;

// 200 Hz — read IMU + high-g, then run fusion pipeline
static void timer_fusion_cb() {
    sensors_update_imu();
    sensors_update_highg();
    fusion_update();
}

// 50 Hz — reads barometer
static void timer_baro_cb() {
    sensors_update_baro();
}

// 25 Hz — reads magnetometer
static void timer_mag_cb() {
    sensors_update_mag();
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

#ifdef APEX_DEBUG
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
#endif

    LOG_INFO("Apex FSW starting");

    bool all_ok = sensors_init();
    fusion_init();
    gps_init();   // non-fatal if GPS not present
    radio_init(); // non-fatal — verifies Si4463 SPI comms only, no TX
    if (!all_ok) {
        LOG_WARN("One or more sensors failed init (health=0x%02X)", sensors_health());
    }

    _timer_fusion.begin(timer_fusion_cb, 1000000 / RATE_FUSION_HZ);
    _timer_baro.begin(timer_baro_cb,     1000000 / RATE_BARO_HZ);
    _timer_mag.begin(timer_mag_cb,       1000000 / RATE_MAG_HZ);

    LOG_INFO("Timers started — fusion %d Hz, baro %d Hz, mag %d Hz",
             RATE_FUSION_HZ, RATE_BARO_HZ, RATE_MAG_HZ);

    g_state.phase = FlightPhase::IDLE;
}

// ─── Loop (background — runs in leftover cycles) ──────────────────────────────

void loop() {
    // GPS — poll at 10 Hz from main loop (non-time-critical)
    static uint32_t last_gps_ms = 0;
    if (millis() - last_gps_ms >= 100) {
        last_gps_ms = millis();
        gps_update();
    }

    static uint32_t last_print = 0;

#if defined(APEX_PLOT)
    // ── Plot output for Apex Monitor (50 Hz, >key:value format) ──────────────
    if (millis() - last_print >= 20) {
        last_print = millis();

        // Fusion
        Serial.printf(">alt_agl:%.2f\n",    g_state.fused.altitude_agl_m);
        Serial.printf(">velocity:%.3f\n",   g_state.fused.velocity_mps);
        Serial.printf(">pred_apogee:%.1f\n",g_state.fused.predicted_apogee_m);
        Serial.printf(">vert_accel:%.3f\n", g_state.fused.accel_mps2);

        // Raw sensors — read from g_state (written by fusion, never consumed)
        Serial.printf(">accel_x:%.3f\n",  g_state.imu.accel_x_mss);
        Serial.printf(">accel_y:%.3f\n",  g_state.imu.accel_y_mss);
        Serial.printf(">accel_z:%.3f\n",  g_state.imu.accel_z_mss);
        Serial.printf(">gyro_x:%.4f\n",   g_state.imu.gyro_x_rads);
        Serial.printf(">gyro_y:%.4f\n",   g_state.imu.gyro_y_rads);
        Serial.printf(">gyro_z:%.4f\n",   g_state.imu.gyro_z_rads);
        Serial.printf(">highg_x:%.2f\n",  g_state.high_g.accel_x_mss);
        Serial.printf(">highg_y:%.2f\n",  g_state.high_g.accel_y_mss);
        Serial.printf(">highg_z:%.2f\n",  g_state.high_g.accel_z_mss);
        Serial.printf(">baro_pa:%.2f\n",  g_state.baro.pressure_pa);
        Serial.printf(">baro_alt:%.1f\n",
            44330.0f * (1.0f - powf(g_state.baro.pressure_pa / ISA_SEA_LEVEL_PA, 1.0f / 5.255f)));
        Serial.printf(">baro_temp:%.2f\n",g_state.baro.temperature_c);
        Serial.printf(">mag_x:%.4f\n",    g_state.mag.x_gauss);
        Serial.printf(">mag_y:%.4f\n",    g_state.mag.y_gauss);
        Serial.printf(">mag_z:%.4f\n",    g_state.mag.z_gauss);

        // State
        Serial.printf("!phase:%s\n",   phase_name(g_state.phase));
        Serial.printf("!health:%d\n",  sensors_health());
        Serial.printf("!gps_fix:%d\n",   gps_fix_state());
        Serial.printf("!radio:%d\n",     radio_status());
        Serial.printf("!gps_sats:%d\n",g_state.gps.satellites);
        if (g_state.gps.time_valid) {
            char utc[32];
            gps_utc_string(utc, sizeof(utc));
            Serial.printf("!utc:%s\n", utc);
        }

        digitalToggle(LED_BUILTIN);
    }

#elif defined(APEX_DEBUG)
    // ── Human-readable debug output at 4 Hz ──────────────────────────────────
    if (millis() - last_print >= 250) {
        last_print = millis();

        float alt = 44330.0f * (1.0f - powf(g_state.baro.pressure_pa / ISA_SEA_LEVEL_PA, 1.0f / 5.255f));
        LOG_RAW("[IMU]  A: %6.2f %6.2f %6.2f m/s²  G: %6.3f %6.3f %6.3f rad/s\n",
            g_state.imu.accel_x_mss, g_state.imu.accel_y_mss, g_state.imu.accel_z_mss,
            g_state.imu.gyro_x_rads, g_state.imu.gyro_y_rads, g_state.imu.gyro_z_rads);
        LOG_RAW("[HG]   A: %6.2f %6.2f %6.2f m/s²\n",
            g_state.high_g.accel_x_mss, g_state.high_g.accel_y_mss, g_state.high_g.accel_z_mss);
        LOG_RAW("[BAR]  P: %.2f Pa  T: %.2f C  Alt: %.1f m\n",
            g_state.baro.pressure_pa, g_state.baro.temperature_c, alt);
        LOG_RAW("[MAG]  X: %.4f  Y: %.4f  Z: %.4f Gauss\n",
            g_state.mag.x_gauss, g_state.mag.y_gauss, g_state.mag.z_gauss);

        LOG_RAW("[FUS]  Alt: %7.1f m  Vel: %6.2f m/s  PredApo: %7.1f m  Phase: %s\n",
            g_state.fused.altitude_agl_m, g_state.fused.velocity_mps,
            g_state.fused.predicted_apogee_m, phase_name(g_state.phase));

        digitalToggle(LED_BUILTIN);
    }
#endif
}

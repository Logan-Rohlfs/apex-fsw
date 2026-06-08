#include <Arduino.h>
#include "config.h"
#include "debug.h"
#include "flight_state.h"
#include "sensors.h"
#include "fusion.h"
#include "gps.h"
#include "radio.h"
#include "led.h"
#include "storage.h"

// APEX_HIL is mutually exclusive with APEX_PLOT and APEX_DEBUG.
// Interleaved text output causes CRC false-failures in the Python parser.
#if defined(APEX_HIL) && (defined(APEX_PLOT) || defined(APEX_DEBUG))
#error "APEX_HIL cannot be combined with APEX_PLOT or APEX_DEBUG"
#endif

#ifdef APEX_HIL
#include "hil.h"
#endif

// ─── Hardware timers (normal mode only) ───────────────────────────────────────
#ifndef APEX_HIL

static IntervalTimer _timer_fusion;
static IntervalTimer _timer_baro;
static IntervalTimer _timer_mag;

static void timer_fusion_cb() {
    sensors_update_imu();
    sensors_update_highg();
    fusion_update();
}
static void timer_baro_cb() { sensors_update_baro(); }
static void timer_mag_cb()  { sensors_update_mag();  }

#endif // !APEX_HIL

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
#ifdef APEX_HIL
    // HIL mode — all sensor data arrives over USB serial as SimPackets.
    // Hardware sensors and timers are not initialised.
    Serial.begin(HIL_BAUD);
    while (!Serial && millis() < 5000);

    // Prominent boot warning. If this appears on a ground station display
    // during flight prep, abort and reflash the correct binary.
    Serial.println("#WARN: *** APEX HIL BUILD — NOT FOR FLIGHT ***");
    Serial.printf("#INFO: SimPacket=%u bytes  TeensyPacket=%u bytes  rate=%d Hz\n",
                  (unsigned)sizeof(SimPacket), (unsigned)sizeof(TeensyPacket), RATE_HIL_HZ);

    sensors_init_hil();
    fusion_init();   // uses RATE_HIL_HZ internally
    g_state.phase = FlightPhase::IDLE;

    // Signal to the Python replay script that the Teensy is ready.
    Serial.println("#HIL_READY");

#else
    // Normal flight / debug mode ─────────────────────────────────────────────
#ifdef APEX_DEBUG
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
#endif

    LOG_INFO("Apex FSW starting");

    pinMode(PIN_3V3_2_EN, OUTPUT);
    digitalWrite(PIN_3V3_2_EN, HIGH);

    led_init();
    storage_init();
    bool all_ok = sensors_init();
    fusion_init();
    gps_init();

    // Power-cycle the radio so the Si4463 always cold-boots into Boot state.
    // Without this, the chip retains state across firmware uploads since the
    // 3V3_2 rail stays live whenever the Teensy is powered.
    digitalWrite(PIN_3V3_2_EN, LOW);
    delay(50);
    digitalWrite(PIN_3V3_2_EN, HIGH);
    delay(200); // extended: allow Si4463 crystal to fully stabilise

    radio_init();
    if (!all_ok)
        LOG_WARN("One or more sensors failed init (health=0x%02X)", sensors_health());

#ifdef APEX_DEBUG
    radio_test_tx();
#endif

    _timer_fusion.begin(timer_fusion_cb, 1000000 / RATE_FUSION_HZ);
    _timer_baro.begin(timer_baro_cb,     1000000 / RATE_BARO_HZ);
    _timer_mag.begin(timer_mag_cb,       1000000 / RATE_MAG_HZ);

    LOG_INFO("Timers started — fusion %d Hz, baro %d Hz, mag %d Hz",
             RATE_FUSION_HZ, RATE_BARO_HZ, RATE_MAG_HZ);

    g_state.phase = FlightPhase::IDLE;
#endif
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

#ifdef APEX_HIL

void loop() {
    // ── Settle period: accumulate 50 baro readings (~0.5s) before arming ─────
    // Ensures pad_pressure_pa is captured from a stable average, not a single
    // cold-start reading. During settle, fusion runs but no TeensyPacket is sent.
    static bool     _armed        = false;
    static uint16_t _settle_count = 0;

    // ── Safety: halt if no SimPacket received within timeout ─────────────────
    // Prevents HIL binary from operating silently as a flight binary if
    // the USB cable is unplugged or the PC script never starts.
    static uint32_t _last_pkt_ms = millis();
    if (!_armed && (millis() - _last_pkt_ms > HIL_NO_PACKET_TIMEOUT_MS)) {
        Serial.println("#ERROR: HIL timeout — no SimPacket received. Halting.");
        while (true) {
            digitalToggle(LED_BUILTIN);
            delay(100); // rapid blink: distinct from flight firmware patterns
        }
    }

    // ── Read and parse incoming bytes ─────────────────────────────────────────
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        SimPacket pkt;
        if (!hil_parse(b, pkt)) continue;

        _last_pkt_ms = millis();
        sensors_inject_hil(pkt);

        // Warm-up: run fusion but defer TeensyPackets until settled + armed
        fusion_update();

        if (!_armed) {
            if (++_settle_count >= 50) {
                fusion_on_armed();
                g_state.phase = FlightPhase::ARMED;
                _armed = true;
                Serial.println("#INFO: Pad reference captured — state ARMED");
            }
            continue; // don't send TeensyPacket yet
        }

        // ── Send response ──────────────────────────────────────────────────
        TeensyPacket reply = hil_make_packet(pkt.sim_time_ms);
        hil_send(reply);

        // ── Monitor output (>key:value + !key:value) ──────────────────────
        // Rate-limited to every 4th packet (~25 Hz) to reduce serial congestion.
        // Python parser ignores these lines; monitor app uses them for display.
        static uint8_t _plot_div = 0;
        if (++_plot_div >= 4) {
            _plot_div = 0;
            Serial.printf(">alt_agl:%.2f\n",     g_state.fused.altitude_agl_m);
            Serial.printf(">velocity:%.3f\n",    g_state.fused.velocity_mps);
            Serial.printf(">pred_apogee:%.1f\n", g_state.fused.predicted_apogee_m);
            Serial.printf(">accel_x:%.3f\n",     g_state.imu.accel_x_mss);
            Serial.printf(">baro_pa:%.1f\n",     g_state.baro.pressure_pa);
            Serial.printf(">deployment:%.3f\n",  g_state.control.deployment_frac);
            Serial.printf("!phase:%s\n",          phase_name(g_state.phase));
            Serial.printf("!health:%d\n",         sensors_health());
        }

        static uint8_t _blink = 0;
        if (++_blink >= 50) { _blink = 0; digitalToggle(LED_BUILTIN); }
    }
}

#else // ── Normal / debug loop ──────────────────────────────────────────────────

void loop() {
    static uint32_t last_gps_ms = 0;
    if (millis() - last_gps_ms >= 100) {
        last_gps_ms = millis();
        gps_update();
    }

    static uint32_t last_print = 0;

#if defined(APEX_PLOT)
    if (millis() - last_print >= 20) {
        last_print = millis();

        Serial.printf(">alt_agl:%.2f\n",    g_state.fused.altitude_agl_m);
        Serial.printf(">velocity:%.3f\n",   g_state.fused.velocity_mps);
        Serial.printf(">pred_apogee:%.1f\n",g_state.fused.predicted_apogee_m);
        Serial.printf(">vert_accel:%.3f\n", g_state.fused.accel_mps2);
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
    }

#elif defined(APEX_DEBUG)
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
    }
#endif

    storage_mtp_loop();
    led_update();
}

#endif // APEX_HIL

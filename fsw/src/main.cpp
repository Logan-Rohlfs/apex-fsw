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
#include "control.h"

// APEX_HIL and APEX_DEBUG are mutually exclusive.
// Interleaved text output causes CRC false-failures in the Python parser.
#if defined(APEX_HIL) && defined(APEX_DEBUG)
#error "APEX_HIL cannot be combined with APEX_DEBUG"
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
    control_init();  // servo runs in HIL too — actuator-in-the-loop on the bench
    g_state.phase = FlightPhase::IDLE;

    // Signal to the Python replay script that the Teensy is ready.
    Serial.println("#HIL_READY");

#else
    // Normal flight / monitor mode ────────────────────────────────────────────
#ifdef APEX_DEBUG
    Serial.begin(921600);
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
    control_init();
    if (!all_ok)
        LOG_WARN("One or more sensors failed init (health=0x%02X)", sensors_health());

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
    static uint32_t _last_pkt_ms  = 0;   // 0 = no SimPacket yet this session

    // ── Session management ────────────────────────────────────────────────────
    // The host (run_hil.py / monitor) usually opens the port long after boot,
    // so a single boot-time #HIL_READY is lost. Announce once per second until
    // a SimPacket arrives. A ≥5 s packet gap ends the session: reset to IDLE
    // and announce again, so repeated HIL runs work without re-plugging USB.
    // (No flight-safety halt needed — this binary never reads real sensors.)
    if (_last_pkt_ms == 0 || millis() - _last_pkt_ms > HIL_SESSION_GAP_MS) {
        if (_armed || _settle_count > 0) {
            _armed = false;
            _settle_count = 0;
            g_state.phase = FlightPhase::IDLE;
            Serial.println("#INFO: HIL session ended — waiting for sim");
        }
        _last_pkt_ms = 0;
        static uint32_t _last_announce_ms = 0;
        if (millis() - _last_announce_ms >= 1000) {
            _last_announce_ms = millis();
            Serial.println("#HIL_READY");
            digitalToggle(LED_BUILTIN);   // slow blink: waiting for the sim
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
                flight_state_arm(millis());
                _armed = true;
                Serial.println("#INFO: Pad reference captured — state ARMED");
            }
            continue; // don't send TeensyPacket yet
        }

        // ── State machine + airbrake control (per packet = 100 Hz) ─────────
        flight_state_update(millis());
        control_update(millis());

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

// Telemetry beacon: off by default on the bench (monitor builds), always on
// in flight builds. Toggled with TELEM_ON / TELEM_OFF over USB.
#ifdef APEX_DEBUG
static bool _telem_enabled = false;
#else
static bool _telem_enabled = true;
#endif

void loop() {
    static uint32_t last_gps_ms = 0;
    if (millis() - last_gps_ms >= 100) {
        last_gps_ms = millis();
        gps_update();
    }

    // ── State machine + airbrake control at RATE_STATE_HZ ────────────────────
    // Fusion runs in the 200 Hz timer ISR; these read its output with brief
    // noInterrupts() snapshots internally.
    static uint32_t last_state_ms = 0;
    if (millis() - last_state_ms >= 1000 / RATE_STATE_HZ) {
        last_state_ms = millis();
        flight_state_update(millis());
        control_update(millis());
    }

    // ── Telemetry downlink ────────────────────────────────────────────────────
    // 1 Hz on the pad / after landing, RADIO_TELEM_FLIGHT_HZ once armed.
    // radio_telemetry_tx is non-blocking (~1 ms of SPI; ~42 ms airtime).
    static uint32_t last_telem_ms = 0;
    if (_telem_enabled) {
        const bool quiescent = (g_state.phase == FlightPhase::IDLE ||
                                g_state.phase == FlightPhase::LANDED);
        const uint32_t period_ms =
            1000U / (quiescent ? RADIO_TELEM_IDLE_HZ : RADIO_TELEM_FLIGHT_HZ);
        if (millis() - last_telem_ms >= period_ms) {
            last_telem_ms = millis();
            radio_telemetry_tx();
        }
    }

    static uint32_t last_print = 0;

#ifdef APEX_DEBUG
    // ── Data output at 50 Hz ──────────────────────────────────────────────────
    if (millis() - last_print >= 20) {
        last_print = millis();

        Serial.printf(">alt_agl:%.2f\n",     g_state.fused.altitude_agl_m);
        Serial.printf(">velocity:%.3f\n",    g_state.fused.velocity_mps);
        Serial.printf(">pred_apogee:%.1f\n", g_state.fused.predicted_apogee_m);
        Serial.printf(">vert_accel:%.3f\n",  g_state.fused.accel_mps2);
        Serial.printf(">accel_x:%.3f\n",     g_state.imu.accel_x_mss);
        Serial.printf(">accel_y:%.3f\n",     g_state.imu.accel_y_mss);
        Serial.printf(">accel_z:%.3f\n",     g_state.imu.accel_z_mss);
        Serial.printf(">gyro_x:%.4f\n",      g_state.imu.gyro_x_rads);
        Serial.printf(">gyro_y:%.4f\n",      g_state.imu.gyro_y_rads);
        Serial.printf(">gyro_z:%.4f\n",      g_state.imu.gyro_z_rads);
        Serial.printf(">highg_x:%.2f\n",     g_state.high_g.accel_x_mss);
        Serial.printf(">highg_y:%.2f\n",     g_state.high_g.accel_y_mss);
        Serial.printf(">highg_z:%.2f\n",     g_state.high_g.accel_z_mss);
        Serial.printf(">baro_pa:%.2f\n",     g_state.baro.pressure_pa);
        Serial.printf(">baro_alt:%.1f\n",
            44330.0f * (1.0f - powf(g_state.baro.pressure_pa / ISA_SEA_LEVEL_PA, 1.0f / 5.255f)));
        Serial.printf(">baro_temp:%.2f\n",   g_state.baro.temperature_c);
        Serial.printf(">mag_x:%.4f\n",       g_state.mag.x_gauss);
        Serial.printf(">mag_y:%.4f\n",       g_state.mag.y_gauss);
        Serial.printf(">mag_z:%.4f\n",       g_state.mag.z_gauss);
        Serial.printf("!phase:%s\n",         phase_name(g_state.phase));
        Serial.printf("!health:%d\n",        sensors_health());
        Serial.printf("!gps_fix:%d\n",       gps_fix_state());
        Serial.printf("!radio:%d\n",         radio_status());
        Serial.printf("!gps_sats:%d\n",      g_state.gps.satellites);

        // Telemetry TX stats for the monitor's Link panel (2 Hz)
        static uint32_t _last_link_ms = 0;
        if (millis() - _last_link_ms >= 500) {
            _last_link_ms = millis();
            uint16_t seq; uint32_t sent, skipped;
            radio_telemetry_stats(&seq, &sent, &skipped);
            Serial.printf("!telem:%d\n", _telem_enabled ? 1 : 0);
            Serial.printf("!tx_seq:%u\n", seq);
            Serial.printf("!tx_sent:%lu\n", (unsigned long)sent);
            Serial.printf("!tx_skipped:%lu\n", (unsigned long)skipped);
        }

        if (g_state.gps.time_valid) {
            char utc[32];
            gps_utc_string(utc, sizeof(utc));
            Serial.printf("!utc:%s\n", utc);
        }
    }

    // ── Command parser ────────────────────────────────────────────────────────
    // Reads newline-terminated ASCII commands from the monitor app.
    // Commands: ARM, DISARM (state machine hooks added in Phase 1).
    static char    _cmd_buf[32];
    static uint8_t _cmd_len = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (_cmd_len > 0) {
                _cmd_buf[_cmd_len] = '\0';
                LOG_INFO("CMD: %s", _cmd_buf);
                if (strcmp(_cmd_buf, "RADIO_MARKER") == 0 ||
                           strcmp(_cmd_buf, "RADIO_MARKER_441") == 0) {
                    radio_marker_tx(RADIO_FREQ_HZ);
                } else if (strcmp(_cmd_buf, "RADIO_DATA_TEST") == 0) {
                    radio_data_test_tx();
                } else if (strcmp(_cmd_buf, "TELEM_ON") == 0) {
                    _telem_enabled = true;
                    LOG_INFO("Telemetry beacon ON (%s, callsign in every frame)",
                             RADIO_CALLSIGN);
                } else if (strcmp(_cmd_buf, "TELEM_OFF") == 0) {
                    _telem_enabled = false;
                    LOG_INFO("Telemetry beacon OFF");
                } else if (strcmp(_cmd_buf, "ARM") == 0) {
                    if (g_state.phase == FlightPhase::IDLE ||
                        g_state.phase == FlightPhase::ARMED) {
                        flight_state_arm(millis());
                        LOG_INFO("ARMED (pad ref %.0f Pa)", g_state.pad_pressure_pa);
                    } else {
                        LOG_WARN("ARM refused in phase %s", phase_name(g_state.phase));
                    }
                } else if (strcmp(_cmd_buf, "DISARM") == 0) {
                    flight_state_disarm();
                    LOG_INFO("Phase now %s", phase_name(g_state.phase));
                }
                _cmd_len = 0;
            }
        } else if (_cmd_len < sizeof(_cmd_buf) - 1) {
            _cmd_buf[_cmd_len++] = c;
        }
    }
#endif

    storage_mtp_loop();
    led_update();
}

#endif // APEX_HIL

#include <Arduino.h>
#include "led.h"
#include "flight_state.h"
#include "sensors.h"
#include "gps.h"

// LED_BUILTIN (pin 13) is shared with SPI0 SCK (IMU). Hardware SPI traffic
// will smear the signal at 200 Hz — patterns are still readable on the pad
// but will look frenetic during boost/coast when the fusion timer is running.

void led_init() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

// Returns true if millis() falls within the half-open window [start, end)
// inside a repeating period of period_ms.
static bool in_pulse(uint16_t period_ms, uint16_t start_ms, uint16_t end_ms) {
    uint16_t t = (uint16_t)(millis() % period_ms);
    return t >= start_ms && t < end_ms;
}

// Returns the desired LED state for the current flight phase and system health.
static bool led_state() {
    FlightPhase phase = g_state.phase;
    uint8_t     health = sensors_health();

    switch (phase) {
        case FlightPhase::IDLE:
            // Sensor fault: rapid 10 Hz — do not arm
            if (health != (SENSOR_OK_IMU | SENSOR_OK_HIGHG | SENSOR_OK_BARO | SENSOR_OK_MAG))
                return in_pulse(100, 0, 50);

            // Waiting for 3-D GPS lock: double-blink every 2 s
            if (gps_fix_state() < 3)
                return in_pulse(2000, 0, 200) || in_pulse(2000, 350, 550);

            // All good, GPS acquired: single slow heartbeat pulse every 2 s
            return in_pulse(2000, 0, 100);

        case FlightPhase::ARMED:
            // Double fast blink every 1.5 s — "armed, stand clear"
            return in_pulse(1500, 0, 100) || in_pulse(1500, 200, 300);

        case FlightPhase::BOOST:
            // Fast 5 Hz blink — motor burning
            return in_pulse(200, 0, 100);

        case FlightPhase::COAST:
            // 2 Hz blink — coasting / airbrake control active
            return in_pulse(500, 0, 150);

        case FlightPhase::DESCENT:
            // 1 Hz blink — past apogee, descending
            return in_pulse(1000, 0, 300);

        case FlightPhase::LANDED:
            // Triple blink every 3 s — "find me"
            return in_pulse(3000, 0, 200) || in_pulse(3000, 400, 600) || in_pulse(3000, 800, 1000);

        default:
            return false;
    }
}

void led_update() {
    digitalWrite(LED_BUILTIN, led_state() ? HIGH : LOW);
}

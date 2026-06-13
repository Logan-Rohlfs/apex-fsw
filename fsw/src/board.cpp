#include "board.h"
#include "config.h"

#include <Arduino.h>

// ─── Power rails ──────────────────────────────────────────────────────────────

static bool _servo_powered = false;

void board_servo_power(bool on) {
    digitalWrite(PIN_SRV_EN, on ? HIGH : LOW);
    _servo_powered = on;
}

bool board_servo_powered() { return _servo_powered; }

// ─── Arm switches (debounced) ─────────────────────────────────────────────────
// Both must read SWITCH_ARMED_LEVEL, stable for SWITCH_DEBOUNCE_MS. INPUT_PULLUP
// means an open/broken switch floats HIGH; with SWITCH_ARMED_LEVEL == LOW that
// is the disarmed/safe state.

#define SWITCH_DEBOUNCE_MS  50

static bool     _armed_stable    = false;
static bool     _armed_candidate = false;
static uint32_t _armed_since_ms  = 0;

static void switches_update(uint32_t now_ms) {
    bool raw = (digitalRead(PIN_SWITCH_1) == SWITCH_ARMED_LEVEL) &&
               (digitalRead(PIN_SWITCH_2) == SWITCH_ARMED_LEVEL);
    if (raw != _armed_candidate) {
        _armed_candidate = raw;
        _armed_since_ms  = now_ms;
    } else if (now_ms - _armed_since_ms >= SWITCH_DEBOUNCE_MS) {
        _armed_stable = _armed_candidate;
    }
}

bool board_switches_armed() {
#ifdef APEX_HIL
    return true;   // no hardware switches in HIL — the sim drives arming
#else
    return _armed_stable;
#endif
}

// ─── Buzzer scheduler (non-blocking) ──────────────────────────────────────────
// Passive piezo: Teensy tone(pin, freq, ms) is non-blocking and self-stops, so
// we just retrigger at each pattern period. Active buzzer: manage the on-window
// with digitalWrite. A one-shot chirp pre-empts the next period boundary.

struct PatternDef { uint16_t period_ms; uint16_t on_ms; uint16_t freq_hz; };

// Indexed by BuzzerPattern.
static const PatternDef _patterns[] = {
    {    0,   0,    0 },   // BUZZ_SILENT
    {  250, 120, 2000 },   // BUZZ_FAULT  — fast/urgent
    { 2000,  80, 1500 },   // BUZZ_PREARM — slow heartbeat
    { 3000,  60, 2500 },   // BUZZ_ARMED  — occasional high chirp
};

static uint8_t  _pattern      = BUZZ_SILENT;
static uint32_t _next_beep_ms = 0;
static uint32_t _beep_off_ms  = 0;   // active-buzzer on-window end
static bool     _beep_on      = false;

static void beep_start(uint16_t freq, uint16_t on_ms, uint32_t now_ms) {
#if BUZZER_ACTIVE
    (void)freq;
    digitalWrite(PIN_BUZZER, HIGH);
    _beep_off_ms = now_ms + on_ms;
    _beep_on = true;
#else
    tone(PIN_BUZZER, freq, on_ms);   // non-blocking, self-stops
    (void)now_ms;
#endif
}

void board_buzzer(uint8_t pattern) {
    if (pattern == _pattern) return;
    _pattern = pattern;
    _next_beep_ms = 0;               // trigger on the next update
}

void board_buzzer_chirp() {
    beep_start(2700, 80, millis());
    // Nudge the periodic schedule so the chirp isn't immediately overlapped.
    _next_beep_ms = millis() + 120;
}

static void buzzer_update(uint32_t now_ms) {
#if BUZZER_ACTIVE
    if (_beep_on && (int32_t)(now_ms - _beep_off_ms) >= 0) {
        digitalWrite(PIN_BUZZER, LOW);
        _beep_on = false;
    }
#endif
    const PatternDef& p = _patterns[_pattern <= BUZZ_ARMED ? _pattern : 0];
    if (p.period_ms == 0) return;
    if (_next_beep_ms == 0 || (int32_t)(now_ms - _next_beep_ms) >= 0) {
        beep_start(p.freq_hz, p.on_ms, now_ms);
        _next_beep_ms = now_ms + p.period_ms;
    }
}

// ─── Init / update ────────────────────────────────────────────────────────────

void board_init() {
    pinMode(PIN_12V_EN, OUTPUT);
    digitalWrite(PIN_12V_EN, HIGH);   // 12 V on at boot

    pinMode(PIN_SRV_EN, OUTPUT);
    digitalWrite(PIN_SRV_EN, LOW);    // servo power off until ARM
    _servo_powered = false;

    pinMode(PIN_SWITCH_1, INPUT_PULLUP);
    pinMode(PIN_SWITCH_2, INPUT_PULLUP);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    _armed_stable = _armed_candidate = false;
    _armed_since_ms = 0;
    _pattern = BUZZ_SILENT;
    _next_beep_ms = 0;
}

void board_update(uint32_t now_ms) {
    switches_update(now_ms);
    buzzer_update(now_ms);
}

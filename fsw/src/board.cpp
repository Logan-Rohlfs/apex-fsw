#include "board.h"
#include "config.h"
#ifdef APEX_HIL
#include "hil.h"          // g_hil_arm_closed — sim-injected arm-switch state
#endif

#include <Arduino.h>

// ─── Power rails ──────────────────────────────────────────────────────────────

static bool _servo_powered = false;

void board_servo_power(bool on) {
    digitalWrite(PIN_SRV_EN, on ? HIGH : LOW);
    _servo_powered = on;
}

bool board_servo_powered() { return _servo_powered; }

// ─── Switches (debounced) ──────────────────────────────────────────────────────
// Each independently reads SWITCH_CLOSED_LEVEL, stable for SWITCH_DEBOUNCE_MS.
// INPUT_PULLUP means an open/broken switch floats HIGH; with
// SWITCH_CLOSED_LEVEL == LOW that is the safe default (arm interlock not
// satisfied / radio silent).

#define SWITCH_DEBOUNCE_MS  50

static bool     _arm_stable      = false;
static bool     _arm_candidate   = false;
static uint32_t _arm_since_ms    = 0;

static bool     _radio_stable    = false;
static bool     _radio_candidate = false;
static uint32_t _radio_since_ms  = 0;

static void switches_update(uint32_t now_ms) {
    bool arm_raw = (digitalRead(PIN_ARM_SWITCH) == SWITCH_CLOSED_LEVEL);
    if (arm_raw != _arm_candidate) {
        _arm_candidate = arm_raw;
        _arm_since_ms  = now_ms;
    } else if (now_ms - _arm_since_ms >= SWITCH_DEBOUNCE_MS) {
        _arm_stable = _arm_candidate;
    }

    bool radio_raw = (digitalRead(PIN_RADIO_SWITCH) == SWITCH_CLOSED_LEVEL);
    if (radio_raw != _radio_candidate) {
        _radio_candidate = radio_raw;
        _radio_since_ms  = now_ms;
    } else if (now_ms - _radio_since_ms >= SWITCH_DEBOUNCE_MS) {
        _radio_stable = _radio_candidate;
    }

    digitalWrite(PIN_12V_EN, board_radio_enabled() ? HIGH : LOW);
}

bool board_arm_switch_closed() {
#ifdef APEX_HIL
    return g_hil_arm_closed;   // sim drives the switch state (HIL_ARM_SWITCH_BIT)
#else
    return _arm_stable;
#endif
}

bool board_radio_enabled() {
#ifdef APEX_HIL
    return true;   // no external video TX / radio-silence concern in HIL
#else
    return _radio_stable;
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
static bool     _st_active    = false;   // self-test sequence in progress

static void beep_start(uint16_t freq, uint16_t on_ms, uint32_t now_ms) {
#if BUZZER_ACTIVE
    (void)freq;
    digitalWrite(PIN_BUZZER, HIGH);
#else
    // Passive piezo via hardware PWM (FlexPWM), NOT tone(). tone() needs a PIT
    // IntervalTimer, but all PITs are consumed by the fusion/baro/mag timers in
    // the flight/debug builds, so tone() silently fails there (it ignores
    // begin() returning false). analogWrite drives the pin's FlexPWM submodule
    // (D6 = FlexPWM2_2_A, independent of the servo on FlexPWM1_0_A) and works in
    // every build. ~50% duty at the 12-bit resolution control_init() sets.
    analogWriteFrequency(PIN_BUZZER, freq);
    analogWrite(PIN_BUZZER, 2048);
#endif
    _beep_off_ms = now_ms + on_ms;   // software on-window; closed in buzzer_update
    _beep_on = true;
}

void board_buzzer(uint8_t pattern) {
    if (pattern == _pattern && !_st_active) return;
    _pattern = pattern;
    _st_active = false;              // an explicit pattern change pre-empts self-test
    _next_beep_ms = 0;               // trigger on the next update
}

void board_buzzer_chirp() {
    beep_start(2700, 80, millis());
    // Nudge the periodic schedule so the chirp isn't immediately overlapped.
    _next_beep_ms = millis() + 120;
}

// ─── Self-test status sequence ────────────────────────────────────────────────
// One-shot "beep code": system i (1-based) plays i mid-tone beeps, a pause,
// then a status beep — two quick low beeps for OK or one long high tone for
// FAULT. The whole sequence is precomputed into _st_events on call and
// stepped through non-blocking in buzzer_update().

#define ST_MID_FREQ_HZ    1800
#define ST_MID_ON_MS      80
#define ST_MID_GAP_MS     120
#define ST_PAUSE_MS       350
#define ST_GOOD_FREQ_HZ   1100
#define ST_GOOD_ON_MS     70
#define ST_GOOD_GAP_MS    90
#define ST_BAD_FREQ_HZ    3200
#define ST_BAD_ON_MS      500
#define ST_SYSTEM_GAP_MS  600

#define ST_MAX_SYSTEMS    8
// Worst case per system: i mid beeps + 2 status beeps (OK is two beeps).
#define ST_MAX_EVENTS     ((ST_MAX_SYSTEMS * (ST_MAX_SYSTEMS + 1)) / 2 + ST_MAX_SYSTEMS * 2)

struct BeepEvent { uint16_t freq_hz; uint16_t on_ms; uint16_t gap_ms; };

static BeepEvent _st_events[ST_MAX_EVENTS];
static uint8_t   _st_count   = 0;
static uint8_t   _st_idx     = 0;
static uint32_t  _st_next_ms = 0;

void board_buzzer_selftest(const bool* system_ok, uint8_t count) {
    if (count > ST_MAX_SYSTEMS) count = ST_MAX_SYSTEMS;
    uint8_t n = 0;
    for (uint8_t sys = 0; sys < count; sys++) {
        const uint8_t beeps = sys + 1;   // system 1 -> 1 beep, system 2 -> 2, ...
        for (uint8_t b = 0; b < beeps; b++) {
            const uint16_t gap = (b == beeps - 1) ? ST_PAUSE_MS : ST_MID_GAP_MS;
            _st_events[n++] = { ST_MID_FREQ_HZ, ST_MID_ON_MS, gap };
        }
        if (system_ok[sys]) {
            _st_events[n++] = { ST_GOOD_FREQ_HZ, ST_GOOD_ON_MS, ST_GOOD_GAP_MS };
            _st_events[n++] = { ST_GOOD_FREQ_HZ, ST_GOOD_ON_MS, ST_SYSTEM_GAP_MS };
        } else {
            _st_events[n++] = { ST_BAD_FREQ_HZ, ST_BAD_ON_MS, ST_SYSTEM_GAP_MS };
        }
    }
    _st_count   = n;
    _st_idx     = 0;
    _st_next_ms = 0;
    _st_active  = (n > 0);
}

static void buzzer_update(uint32_t now_ms) {
    if (_beep_on && (int32_t)(now_ms - _beep_off_ms) >= 0) {
#if BUZZER_ACTIVE
        digitalWrite(PIN_BUZZER, LOW);
#else
        analogWrite(PIN_BUZZER, 0);   // 0% duty → silent
#endif
        _beep_on = false;
    }

    if (_st_active) {
        if (!_beep_on && (int32_t)(now_ms - _st_next_ms) >= 0) {
            if (_st_idx < _st_count) {
                const BeepEvent& e = _st_events[_st_idx++];
                beep_start(e.freq_hz, e.on_ms, now_ms);
                _st_next_ms = now_ms + e.on_ms + e.gap_ms;
            } else {
                _st_active = false;
                _next_beep_ms = 0;   // resume the periodic pattern fresh
            }
        }
        return;   // self-test owns the buzzer while active
    }

    const PatternDef& p = _patterns[_pattern <= BUZZ_ARMED ? _pattern : 0];
    if (p.period_ms == 0) return;
    if (_next_beep_ms == 0 || (int32_t)(now_ms - _next_beep_ms) >= 0) {
        beep_start(p.freq_hz, p.on_ms, now_ms);
        _next_beep_ms = now_ms + p.period_ms;
    }
}

// ─── Init / update ────────────────────────────────────────────────────────────

void board_init() {
    pinMode(PIN_SRV_EN, OUTPUT);
    digitalWrite(PIN_SRV_EN, LOW);    // servo power off until ARM
    _servo_powered = false;

    pinMode(PIN_ARM_SWITCH, INPUT_PULLUP);
    pinMode(PIN_RADIO_SWITCH, INPUT_PULLUP);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    _arm_stable = _arm_candidate = false;
    _arm_since_ms = 0;
    _radio_stable = _radio_candidate = false;
    _radio_since_ms = 0;
    _pattern = BUZZ_SILENT;
    _next_beep_ms = 0;

    // Read the radio switch synchronously (no debounce wait) so PIN_12V_EN
    // starts in the correct state at boot instead of defaulting on.
    _radio_stable = _radio_candidate =
        (digitalRead(PIN_RADIO_SWITCH) == SWITCH_CLOSED_LEVEL);
    pinMode(PIN_12V_EN, OUTPUT);
    digitalWrite(PIN_12V_EN, board_radio_enabled() ? HIGH : LOW);
}

void board_update(uint32_t now_ms) {
    switches_update(now_ms);
    buzzer_update(now_ms);
}

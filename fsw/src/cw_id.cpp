#include "cw_id.h"
#include "config.h"
#include "board.h"        // board_radio_enabled() — VTX powered?
#include "debug.h"

#include <Arduino.h>
#include <string.h>

#ifndef APEX_HIL

// ─── Morse table (A–Z, 0–9) ───────────────────────────────────────────────────
struct MorseEntry { char c; const char* code; };

static const MorseEntry MORSE[] = {
    {'A', ".-"},   {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},  {'E', "."},
    {'F', "..-."}, {'G', "--."},  {'H', "...."}, {'I', ".."},   {'J', ".---"},
    {'K', "-.-"},  {'L', ".-.."}, {'M', "--"},   {'N', "-."},   {'O', "---"},
    {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."},  {'S', "..."},  {'T', "-"},
    {'U', "..-"},  {'V', "...-"}, {'W', ".--"},  {'X', "-..-"}, {'Y', "-.--"},
    {'Z', "--.."},
    {'0', "-----"},{'1', ".----"},{'2', "..---"},{'3', "...--"},{'4', "....-"},
    {'5', "....."},{'6', "-...."},{'7', "--..."},{'8', "---.."},{'9', "----."},
};

static const char* morse_for(char c) {
    if (c >= 'a' && c <= 'z') c -= 32;
    for (const MorseEntry& m : MORSE)
        if (m.c == c) return m.code;
    return nullptr;   // unsupported char → skipped
}

// ─── Flattened keying plan ─────────────────────────────────────────────────────
// The whole callsign is expanded once into alternating tone/silence segments;
// the scheduler then just steps the index on a millis() timeline.
struct Segment { bool on; uint16_t ms; };

static Segment  _seg[160];
static uint16_t _seg_count   = 0;
static uint16_t _seg_idx     = 0;
static uint32_t _seg_end_ms  = 0;
static bool     _active      = false;   // currently keying an ID
static uint32_t _next_id_ms  = 0;       // when the next ID may start
static bool     _ready       = false;

static void push_seg(bool on, uint16_t ms) {
    if (_seg_count < (sizeof(_seg) / sizeof(_seg[0]))) {
        _seg[_seg_count].on = on;
        _seg[_seg_count].ms = ms;
        _seg_count++;
    }
}

static void build_plan() {
    _seg_count = 0;
    const uint16_t dot    = CW_DOT_MS;
    const uint16_t dash   = (uint16_t)(CW_DOT_MS * 3);
    const uint16_t intra  = CW_DOT_MS;            // gap between elements of a letter
    const uint16_t letter = (uint16_t)(CW_DOT_MS * 3);  // gap between letters

    const char* cs = CW_ID_CALLSIGN;
    const size_t n = strlen(cs);
    for (size_t i = 0; i < n; i++) {
        const char* code = morse_for(cs[i]);
        if (!code) continue;
        for (size_t e = 0; code[e] != '\0'; e++) {
            push_seg(true, code[e] == '-' ? dash : dot);
            if (code[e + 1] != '\0') push_seg(false, intra);   // between elements
        }
        if (i + 1 < n) push_seg(false, letter);                // between letters
    }
}

static inline void key_off() { analogWrite(PIN_CW_ID, 0); }
static inline void key_on()  { analogWrite(PIN_CW_ID, 1 << (CW_PWM_RES_BITS - 1)); } // ~50% duty

static void apply_segment(uint32_t now_ms) {
    _seg[_seg_idx].on ? key_on() : key_off();
    _seg_end_ms = now_ms + _seg[_seg_idx].ms;
}

void cw_id_init() {
    pinMode(PIN_CW_ID, OUTPUT);
    // QuadTimer3_3 — independent of the servo (FlexPWM2_3) and buzzer (FlexPWM2_2),
    // so this 1 kHz cannot perturb their frequencies. analogWriteResolution is set
    // globally by control_init(); we reuse it (CW_PWM_RES_BITS).
    analogWriteFrequency(PIN_CW_ID, CW_TONE_HZ);
    key_off();
    _active     = false;
    _seg_count  = 0;
    _next_id_ms = millis() + CW_ID_FIRST_DELAY_MS;
    _ready      = true;
    LOG_INFO("CW ID ready — \"%s\" on pin %d @ %d Hz, every %lu s",
             CW_ID_CALLSIGN, PIN_CW_ID, CW_TONE_HZ,
             (unsigned long)(CW_ID_INTERVAL_MS / 1000UL));
}

void cw_id_update(uint32_t now_ms) {
    if (!_ready) return;

    // Only ID while the VTX could actually be transmitting (radio switch on).
    // While radio-silent there is nothing to identify; hold off and re-ID soon
    // after the link comes back.
    if (!board_radio_enabled()) {
        if (_active) { key_off(); _active = false; }
        _next_id_ms = now_ms + CW_ID_FIRST_DELAY_MS;
        return;
    }

    if (!_active) {
        if ((int32_t)(now_ms - _next_id_ms) >= 0) {
            build_plan();
            if (_seg_count == 0) {            // nothing to send (bad callsign)
                _next_id_ms = now_ms + CW_ID_INTERVAL_MS;
                return;
            }
            _active  = true;
            _seg_idx = 0;
            apply_segment(now_ms);
        }
        return;
    }

    // Active: advance the plan as each element/gap completes.
    if ((int32_t)(now_ms - _seg_end_ms) >= 0) {
        _seg_idx++;
        if (_seg_idx >= _seg_count) {         // ID complete
            key_off();
            _active     = false;
            _next_id_ms = now_ms + CW_ID_INTERVAL_MS;
            return;
        }
        apply_segment(now_ms);
    }
}

#else  // APEX_HIL — no VTX in the loop; CW ID is a no-op so sim builds are unaffected.

void cw_id_init() {}
void cw_id_update(uint32_t) {}

#endif

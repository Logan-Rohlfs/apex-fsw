#pragma once
#include <stdint.h>
#include <stdbool.h>

// Board I/O: power rails, arm switches, and the status buzzer.
// See fsw/docs/flight_readiness.md (Phases A + D).

// Configure pins, drive 12 V on, servo power off, switches to pull-ups, buzzer
// idle. Call once at the very start of setup(), before anything that needs the
// 12 V rail.
void board_init();

// True when BOTH arm switches are in the armed position (debounced). The
// switches close to GND when armed (SWITCH_ARMED_LEVEL). A broken/unplugged
// switch floats to the safe side and reads disarmed. In the HIL build there is
// no hardware, so this always returns true (HIL arms via the sim).
bool board_switches_armed();

// Servo power high-side switch. ON at ARM, OFF in IDLE so the servo does not
// hold/chatter (and drain the pack) through the long pad sit.
void board_servo_power(bool on);
bool board_servo_powered();

// ── Buzzer (non-blocking) ─────────────────────────────────────────────────────
// Set the repeating status pattern; board_update() services it. Patterns are
// the only feedback to an operator on the pad with no uplink.
enum BuzzerPattern : uint8_t {
    BUZZ_SILENT = 0,
    BUZZ_FAULT,     // can't arm / storage fault — urgent fast triple
    BUZZ_PREARM,    // alive, waiting to arm — slow single beep
    BUZZ_ARMED,     // armed — occasional short high chirp
};
void board_buzzer(uint8_t pattern);
void board_buzzer_chirp();            // one-shot chirp over the current pattern
                                      // (GPS lock, ARM transition, etc.)

// Service switch debounce and the buzzer scheduler. Call every main-loop pass.
void board_update(uint32_t now_ms);

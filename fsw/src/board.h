#pragma once
#include <stdint.h>
#include <stdbool.h>

// Board I/O: power rails, arm switches, and the status buzzer.
// See fsw/docs/flight_readiness.md (Phases A + D).

// Configure pins, drive 12 V on, servo power off, switches to pull-ups, buzzer
// idle. Call once at the very start of setup(), before anything that needs the
// 12 V rail.
void board_init();

// True when the arm switch is closed (debounced) — one of several AND'd
// arming-interlock conditions in flight_state.cpp; closing it does not arm by
// itself. A broken/unplugged switch floats open/safe (SWITCH_CLOSED_LEVEL). In
// the HIL build there is no hardware: this returns the sim-injected state.
bool board_arm_switch_closed();

// True when the radio switch is closed — radio transmissions are enabled
// (onboard Si4463 TX permitted and PIN_12V_EN, the external video TX rail, is
// driven high). While open the FC is radio-silent. In the HIL build there is
// no external video TX or radio-silence concern, so this always returns true.
bool board_radio_enabled();

// Servo power high-side switch. OFF through the armed pad sit; launch detection
// enables it so the actuator is ready before burnout.
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

// One-shot status "beep code", non-blocking. For each system i (1-based,
// system_ok[0] is system 1): play i mid-tone beeps, pause, then a status
// beep — two quick low beeps for OK, one long high tone for FAULT. e.g.
// "system 1 OK" = one mid beep, pause, two low beeps. "system 3 FAULT" =
// three mid beeps, pause, one long high tone. Runs once, then restores
// whatever buzzer pattern was active before the call. Intended for setup()
// so the operator gets an audible go/no-go checklist with no display.
void board_buzzer_selftest(const bool* system_ok, uint8_t count);

// Service switch debounce and the buzzer scheduler. Call every main-loop pass.
void board_update(uint32_t now_ms);

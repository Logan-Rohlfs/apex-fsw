#pragma once
#include <stdint.h>

// Airbrake control loop + servo driver.
//
// PID is the MATLAB-tuned competition controller (rocket_sim_4_23_26.m,
// gains rescaled ft → m in config.h). Reference implementation validated
// against real flight data: sim/apex_sim/hil/fake_teensy.py::FlightLogic.
//
// Deployment gates (all must pass, see config.h "Airbrake Gates"):
//   phase == COAST, ≥ POST_BURNOUT_LOCKOUT_MS after burnout, ascending,
//   velocity < MACH_GATE_MPS, altitude > MIN_DEPLOY_ALT_M.
// Brakes retract on DESCENT entry and hold position while gated in COAST.

// Configure servo PWM and drive to the retracted position. Call from setup().
void control_init();

// Reset the controller to the fresh-flight condition: clears the PID
// integral, the rate-limited deployment command, the dt anchor and the
// active-edge latch, zeroes g_state.control and drives the servo retracted.
// Called from flight_state_arm() — arming is the "new flight" boundary —
// so a re-arm (or a second HIL session without a power cycle) can never
// start with a stale integral. With PID_KI < 0 a leftover positive integral
// suppresses u → late deployment and early close.
void control_reset();

// Run one control tick. Call at RATE_CONTROL_HZ from the main loop, after
// flight_state_update(). Computes PID + gates, applies the servo rate limit,
// writes g_state.control and the servo output.
void control_update(uint32_t now_ms);

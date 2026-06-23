#pragma once
#include <stdint.h>

// CW (Morse) station ID for the analog video downlink. Keys a 1 kHz tone on
// PIN_CW_ID (into the VTX audio input) spelling CW_ID_CALLSIGN every
// CW_ID_INTERVAL_MS while the radio switch is on. Fully non-blocking — the tone
// is hardware PWM and the element timing is a millis() scheduler, so it never
// stalls the flight loop. See config.h for pin/timer/timing rationale.

// Configure the CW output pin (QuadTimer PWM) and arm the scheduler. Call once
// from setup() after control_init() (which sets the global PWM resolution).
void cw_id_init();

// Advance the non-blocking keying scheduler. Call every main-loop pass.
void cw_id_update(uint32_t now_ms);

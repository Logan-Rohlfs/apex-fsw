#pragma once

// Hardware watchdog (Teensy 4 RTWDOG). See fsw/docs/flight_readiness.md Phase C.
//
// Purpose: survive a hang during the long pad sit (or anywhere). If the main
// loop stops feeding for WATCHDOG_TIMEOUT_S, the board resets, reboots through
// setup(), and returns to IDLE — re-arming automatically once the pad reference
// is recaptured if the arm switches are still closed and storage is healthy. So
// a wedge while the rocket sits on the rail self-heals with no human action.
//
// Mid-flight reset is best-effort only: the FC will reboot but has no way to
// recover its in-flight phase, so it will likely miss the rest of that flight.
// Acceptable because recovery (drogue/main) is a separate, independent altimeter.

// Start the watchdog. Call once at the very END of setup(), after all slow
// init (storage mount, radio power-cycle) has finished.
void watchdog_begin();

// Pet the dog. Call once every main-loop pass, before any potentially long op.
void watchdog_feed();

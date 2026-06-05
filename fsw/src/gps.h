#pragma once
#include <stdint.h>
#include <stddef.h>

// Call once from setup() after Wire2.begin().
bool gps_init();

// Returns current fix state: -1=offline (init failed), 0=searching, 2=2D, 3=3D, 4=3D+DR
int8_t gps_fix_state();

// Call from the main loop at ~10 Hz.
// Checks for new NAV-PVT data and updates g_state.gps.
void gps_update();

// Returns a human-readable UTC string, e.g. "2026-06-04T14:32:07.000Z".
// Returns "NO FIX" if time is not yet valid.
void gps_utc_string(char* buf, size_t len);

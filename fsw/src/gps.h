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

// Fix trust model — call at ~10–100 Hz from the main loop in BOTH builds
// (after gps_update() on hardware; after sensors_inject_hil() in HIL).
// Tracks staleness, fix-loss/regain transitions (phase-aware: loss during
// BOOST is expected — AIRBORNE4g cannot track a 14 g launch), and requires
// GPS_REACQUIRE_EPOCHS consecutive fresh fixes before re-trusting, because
// the first epochs after reacquisition are often garbage.
void gps_monitor_update(uint32_t now_ms);

// True only when there is a current 3D fix that has survived the
// reacquisition confirmation window. The ONLY GPS signal fusion may consume
// (baro-dead altitude fallback). Control and state gates never use GPS.
bool gps_trusted();

// Returns a human-readable UTC string, e.g. "2026-06-04T14:32:07.000Z".
// Returns "NO FIX" if time is not yet valid.
void gps_utc_string(char* buf, size_t len);

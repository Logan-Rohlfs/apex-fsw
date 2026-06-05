#pragma once
#include "flight_state.h"

// ─── Init ─────────────────────────────────────────────────────────────────────
// Call once from setup(). Returns true if all sensors initialised successfully.
// Logs individual failures via LOG_ERROR even on partial success.

bool sensors_init();

// ─── Update (call from timer ISRs) ────────────────────────────────────────────
// Each function reads the corresponding sensor into an internal staging buffer
// and sets a ready flag. Safe to call from interrupt context — no dynamic
// allocation, no Serial output.
//
// Call schedule:
//   sensors_update_imu()   — RATE_FUSION_HZ  (200 Hz timer)
//   sensors_update_highg() — RATE_FUSION_HZ  (200 Hz timer, same callback)
//   sensors_update_baro()  — RATE_BARO_HZ    (50 Hz timer)
//   sensors_update_mag()   — RATE_MAG_HZ     (25 Hz timer)

void sensors_update_imu();
void sensors_update_highg();
void sensors_update_baro();
void sensors_update_mag();

// ─── Get (call from fusion layer) ─────────────────────────────────────────────
// Atomic copy of the staging buffer into `out`. Clears the ready flag.
// Returns false if no new data is available since the last call.

bool sensors_get_imu(ImuData& out);
bool sensors_get_highg(HighGData& out);
bool sensors_get_baro(BaroData& out);
bool sensors_get_mag(MagData& out);

// ─── Health ───────────────────────────────────────────────────────────────────
// Bitmask of which sensors are online. Set during sensors_init().

#define SENSOR_OK_IMU    (1 << 0)
#define SENSOR_OK_HIGHG  (1 << 1)
#define SENSOR_OK_BARO   (1 << 2)
#define SENSOR_OK_MAG    (1 << 3)

uint8_t sensors_health();

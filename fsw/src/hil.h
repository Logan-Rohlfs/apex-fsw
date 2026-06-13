#pragma once
#include <Arduino.h>
#include "flight_state.h"

// HIL serial protocol — Teensy ↔ Python sim (USB serial)
// Both directions use little-endian packed structs + CRC-8.

#define HIL_MAGIC_SIM_TO_TEENSY     0xABCD
#define HIL_MAGIC_TEENSY_TO_SIM     0xCDAB
#define HIL_BAUD                    921600

// ─── Sim → Teensy (64 bytes) ──────────────────────────────────────────────────
// Injected at RATE_CONTROL_HZ (100 Hz).
// accel/gyro in body frame. gps_alt_m = NaN when no fix.

struct __attribute__((packed)) SimPacket {
    uint16_t magic;             // HIL_MAGIC_SIM_TO_TEENSY
    uint32_t sim_time_ms;
    float    accel_x_mss;       // ICM body-frame m/s²
    float    accel_y_mss;
    float    accel_z_mss;
    float    gyro_x_rads;       // ICM body-frame rad/s
    float    gyro_y_rads;
    float    gyro_z_rads;
    float    baro_pa;           // BMP581 Pa
    float    highg_x_mss;       // ADXL375 body-frame m/s²
    float    highg_y_mss;
    float    highg_z_mss;
    float    mag_x_gauss;       // MMC5983MA body-frame Gauss
    float    mag_y_gauss;
    float    mag_z_gauss;
    float    gps_alt_msl_m;     // NaN = no fix
    uint8_t  gps_valid;         // bitfield: see HIL_GPS_FIX_BIT / HIL_ARM_SWITCH_BIT
    uint8_t  crc8;
};
static_assert(sizeof(SimPacket) == 64, "SimPacket size mismatch");

// SimPacket.gps_valid is a bitfield (kept in one byte so the wire stays 64 B):
//   bit 0 — GPS fix valid (the original meaning)
//   bit 1 — arm switches CLOSED (operator-armed). Lets the sim drive the real
//           arming gate instead of the firmware hardcoding switches=armed, so a
//           HIL run exercises the same IDLE→ARMED→disarm path as flight.
#define HIL_GPS_FIX_BIT     0x01
#define HIL_ARM_SWITCH_BIT  0x02

// ─── Teensy → Sim (24 bytes) ──────────────────────────────────────────────────
// Sent every control loop tick.

struct __attribute__((packed)) TeensyPacket {
    uint16_t magic;             // HIL_MAGIC_TEENSY_TO_SIM
    uint32_t sim_time_ms;       // echo for latency measurement
    float    deployment_frac;   // 0.0–1.0
    float    est_alt_agl_m;     // fused altitude AGL
    float    est_vel_mps;       // fused vertical velocity
    float    pred_apogee_m;     // PID prediction
    uint8_t  phase;             // FlightPhase enum value
    uint8_t  crc8;
};
static_assert(sizeof(TeensyPacket) == 24, "TeensyPacket size mismatch");

// HIL fusion rate — Mahony must be initialised at this rate, not RATE_FUSION_HZ.
// Using begin(RATE_FUSION_HZ=200) but calling fusion_update() at 100 Hz
// would make the quaternion integrate at half-speed (Supervisor Q2 HIGH issue).
#define RATE_HIL_HZ  100

// A SimPacket gap this long ends the HIL session: the firmware resets to
// IDLE and resumes announcing #HIL_READY once per second, so a host can
// connect (or reconnect) at any time without power-cycling the Teensy.
#define HIL_SESSION_GAP_MS  5000

#ifdef APEX_HIL

// Sim-authoritative timestep (seconds) for the packet currently being
// processed, derived by the HIL loop from consecutive SimPacket.sim_time_ms.
// fusion_update() and control_update() read this instead of wall-clock
// micros()/millis() so the estimator advances on *simulated* time — one
// real packet, one real step — immune to USB delivery jitter, and correct
// even if a packet is dropped or the host streams at a different rate.
extern float g_hil_dt_s;

// Injected arm-switch state (HIL_ARM_SWITCH_BIT of the latest SimPacket).
// board_switches_armed() returns this in HIL, so the sim drives arming exactly
// as the operator's screw switches would in flight. Defaults open (false).
extern bool g_hil_arm_closed;

// ─── CRC-8 ────────────────────────────────────────────────────────────────────
// Polynomial 0x07 (CRC-8/SMBUS). Covers all bytes except the crc8 field itself.
// Table-driven for predictable latency.

uint8_t hil_crc8(const uint8_t* data, size_t len);

// ─── Parse / Emit ─────────────────────────────────────────────────────────────
// Returns true if a complete valid packet was parsed.
bool     hil_parse(uint8_t byte, SimPacket& out);
void     hil_send(const TeensyPacket& pkt);

// Populate a TeensyPacket from g_state.
TeensyPacket hil_make_packet(uint32_t echo_time_ms);

#endif // APEX_HIL

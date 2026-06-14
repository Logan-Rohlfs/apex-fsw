#pragma once
#include <stdint.h>

// Initialise SPI1, boot the Si4463, and verify it responds with the correct
// part ID. Safe with no antenna — POWER_UP does not enable the PA.
// Returns true if chip is found and part ID is 0x4463.
bool radio_init();

// -1 = offline (SPI no response or wrong part ID)
//  0 = online (verified and booted Si4463, not yet configured for TX)
int8_t radio_status();

// Bench aid: blink a CW carrier on/off at a single frequency.
bool radio_marker_tx(uint32_t freq_hz);

// Bench aid: transmit a burst of 2-GFSK test frames (10 kbps, ±25 kHz dev).
// Frame: 0xAA x8, sync 0x2DD4, type 0x01, seq, "APEX RADIO TEST", CRC-16-CCITT.
// Decoded by sim/scripts/radio_gfsk_rx.py / the monitor's RTL-SDR source.
bool radio_data_test_tx();

// Transmit one telemetry frame. FLIGHT (type 0x02, every beat): callsign,
// seq, phase, health, GPS, fused state, accel_z/roll/airbrake/baro. One beat
// per second sends HOUSEKEEPING (type 0x03) instead: mag, high-g, off-axis
// gyro, uptime. Non-blocking — skips the beat if a frame is still on air.
// Phase and health retain their byte positions while upper bits carry
// operational and system-health flags decoded by radio_gfsk_rx.py.
// Callsign in every FLIGHT frame satisfies FCC §97.119 station ID.
bool radio_telemetry_tx();

// TX-side counters for the monitor's Link panel: next seq, frames sent,
// beats skipped because the previous frame was still on air.
void radio_telemetry_stats(uint16_t* seq, uint32_t* sent, uint32_t* skipped);

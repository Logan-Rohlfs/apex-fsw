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
// Frame: 0xAA x8, sync 0x2DD4, seq, "APEX RADIO TEST", CRC-16-CCITT.
// Decoded by sim/scripts/radio_gfsk_rx.py / the monitor's RTL-SDR source.
bool radio_data_test_tx();

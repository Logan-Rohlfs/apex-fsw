#pragma once
#include <stdint.h>

// Initialise SPI1 and verify the Si4463 responds with the correct part ID.
// Safe with no antenna — no POWER_UP or TX commands are sent.
// Returns true if chip is found and part ID is 0x4463.
bool radio_init();

// -1 = offline (SPI no response or wrong part ID)
//  0 = online (verified Si4463, not yet configured for TX)
int8_t radio_status();

// Boot the Si4463 and transmit a CW carrier at RADIO_FREQ_HZ.
// Requires radio_init() to have succeeded. Carrier stays on until reset.
// Only call this for bench testing — not part of the flight sequence.
bool radio_test_tx();

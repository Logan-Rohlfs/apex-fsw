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

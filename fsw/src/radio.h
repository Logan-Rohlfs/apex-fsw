#pragma once
#include <stdint.h>

// Initialise SPI1 and verify the Si4463 responds with the correct part ID.
// Safe with no antenna — no POWER_UP or TX commands are sent.
// Returns true if chip is found and part ID is 0x4463.
bool radio_init();

// -1 = offline (SPI no response or wrong part ID)
//  0 = online (verified Si4463, not yet configured for TX)
int8_t radio_status();

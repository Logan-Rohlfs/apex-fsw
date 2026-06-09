#pragma once
#include <stdint.h>

// Initialise SPI1, boot the Si4463, and verify it responds with the correct
// part ID. Safe with no antenna — POWER_UP does not enable the PA.
// Returns true if chip is found and part ID is 0x4463.
bool radio_init();

// -1 = offline (SPI no response or wrong part ID)
//  0 = online (verified and booted Si4463, not yet configured for TX)
int8_t radio_status();

// Configure the booted Si4463 and transmit a CW carrier at RADIO_FREQ_HZ.
// Requires radio_init() to have succeeded. Carrier stays on until reset.
// Only call this for bench testing — not part of the flight sequence.
bool radio_test_tx();

// Bench aid: wiggle radio pins slowly so they can be checked with a DMM.
void radio_dmm_pin_test();

// Bench aid: sweep a CW marker across the likely frequency range.
bool radio_sweep_tx();

// Bench aid: blink a CW carrier on/off at a single frequency.
bool radio_marker_tx(uint32_t freq_hz);

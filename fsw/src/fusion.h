#pragma once
#include <stdint.h>

// Called once from setup() after sensors_init().
void fusion_init();

// Called from 200 Hz timer ISR. Reads sensors, runs Mahony AHRS +
// complementary filter, writes results to g_state.fused.
void fusion_update();

// Call when transitioning to ARMED. Captures pad pressure reference
// and resets the complementary filter to AGL = 0.
void fusion_on_armed();

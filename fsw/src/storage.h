#pragma once
#include <stdint.h>

#define STORAGE_OK_FLASH  (1 << 0)
#define STORAGE_OK_SD     (1 << 1)

// Mount and verify both storage media. Returns health bitmask.
// Safe to call if either medium is absent — failures are logged, not fatal.
uint8_t storage_init();

uint8_t storage_health();

// Call from the main loop to service USB MTP file transfers.
void storage_mtp_loop();

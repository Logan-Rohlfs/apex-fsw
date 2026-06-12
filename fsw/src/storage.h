#pragma once
#include <stdint.h>
#include <stdbool.h>

#define STORAGE_OK_FLASH  (1 << 0)
#define STORAGE_OK_SD     (1 << 1)

// Binary log format identifiers. The decoder uses these to build one wide CSV.
#define APEX_LOG_MAGIC    0x4C585041UL  // "APXL" little-endian
#define APEX_LOG_VERSION  1

enum LogRecordType : uint8_t {
    LOG_REC_BOOT    = 1,
    LOG_REC_EVENT   = 2,
    LOG_REC_SAMPLE  = 3,
};

enum LogEventId : uint8_t {
    LOG_EVENT_BOOT              = 1,
    LOG_EVENT_ARMED             = 2,
    LOG_EVENT_DISARMED          = 3,
    LOG_EVENT_LAUNCH_DETECTED   = 4,
    LOG_EVENT_PHASE             = 5,
    LOG_EVENT_CONTROL_ACTIVE    = 6,
    LOG_EVENT_STORAGE_FAULT     = 7,
    LOG_EVENT_HIL_SESSION_END   = 8,
    LOG_EVENT_GPS_FIX_LOST      = 9,
    LOG_EVENT_GPS_FIX_REGAINED  = 10,
};

enum LogFault : uint16_t {
    LOG_FAULT_NONE         = 0,
    LOG_FAULT_FLASH_WRITE  = 1 << 0,
    LOG_FAULT_SD_WRITE     = 1 << 1,
    LOG_FAULT_FILE_OPEN    = 1 << 2,
    LOG_FAULT_RECORD_DROP  = 1 << 3,
};

// Mount and verify both storage media. Returns health bitmask.
// Storage failure is launch-fatal: flight_state_arm() refuses to arm unless
// storage_logging_ready() is true. Once airborne, logging faults are recorded
// but must not block control.
uint8_t storage_init();

uint8_t storage_health();
uint16_t storage_faults();
bool storage_logging_ready();
uint32_t storage_boot_id();
uint32_t storage_flight_id();

void storage_log_event(uint8_t event_id, const char* detail);
void storage_begin_flight(uint32_t now_ms, const char* reason);
void storage_log_update(uint32_t now_ms);
void storage_end_session(uint32_t now_ms, const char* reason);

// Call from the main loop to service USB MTP file transfers.
void storage_mtp_loop();

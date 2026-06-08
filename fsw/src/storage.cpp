#include "storage.h"
#include "debug.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <SD.h>
#include <MTP_Teensy.h>

static uint8_t _health = 0;

static LittleFS_QPINAND _flash;

// ─── Flash ────────────────────────────────────────────────────────────────────

static bool flash_init() {
    if (!_flash.begin()) {
        LOG_ERROR("Storage: QSPI flash mount failed");
        return false;
    }

    File f = _flash.open("THIS_IS_APEX_FLASH.txt", FILE_WRITE);
    if (!f) {
        LOG_ERROR("Storage: QSPI flash write test failed");
        return false;
    }
    f.println("Apex QSPI NAND Flash (128 MB, soldered)");
    f.close();

    uint32_t total = _flash.totalSize();
    uint32_t used  = _flash.usedSize();
    LOG_INFO("Storage: QSPI flash OK — %lu KB total, %lu KB used",
             total / 1024, used / 1024);
    return true;
}

// ─── SD ───────────────────────────────────────────────────────────────────────

static bool sd_init() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        LOG_ERROR("Storage: SD card mount failed — card inserted?");
        return false;
    }

    File f = SD.open("THIS_IS_APEX_SD.txt", FILE_WRITE);
    if (!f) {
        LOG_ERROR("Storage: SD write test failed");
        return false;
    }
    f.println("Apex MicroSD Card (64 GB, removable)");
    f.close();

    uint64_t total = SD.totalSize();
    uint64_t used  = SD.usedSize();
    LOG_INFO("Storage: SD card OK — %llu MB total, %llu MB used",
             total / (1024 * 1024), used / (1024 * 1024));
    return true;
}

// ─── Public ───────────────────────────────────────────────────────────────────

uint8_t storage_init() {
    _health = 0;
    if (flash_init()) _health |= STORAGE_OK_FLASH;
    if (sd_init())    _health |= STORAGE_OK_SD;

    if (_health == 0)
        LOG_ERROR("Storage: both flash and SD failed — no logging available");
    else if (_health != (STORAGE_OK_FLASH | STORAGE_OK_SD))
        LOG_WARN("Storage: running on single medium (health=0x%02X)", _health);

    // Register available volumes with MTP so they appear as drives over USB.
    // mtp_loop() must be called from the main loop to service MTP transfers.
    MTP.begin();
    if (_health & STORAGE_OK_FLASH) MTP.addFilesystem(_flash, "APEX-FLASH");
    if (_health & STORAGE_OK_SD)    MTP.addFilesystem(SD,     "APEX-SD");

    return _health;
}

uint8_t storage_health() { return _health; }

void storage_mtp_loop() {
    MTP.loop();
}

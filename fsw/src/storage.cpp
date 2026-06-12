#include "storage.h"
#include "config.h"
#include "debug.h"
#include "flight_state.h"
#include "gps.h"
#include "sensors.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <SD.h>
#include <string.h>

// MTP (drag-and-drop log download over USB) only exists when the USB type
// includes it. HIL currently uses plain USB serial, so HIL logs are written by
// the same code path but MTP offload is only present in MTP-enabled builds.
#ifdef USB_MTPDISK_SERIAL
#include <MTP_Teensy.h>
#endif

// ─── Binary record layout ────────────────────────────────────────────────────

struct __attribute__((packed)) LogHeader {
    uint32_t magic;
    uint8_t  version;
    uint8_t  type;
    uint16_t length;
    uint32_t seq;
    uint32_t boot_id;
    uint32_t flight_id;
    uint32_t time_ms;
    uint16_t crc;
};

struct __attribute__((packed)) BootPayload {
    uint32_t build_flags;
    uint32_t config_hash;
    uint32_t log_header_size;
    uint32_t boot_payload_size;
    uint32_t event_payload_size;
    uint32_t sample_payload_size;
    uint32_t rate_fusion_hz;
    uint32_t rate_state_hz;
    uint32_t rate_control_hz;
    uint32_t target_apogee_cm;
    uint32_t log_flight_hz;
    uint8_t  storage_health;
    uint8_t  reserved[3];
};

struct __attribute__((packed)) EventPayload {
    uint8_t event_id;
    uint8_t phase;
    uint8_t storage_health;
    uint8_t gps_fix;
    uint16_t storage_faults;
    char detail[48];
};

struct __attribute__((packed)) SamplePayload {
    uint32_t sample_ms;
    uint8_t  phase;
    uint8_t  storage_health;
    uint8_t  gps_fix;
    uint8_t  gps_sats;
    uint16_t storage_faults;
    uint8_t  control_active;
    uint8_t  utc_valid;
    uint16_t utc_year;
    uint8_t  utc_month;
    uint8_t  utc_day;
    uint8_t  utc_hour;
    uint8_t  utc_minute;
    uint8_t  utc_second;
    uint16_t utc_ms;
    uint8_t  reserved;
    float accel_x_mss;
    float accel_y_mss;
    float accel_z_mss;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;
    float highg_x_mss;
    float baro_pa;
    float baro_temp_c;
    float alt_agl_m;
    float velocity_mps;
    float pred_apogee_m;
    float vert_accel_mps2;
    float deployment_frac;
    float pid_error_m;
    float pid_p_term;
    float pid_i_term;
    float pid_d_term;
    float gps_lat_deg;
    float gps_lon_deg;
    float gps_alt_msl_m;
};

static_assert(sizeof(LogHeader) == 26, "LogHeader wire size changed");
static_assert(sizeof(SamplePayload) <= 128, "SamplePayload ring budget changed");

// ─── Module state ────────────────────────────────────────────────────────────

static uint8_t  _health = 0;
static uint16_t _faults = LOG_FAULT_NONE;

static LittleFS_QPINAND _flash;
static File _flash_log;
static File _sd_log;

static uint32_t _boot_id = 0;
static uint32_t _next_flight_id = 1;
static uint32_t _flight_id = 0;
static uint32_t _seq = 0;

static bool _flight_started = false;
static bool _ring_flushed = false;
static uint32_t _last_ring_ms = 0;
static uint32_t _last_file_ms = 0;
static uint32_t _last_flush_ms = 0;

#define LOG_DIR "APEX"
#define LOG_NEXT_BOOT_PATH   LOG_DIR "/NEXTBOOT.TXT"
#define LOG_NEXT_FLIGHT_PATH LOG_DIR "/NEXTFLT.TXT"

static constexpr uint16_t RING_RECORDS =
    LOG_RING_BUF_SECONDS * LOG_PRELAUNCH_RING_HZ;

// Large prelaunch buffer lives in RAM2 so it does not consume tightly-coupled
// RAM1 stack/local-variable headroom needed by the flight code.
DMAMEM SamplePayload _ring[RING_RECORDS];
static uint16_t _ring_head = 0;
static uint16_t _ring_count = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t* data, size_t len) {
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static uint32_t fnv1a_update_u32(uint32_t hash, uint32_t value) {
    for (uint8_t i = 0; i < 4; i++) {
        hash ^= (uint8_t)(value >> (i * 8));
        hash *= 16777619UL;
    }
    return hash;
}

static uint32_t config_hash() {
    uint32_t h = 2166136261UL;
    const uint32_t values[] = {
        RATE_FUSION_HZ, RATE_STATE_HZ, RATE_CONTROL_HZ,
        RATE_BARO_HZ, RATE_MAG_HZ,
        (uint32_t)(TARGET_APOGEE_M * 100.0f + 0.5f),
        (uint32_t)(LAUNCH_ACCEL_THRESH_MSS * 100.0f + 0.5f),
        LAUNCH_CONFIRM_MS,
        (uint32_t)(LAUNCH_BARO_BACKUP_M * 100.0f + 0.5f),
        LAUNCH_BARO_CONFIRM_MS,
        BURNOUT_CONFIRM_MS, BOOST_MAX_MS,
        POST_BURNOUT_LOCKOUT_MS,
        (uint32_t)(MACH_GATE_MPS * 100.0f + 0.5f),
        (uint32_t)(MIN_DEPLOY_ALT_M * 100.0f + 0.5f),
        (uint32_t)(APOGEE_VEL_THRESH_MPS * 100.0f + 0.5f),
        APOGEE_CONFIRM_MS, APOGEE_BACKUP_LOCKOUT_MS,
        (uint32_t)(APOGEE_BARO_FALL_M * 100.0f + 0.5f),
        LOG_RING_BUF_SECONDS, LOG_PRELAUNCH_RING_HZ,
        LOG_PAD_FILE_HZ, LOG_FLIGHT_FILE_HZ, LOG_RATE_DESCENT_HZ,
    };
    for (uint32_t value : values) h = fnv1a_update_u32(h, value);
    return h;
}

static int8_t current_gps_fix() {
    if (g_state.gps.valid) return (int8_t)g_state.gps.fix_quality;
    return gps_fix_state();
}

static uint32_t read_counter_flash(const char* path, uint32_t fallback) {
    File f = _flash.open(path, FILE_READ);
    if (!f) return fallback;
    char buf[16] = {0};
    size_t n = f.readBytes(buf, sizeof(buf) - 1);
    f.close();
    if (n == 0) return fallback;
    uint32_t value = strtoul(buf, nullptr, 10);
    return value == 0 ? fallback : value;
}

static void write_counter_flash(const char* path, uint32_t value) {
    _flash.remove(path);
    File f = _flash.open(path, FILE_WRITE);
    if (!f) {
        _faults |= LOG_FAULT_FILE_OPEN;
        return;
    }
    f.printf("%lu\n", (unsigned long)value);
    f.close();
}

static void mirror_counter_sd(const char* path, uint32_t value) {
    if (!(_health & STORAGE_OK_SD)) return;
    SD.remove(path);
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        _faults |= LOG_FAULT_FILE_OPEN;
        return;
    }
    f.printf("%lu\n", (unsigned long)value);
    f.close();
}

static void open_boot_logs() {
    char path[40];
    snprintf(path, sizeof(path), LOG_DIR "/BOOT_%05lu.APXLOG",
             (unsigned long)_boot_id);

    _flash_log = _flash.open(path, FILE_WRITE);
    if (!_flash_log) _faults |= LOG_FAULT_FILE_OPEN;

    if (_health & STORAGE_OK_SD) {
        _sd_log = SD.open(path, FILE_WRITE);
        if (!_sd_log) _faults |= LOG_FAULT_FILE_OPEN;
    }
}

static void mark_write_fault(uint16_t fault, uint32_t now_ms) {
    bool first = (_faults & fault) == 0;
    _faults |= fault;
    if (first) {
        (void)now_ms;
        LOG_ERROR("Storage logging fault 0x%04X", _faults);
    }
}

static bool write_raw(const uint8_t* data, size_t len, uint32_t now_ms) {
    bool ok = true;
    if (_flash_log) {
        if (_flash_log.write(data, len) != len) {
            mark_write_fault(LOG_FAULT_FLASH_WRITE, now_ms);
            ok = false;
        }
    } else {
        mark_write_fault(LOG_FAULT_FILE_OPEN, now_ms);
        ok = false;
    }

    if (_sd_log) {
        if (_sd_log.write(data, len) != len) {
            mark_write_fault(LOG_FAULT_SD_WRITE, now_ms);
            ok = false;
        }
    } else {
        mark_write_fault(LOG_FAULT_FILE_OPEN, now_ms);
        ok = false;
    }
    return ok;
}

static bool write_record(uint8_t type, uint32_t time_ms,
                         const void* payload, uint16_t length) {
    if (length > 160) {
        _faults |= LOG_FAULT_RECORD_DROP;
        return false;
    }

    uint8_t frame[sizeof(LogHeader) + 160];
    LogHeader hdr = {};
    hdr.magic = APEX_LOG_MAGIC;
    hdr.version = APEX_LOG_VERSION;
    hdr.type = type;
    hdr.length = length;
    hdr.seq = _seq++;
    hdr.boot_id = _boot_id;
    hdr.flight_id = _flight_id;
    hdr.time_ms = time_ms;
    hdr.crc = 0;

    memcpy(frame, &hdr, sizeof(hdr));
    if (length > 0) memcpy(frame + sizeof(hdr), payload, length);

    uint16_t crc = 0xFFFF;
    crc = crc16_ccitt_update(crc, frame, sizeof(hdr));
    crc = crc16_ccitt_update(crc, frame + sizeof(hdr), length);
    ((LogHeader*)frame)->crc = crc;

    return write_raw(frame, sizeof(hdr) + length, time_ms);
}

static SamplePayload make_sample(uint32_t now_ms) {
    SamplePayload s = {};
    noInterrupts();
    s.sample_ms = now_ms;
    s.phase = (uint8_t)g_state.phase;
    s.accel_x_mss = g_state.imu.accel_x_mss;
    s.accel_y_mss = g_state.imu.accel_y_mss;
    s.accel_z_mss = g_state.imu.accel_z_mss;
    s.gyro_x_rads = g_state.imu.gyro_x_rads;
    s.gyro_y_rads = g_state.imu.gyro_y_rads;
    s.gyro_z_rads = g_state.imu.gyro_z_rads;
    s.highg_x_mss = g_state.high_g.accel_x_mss;
    s.baro_pa = g_state.baro.pressure_pa;
    s.baro_temp_c = g_state.baro.temperature_c;
    s.alt_agl_m = g_state.fused.altitude_agl_m;
    s.velocity_mps = g_state.fused.velocity_mps;
    s.pred_apogee_m = g_state.fused.predicted_apogee_m;
    s.vert_accel_mps2 = g_state.fused.accel_mps2;
    s.deployment_frac = g_state.control.deployment_frac;
    s.pid_error_m = g_state.control.pid_error_m;
    s.pid_p_term = g_state.control.pid_p_term;
    s.pid_i_term = g_state.control.pid_i_term;
    s.pid_d_term = g_state.control.pid_d_term;
    s.control_active = g_state.control.active ? 1 : 0;
    s.gps_sats = g_state.gps.satellites;
    s.gps_lat_deg = g_state.gps.lat_deg;
    s.gps_lon_deg = g_state.gps.lon_deg;
    s.gps_alt_msl_m = g_state.gps.altitude_msl_m;
    s.utc_valid = g_state.gps.time_valid ? 1 : 0;
    s.utc_year = g_state.gps.utc_year;
    s.utc_month = g_state.gps.utc_month;
    s.utc_day = g_state.gps.utc_day;
    s.utc_hour = g_state.gps.utc_hour;
    s.utc_minute = g_state.gps.utc_minute;
    s.utc_second = g_state.gps.utc_second;
    s.utc_ms = g_state.gps.utc_ms;
    interrupts();

    s.storage_health = _health;
    s.storage_faults = _faults;
    s.gps_fix = (uint8_t)current_gps_fix();
    return s;
}

static void ring_push(const SamplePayload& s) {
    _ring[_ring_head] = s;
    _ring_head = (_ring_head + 1) % RING_RECORDS;
    if (_ring_count < RING_RECORDS) _ring_count++;
}

static void flush_prelaunch_ring() {
    if (_ring_flushed) return;
    uint16_t start = (_ring_head + RING_RECORDS - _ring_count) % RING_RECORDS;
    for (uint16_t i = 0; i < _ring_count; i++) {
        const SamplePayload& s = _ring[(start + i) % RING_RECORDS];
        write_record(LOG_REC_SAMPLE, s.sample_ms, &s, sizeof(s));
    }
    _ring_flushed = true;
}

static void flush_logs(uint32_t now_ms, bool force) {
    if (!force && now_ms - _last_flush_ms < LOG_FILE_FLUSH_INTERVAL_MS) return;
    _last_flush_ms = now_ms;
    if (_flash_log) _flash_log.flush();
    if (_sd_log) _sd_log.flush();
}

// ─── Init ────────────────────────────────────────────────────────────────────

static bool flash_init() {
    if (!_flash.begin()) {
        LOG_ERROR("Storage: QSPI flash mount failed");
        return false;
    }
    _flash.mkdir(LOG_DIR);

    File f = _flash.open(LOG_DIR "/THIS_IS_APEX_FLASH.txt", FILE_WRITE);
    if (!f) {
        LOG_ERROR("Storage: QSPI flash write test failed");
        return false;
    }
    f.println("Apex QSPI NAND Flash (primary black-box log)");
    f.close();

    LOG_INFO("Storage: QSPI flash OK — %lu KB total, %lu KB used",
             _flash.totalSize() / 1024, _flash.usedSize() / 1024);
    return true;
}

static bool sd_init() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        LOG_ERROR("Storage: SD card mount failed — card inserted?");
        return false;
    }
    SD.mkdir(LOG_DIR);

    File f = SD.open(LOG_DIR "/THIS_IS_APEX_SD.txt", FILE_WRITE);
    if (!f) {
        LOG_ERROR("Storage: SD write test failed");
        return false;
    }
    f.println("Apex MicroSD Card (removable mirror log)");
    f.close();

    LOG_INFO("Storage: SD card OK — %llu MB total, %llu MB used",
             SD.totalSize() / (1024 * 1024), SD.usedSize() / (1024 * 1024));
    return true;
}

// ─── Public ──────────────────────────────────────────────────────────────────

uint8_t storage_init() {
    _health = 0;
    _faults = LOG_FAULT_NONE;
    _flight_started = false;
    _ring_flushed = false;
    _ring_head = 0;
    _ring_count = 0;

    if (flash_init()) _health |= STORAGE_OK_FLASH;
    if (sd_init())    _health |= STORAGE_OK_SD;

    if (_health & STORAGE_OK_FLASH) {
        _boot_id = read_counter_flash(LOG_NEXT_BOOT_PATH, 1);
        write_counter_flash(LOG_NEXT_BOOT_PATH, _boot_id + 1);
        mirror_counter_sd(LOG_NEXT_BOOT_PATH, _boot_id + 1);

        _next_flight_id = read_counter_flash(LOG_NEXT_FLIGHT_PATH, 1);
        mirror_counter_sd(LOG_NEXT_FLIGHT_PATH, _next_flight_id);
        open_boot_logs();
    } else {
        _faults |= LOG_FAULT_FILE_OPEN;
    }

    if (_health != (STORAGE_OK_FLASH | STORAGE_OK_SD)) {
        LOG_ERROR("Storage: launch logging unavailable (health=0x%02X)", _health);
    }

    BootPayload boot = {};
#ifdef APEX_HIL
    boot.build_flags |= 1UL << 0;
#endif
#ifdef APEX_DEBUG
    boot.build_flags |= 1UL << 1;
#endif
    boot.config_hash = config_hash();
    boot.log_header_size = sizeof(LogHeader);
    boot.boot_payload_size = sizeof(BootPayload);
    boot.event_payload_size = sizeof(EventPayload);
    boot.sample_payload_size = sizeof(SamplePayload);
    boot.rate_fusion_hz = RATE_FUSION_HZ;
    boot.rate_state_hz = RATE_STATE_HZ;
    boot.rate_control_hz = RATE_CONTROL_HZ;
    boot.target_apogee_cm = (uint32_t)(TARGET_APOGEE_M * 100.0f + 0.5f);
    boot.log_flight_hz = LOG_FLIGHT_FILE_HZ;
    boot.storage_health = _health;
    write_record(LOG_REC_BOOT, millis(), &boot, sizeof(boot));
    storage_log_event(LOG_EVENT_BOOT, storage_logging_ready() ? "storage ready" : "storage not ready");

    // Register available volumes with MTP so they appear as drives over USB.
#ifdef USB_MTPDISK_SERIAL
    MTP.begin();
    if (_health & STORAGE_OK_FLASH) MTP.addFilesystem(_flash, "APEX-FLASH");
    if (_health & STORAGE_OK_SD)    MTP.addFilesystem(SD,     "APEX-SD");
#endif

    return _health;
}

uint8_t storage_health() { return _health; }
uint16_t storage_faults() { return _faults; }
uint32_t storage_boot_id() { return _boot_id; }
uint32_t storage_flight_id() { return _flight_id; }

bool storage_logging_ready() {
    const uint8_t required = STORAGE_OK_FLASH | STORAGE_OK_SD;
    return ((_health & required) == required) && _flash_log && _sd_log && _faults == LOG_FAULT_NONE;
}

void storage_log_event(uint8_t event_id, const char* detail) {
    EventPayload e = {};
    e.event_id = event_id;
    e.phase = (uint8_t)g_state.phase;
    e.storage_health = _health;
    e.storage_faults = _faults;
    e.gps_fix = (uint8_t)current_gps_fix();
    if (detail != nullptr) {
        strncpy(e.detail, detail, sizeof(e.detail) - 1);
        e.detail[sizeof(e.detail) - 1] = '\0';
    }
    write_record(LOG_REC_EVENT, millis(), &e, sizeof(e));
}

void storage_begin_flight(uint32_t now_ms, const char* reason) {
    if (_flight_started) return;
    _flight_started = true;
    _flight_id = _next_flight_id++;
    write_counter_flash(LOG_NEXT_FLIGHT_PATH, _next_flight_id);
    mirror_counter_sd(LOG_NEXT_FLIGHT_PATH, _next_flight_id);
    storage_log_event(LOG_EVENT_LAUNCH_DETECTED, reason);
    flush_prelaunch_ring();
    flush_logs(now_ms, true);
}

void storage_log_update(uint32_t now_ms) {
    FlightPhase phase = g_state.phase;
    const bool prelaunch = (phase == FlightPhase::IDLE || phase == FlightPhase::ARMED);

    if (prelaunch && now_ms - _last_ring_ms >= 1000U / LOG_PRELAUNCH_RING_HZ) {
        _last_ring_ms = now_ms;
        ring_push(make_sample(now_ms));
    }

    uint32_t rate = LOG_PAD_FILE_HZ;
    if (phase == FlightPhase::BOOST || phase == FlightPhase::COAST) {
        rate = LOG_FLIGHT_FILE_HZ;
    } else if (phase == FlightPhase::DESCENT) {
        rate = LOG_RATE_DESCENT_HZ;
    }

    if (now_ms - _last_file_ms >= 1000U / rate) {
        _last_file_ms = now_ms;
        SamplePayload s = make_sample(now_ms);
        write_record(LOG_REC_SAMPLE, now_ms, &s, sizeof(s));
    }
    flush_logs(now_ms, false);
}

void storage_end_session(uint32_t now_ms, const char* reason) {
    storage_log_event(LOG_EVENT_HIL_SESSION_END, reason);
    flush_logs(now_ms, true);
    _flight_started = false;
    _flight_id = 0;
    _ring_flushed = false;
    _ring_head = 0;
    _ring_count = 0;
}

void storage_mtp_loop() {
#ifdef USB_MTPDISK_SERIAL
    MTP.loop();
#endif
}

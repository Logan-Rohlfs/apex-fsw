#include "storage.h"
#include "config.h"
#include "debug.h"
#include "flight_state.h"
#include "gps.h"
#include "sensors.h"

#include <Arduino.h>
#include <SD.h>
#include <string.h>

#ifdef USB_MTPDISK_SERIAL
#include <MTP_Teensy.h>
#endif

// SD-primary logger (the QSPI NAND died; the replacement APS6404L PSRAM is not
// detected, so microSD is the only non-volatile medium):
//  - IDLE/ARMED (pad) and DESCENT/LANDED  → buffered SD writes (stalls harmless).
//  - BOOST/COAST (ascent, timing-critical) → an OCRAM "black box" RAM buffer, so
//    an SD FAT stall can never delay burnout/apogee detection.
//  - At apogee + LOG_APOGEE_DUMP_DELAY_MS (under canopy) the RAM buffer is
//    dumped to SD in one blocking pass. Record/wire formats are unchanged, so
//    the ground decoder is unaffected — only *where* the bytes go changed.

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

#define LOG_DIR "APEX"
#define LOG_NEXT_BOOT_PATH   LOG_DIR "/NEXTBOOT.TXT"
#define LOG_NEXT_FLIGHT_PATH LOG_DIR "/NEXTFLT.TXT"

static constexpr size_t SD_BUFFER_BYTES = 8192;
static constexpr uint16_t RING_RECORDS =
    LOG_RING_BUF_SECONDS * LOG_PRELAUNCH_RING_HZ;

struct BufferedFileSink {
    File file;
    char path[40] = {0};
    uint8_t buf[SD_BUFFER_BYTES];
    size_t len = 0;
    uint32_t last_flush_ms = 0;
    uint8_t consecutive_failures = 0;
    bool suspended = false;
};

static BufferedFileSink _sd;
static char _log_path[40] = {0};

static uint8_t  _health = 0;
static uint16_t _faults = LOG_FAULT_NONE;
static uint32_t _boot_id = 0;
static uint32_t _next_flight_id = 1;
static uint32_t _flight_id = 0;
static uint32_t _seq = 0;

static bool _flight_started = false;
static bool _ring_flushed = false;
static bool _ascent_flushed = false;
static uint32_t _last_ring_ms = 0;
static uint32_t _last_sample_ms = 0;

// Prelaunch context ring (compact, last LOG_RING_BUF_SECONDS before launch).
DMAMEM SamplePayload _ring[RING_RECORDS];
static uint16_t _ring_head = 0;
static uint16_t _ring_count = 0;

// Ascent RAM "black box" — framed records for BOOST/COAST live here (no SD
// stalls), then dump to SD under canopy. OCRAM via DMAMEM; not zero-inited.
DMAMEM uint8_t _ascent_buf[LOG_ASCENT_BUF_BYTES];
static size_t _ascent_len = 0;
static bool _ascent_overflow = false;

#ifdef APEX_DEBUG
struct StorageDebugStats {
    uint32_t period_start_ms = 0;
    uint32_t samples_logged = 0;
    uint32_t events_logged = 0;
    uint32_t ring_pushes = 0;
    uint32_t ascent_bytes = 0;
    uint32_t sd_writes = 0;
    uint32_t sd_failures = 0;
    uint32_t max_sd_us = 0;
    uint32_t max_update_us = 0;
    uint16_t faults_seen = 0;
};

static StorageDebugStats _dbg_storage;

static void dbg_record_max(uint32_t& slot, uint32_t elapsed_us) {
    if (elapsed_us > slot) slot = elapsed_us;
}

static void dbg_storage_summary(uint32_t now_ms) {
    if (_dbg_storage.period_start_ms == 0) {
        _dbg_storage.period_start_ms = now_ms;
        _dbg_storage.faults_seen = _faults;
        return;
    }
    const bool due = now_ms - _dbg_storage.period_start_ms >= DEBUG_STORAGE_SUMMARY_MS;
    const bool slow =
        _dbg_storage.max_sd_us >= DEBUG_STORAGE_SLOW_US ||
        _dbg_storage.max_update_us >= DEBUG_STORAGE_SLOW_US;
    const bool fault_changed = _dbg_storage.faults_seen != _faults;
    const bool failure = _dbg_storage.sd_failures != 0;
    if (!due && !slow && !fault_changed && !failure) return;

    LOG_INFO("Storage dbg: phase=%s samples=%lu events=%lu ring=%lu "
             "ascent_buf=%lu/%u B sd=%lu/%lu max_us sd=%lu update=%lu faults=0x%04X",
             phase_name(g_state.phase),
             (unsigned long)_dbg_storage.samples_logged,
             (unsigned long)_dbg_storage.events_logged,
             (unsigned long)_dbg_storage.ring_pushes,
             (unsigned long)_ascent_len, (unsigned)sizeof(_ascent_buf),
             (unsigned long)_dbg_storage.sd_writes,
             (unsigned long)_dbg_storage.sd_failures,
             (unsigned long)_dbg_storage.max_sd_us,
             (unsigned long)_dbg_storage.max_update_us,
             _faults);

    _dbg_storage = {};
    _dbg_storage.period_start_ms = now_ms;
    _dbg_storage.faults_seen = _faults;
}
#endif

static void mark_fault(uint16_t fault, uint32_t now_ms) {
    const bool first = (_faults & fault) == 0;
    _faults |= fault;
    if (first) {
        (void)now_ms;
        LOG_ERROR("Storage fault 0x%04X", _faults);
    }
}

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
        LOG_ASCENT_BUF_BYTES, LOG_APOGEE_DUMP_DELAY_MS,
        LOG_SD_FLUSH_INTERVAL_MS, LOG_SD_MAX_FAULTS,
        LOG_SD_SLOW_DISABLE_US,
    };
    for (uint32_t value : values) h = fnv1a_update_u32(h, value);
    return h;
}

// ── Boot/flight counters (persisted on SD now that QSPI is gone) ───────────────
static uint32_t read_counter_sd(const char* path, uint32_t fallback) {
    File f = SD.open(path, FILE_READ);
    if (!f) return fallback;
    char buf[16] = {0};
    const size_t n = f.readBytes(buf, sizeof(buf) - 1);
    f.close();
    if (n == 0) return fallback;
    const uint32_t value = strtoul(buf, nullptr, 10);
    return value == 0 ? fallback : value;
}

static void write_counter_sd(const char* path, uint32_t value) {
    SD.remove(path);
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        mark_fault(LOG_FAULT_FILE_OPEN, millis());
        return;
    }
    f.printf("%lu\n", (unsigned long)value);
    f.close();
}

// ── SD sink ───────────────────────────────────────────────────────────────────
static void note_sd_result(bool ok, uint32_t elapsed_us, uint32_t now_ms) {
    const bool slow = elapsed_us >= LOG_SD_SLOW_DISABLE_US;
    if (ok && !slow) {
        _sd.consecutive_failures = 0;
        return;
    }
    if (_sd.consecutive_failures < UINT8_MAX) _sd.consecutive_failures++;
    mark_fault(LOG_FAULT_SD_WRITE, now_ms);
#ifdef APEX_DEBUG
    _dbg_storage.sd_failures++;
#endif
    if (!_sd.suspended &&
        (_sd.consecutive_failures >= LOG_SD_MAX_FAULTS || slow)) {
        _sd.suspended = true;
        LOG_ERROR("SD logging suspended (%s, elapsed_us=%lu)",
                  ok ? "slow write" : "write fault",
                  (unsigned long)elapsed_us);
    }
}

static bool sd_write_direct(const uint8_t* data, size_t len, uint32_t now_ms, bool flush) {
    if (!(_health & STORAGE_OK_SD) || !_sd.file || _sd.suspended) return false;
    const uint32_t start_us = micros();
    const bool ok = (_sd.file.write(data, len) == len);
    if (ok && flush) _sd.file.flush();
    const uint32_t elapsed_us = micros() - start_us;
    note_sd_result(ok, elapsed_us, now_ms);
#ifdef APEX_DEBUG
    if (ok) _dbg_storage.sd_writes++;
    dbg_record_max(_dbg_storage.max_sd_us, elapsed_us);
#endif
    return ok;
}

static bool sd_flush_buffer(uint32_t now_ms, bool force) {
    if (_sd.len == 0) return true;
    if (!force && now_ms - _sd.last_flush_ms < LOG_SD_FLUSH_INTERVAL_MS) return true;
    const bool ok = sd_write_direct(_sd.buf, _sd.len, now_ms, true);
    if (ok) {
        _sd.len = 0;
        _sd.last_flush_ms = now_ms;
    }
    return ok;
}

// SD writes are withheld during BOOST/COAST — a FAT stall there would delay
// burnout/apogee detection. Those records go to the ascent RAM buffer instead.
// If the RAM buffer overflows (ascent longer than budget), SD becomes the only
// path left, so allow it — degraded in-flight data beats none.
static bool sd_live_allowed() {
    if (_ascent_overflow) return true;
    const FlightPhase p = g_state.phase;
    return !(p == FlightPhase::BOOST || p == FlightPhase::COAST);
}

static void sd_buffer_record(const uint8_t* data, size_t len, uint32_t now_ms) {
    if (!(_health & STORAGE_OK_SD) || !_sd.file || _sd.suspended || !sd_live_allowed()) return;
    if (len > sizeof(_sd.buf)) return;
    if (_sd.len + len > sizeof(_sd.buf)) sd_flush_buffer(now_ms, true);
    if (_sd.len + len <= sizeof(_sd.buf)) {
        memcpy(_sd.buf + _sd.len, data, len);
        _sd.len += len;
    }
}

// ── Ascent RAM black box ──────────────────────────────────────────────────────
static inline bool ascent_phase(FlightPhase p) {
    return p == FlightPhase::BOOST || p == FlightPhase::COAST;
}

// Append a framed record to the RAM buffer. Returns false if it didn't fit (the
// caller then routes the record to SD-live as a degraded fallback).
static bool ascent_append(const uint8_t* frame, size_t len, uint32_t now_ms) {
    if (_ascent_overflow) return false;
    if (_ascent_len + len > sizeof(_ascent_buf)) {
        _ascent_overflow = true;
        mark_fault(LOG_FAULT_RECORD_DROP, now_ms);
        LOG_ERROR("Ascent RAM buffer full at %lu B — falling back to SD-live",
                  (unsigned long)_ascent_len);
        return false;
    }
    memcpy(_ascent_buf + _ascent_len, frame, len);
    _ascent_len += len;
#ifdef APEX_DEBUG
    _dbg_storage.ascent_bytes += len;
#endif
    return true;
}

static void flush_ascent_to_sd(uint32_t now_ms) {
    if (_ascent_flushed) return;
    _ascent_flushed = true;
    if (_ascent_len == 0) return;
    if (!(_health & STORAGE_OK_SD) || !_sd.file || _sd.suspended) return;

    // One blocking pass — we are under canopy, timing no longer matters.
    sd_flush_buffer(now_ms, true);            // commit any pending live bytes first
    size_t off = 0;
    while (off < _ascent_len) {
        size_t n = _ascent_len - off;
        if (n > SD_BUFFER_BYTES) n = SD_BUFFER_BYTES;
        if (!sd_write_direct(_ascent_buf + off, n, now_ms, false)) break;
        off += n;
    }
    if (_sd.file) _sd.file.flush();
    storage_log_event(LOG_EVENT_PHASE, "ascent ram dumped to sd");
    sd_flush_buffer(now_ms, true);
}

static int8_t current_gps_fix() {
    if (g_state.gps.valid) return (int8_t)g_state.gps.fix_quality;
    return gps_fix_state();
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

    const size_t frame_len = sizeof(hdr) + length;

    // Route: ascent (BOOST/COAST) → RAM black box; everything else → SD-live.
    // On RAM overflow the record falls through to SD so nothing is silently lost.
    bool ok = true;
    if (ascent_phase(g_state.phase) && !_ascent_overflow) {
        if (!ascent_append(frame, frame_len, time_ms)) {
            sd_buffer_record(frame, frame_len, time_ms);
        }
    } else {
        sd_buffer_record(frame, frame_len, time_ms);
    }
#ifdef APEX_DEBUG
    if (type == LOG_REC_SAMPLE) _dbg_storage.samples_logged++;
    if (type == LOG_REC_EVENT) _dbg_storage.events_logged++;
#endif
    return ok;
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
    const uint16_t start = (_ring_head + RING_RECORDS - _ring_count) % RING_RECORDS;
    for (uint16_t i = 0; i < _ring_count; i++) {
        const SamplePayload& s = _ring[(start + i) % RING_RECORDS];
        write_record(LOG_REC_SAMPLE, s.sample_ms, &s, sizeof(s));
    }
    _ring_flushed = true;
}

static void flush_logs(uint32_t now_ms, bool force) {
    sd_flush_buffer(now_ms, force);
}

static bool sd_init() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        LOG_ERROR("Storage: SD card mount failed — card inserted?");
        return false;
    }
    SD.mkdir(LOG_DIR);
    LOG_INFO("Storage: SD card OK — %llu MB total, %llu MB used",
             SD.totalSize() / (1024 * 1024), SD.usedSize() / (1024 * 1024));
    return true;
}

static void open_logs() {
    if (!(_health & STORAGE_OK_SD)) return;
    snprintf(_log_path, sizeof(_log_path), LOG_DIR "/BOOT_%05lu.APXLOG",
             (unsigned long)_boot_id);
    snprintf(_sd.path, sizeof(_sd.path), "%s", _log_path);
    _sd.file = SD.open(_sd.path, FILE_WRITE);
    if (!_sd.file) mark_fault(LOG_FAULT_FILE_OPEN, millis());
}

static void write_boot_record() {
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
    storage_log_event(LOG_EVENT_BOOT,
                      storage_logging_ready() ? "sd logging ready" : "storage not ready");
    flush_logs(millis(), true);
}

uint8_t storage_init() {
    _health = 0;
    _faults = LOG_FAULT_NONE;
    _boot_id = 0;
    _next_flight_id = 1;
    _flight_id = 0;
    _seq = 0;
    _flight_started = false;
    _ring_flushed = false;
    _ascent_flushed = false;
    _ring_head = 0;
    _ring_count = 0;
    _ascent_len = 0;
    _ascent_overflow = false;
    _sd = BufferedFileSink{};

    if (sd_init()) _health |= STORAGE_OK_SD;

    if (_health & STORAGE_OK_SD) {
        _boot_id = read_counter_sd(LOG_NEXT_BOOT_PATH, 1);
        _next_flight_id = read_counter_sd(LOG_NEXT_FLIGHT_PATH, 1);
        write_counter_sd(LOG_NEXT_BOOT_PATH, _boot_id + 1);
        open_logs();
    } else {
        mark_fault(LOG_FAULT_FILE_OPEN, millis());
        LOG_ERROR("Storage: SD unavailable — launch logging unavailable (health=0x%02X)",
                  _health);
    }

    write_boot_record();

#ifdef USB_MTPDISK_SERIAL
    MTP.begin();
    if (_health & STORAGE_OK_SD) MTP.addFilesystem(SD, "APEX-SD");
#endif

    return _health;
}

uint8_t storage_health() { return _health; }
uint16_t storage_faults() { return _faults; }
uint32_t storage_boot_id() { return _boot_id; }

bool storage_logging_ready() {
    // SD is the only non-volatile medium now, so it is the launch gate.
    return (_health & STORAGE_OK_SD) &&
           _sd.file &&
           !_sd.suspended &&
           ((_faults & LOG_FAULT_RECORD_DROP) == 0);
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
    write_counter_sd(LOG_NEXT_FLIGHT_PATH, _next_flight_id);
    storage_log_event(LOG_EVENT_LAUNCH_DETECTED, reason);
    flush_prelaunch_ring();
    flush_logs(now_ms, true);
}

void storage_log_update(uint32_t now_ms) {
#ifdef APEX_DEBUG
    const uint32_t update_start_us = micros();
#endif
    const FlightPhase phase = g_state.phase;
    const bool prelaunch = (phase == FlightPhase::IDLE || phase == FlightPhase::ARMED);

    if (prelaunch && now_ms - _last_ring_ms >= 1000U / LOG_PRELAUNCH_RING_HZ) {
        _last_ring_ms = now_ms;
        ring_push(make_sample(now_ms));
#ifdef APEX_DEBUG
        _dbg_storage.ring_pushes++;
#endif
    }

    uint32_t rate = LOG_PAD_FILE_HZ;
    if (phase == FlightPhase::BOOST || phase == FlightPhase::COAST) {
        rate = LOG_FLIGHT_FILE_HZ;
    } else if (phase == FlightPhase::DESCENT) {
        rate = LOG_RATE_DESCENT_HZ;
    }

    if (now_ms - _last_sample_ms >= 1000U / rate) {
        _last_sample_ms = now_ms;
        const SamplePayload s = make_sample(now_ms);
        write_record(LOG_REC_SAMPLE, now_ms, &s, sizeof(s));
    }

    // SD flushing only happens when SD-live is allowed (i.e. not mid-ascent).
    if (sd_live_allowed()) flush_logs(now_ms, false);

    // Dump the ascent RAM black box to SD a few seconds after apogee (under
    // canopy), with a landing fallback if DESCENT was skipped entirely.
    if (_flight_started && !_ascent_flushed && _ascent_len > 0) {
        const bool apogee_window =
            (phase == FlightPhase::DESCENT &&
             now_ms - g_state.phase_entry_ms >= LOG_APOGEE_DUMP_DELAY_MS);
        const bool landed =
            (phase == FlightPhase::LANDED &&
             now_ms - g_state.phase_entry_ms >= LOG_LANDING_DUMP_DELAY_MS);
        if (apogee_window || landed) flush_ascent_to_sd(now_ms);
    }
#ifdef APEX_DEBUG
    dbg_record_max(_dbg_storage.max_update_us, micros() - update_start_us);
    dbg_storage_summary(now_ms);
#endif
}

void storage_end_session(uint32_t now_ms, const char* reason) {
    storage_log_event(LOG_EVENT_HIL_SESSION_END, reason);
    flush_ascent_to_sd(now_ms);
    flush_logs(now_ms, true);
    _flight_started = false;
    _flight_id = 0;
    _ring_flushed = false;
    _ring_head = 0;
    _ring_count = 0;
    _ascent_flushed = false;
    _ascent_len = 0;
    _ascent_overflow = false;
}

bool storage_format_qspi(uint32_t now_ms) {
    // The QSPI NAND is physically gone (replaced by undetected PSRAM). There is
    // nothing to low-level format; SD logs are managed from the host instead.
    (void)now_ms;
    Serial.println("#WARN: FORMAT_QSPI ignored — no QSPI flash present (SD-only build). "
                   "Delete SD logs from HORIZON / the host file browser instead.");
    return false;
}

static void print_log_dir_entry(const char* medium, File& entry) {
    Serial.printf("#INFO: LOG_FILE medium=%s name=%s size=%llu dir=%d\n",
                  medium, entry.name(), (unsigned long long)entry.size(),
                  entry.isDirectory() ? 1 : 0);
}

static uint16_t print_log_dir(FS& fs, const char* medium) {
    File dir = fs.open(LOG_DIR, FILE_READ);
    if (!dir || !dir.isDirectory()) {
        Serial.printf("#WARN: LOG_LIST medium=%s path=%s unavailable\n",
                      medium, LOG_DIR);
        if (dir) dir.close();
        return 0;
    }

    uint16_t count = 0;
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break;
        print_log_dir_entry(medium, entry);
        if (!entry.isDirectory()) count++;
        entry.close();
    }
    dir.close();
    Serial.printf("#INFO: LOG_LIST medium=%s files=%u\n", medium, count);
    return count;
}

void storage_print_log_directory(uint32_t now_ms) {
    if (sd_live_allowed()) flush_logs(now_ms, true);
    Serial.printf("#INFO: LOG_STATUS health=0x%02X faults=0x%04X boot_id=%lu "
                  "flight_id=%lu ready=%d sd_open=%d ascent_buf=%lu/%u dumped=%d path=%s\n",
                  _health, _faults, (unsigned long)_boot_id,
                  (unsigned long)_flight_id, storage_logging_ready() ? 1 : 0,
                  _sd.file ? 1 : 0, (unsigned long)_ascent_len,
                  (unsigned)sizeof(_ascent_buf), _ascent_flushed ? 1 : 0, _log_path);

    if (_health & STORAGE_OK_SD) {
        print_log_dir(SD, "SD");
    } else {
        Serial.println("#WARN: LOG_LIST medium=SD unavailable (SD not healthy)");
    }
}

void storage_mtp_loop() {
#ifdef USB_MTPDISK_SERIAL
    MTP.loop();
#endif
}

#include "gps.h"
#include "flight_state.h"
#include "config.h"
#include "debug.h"

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

static SFE_UBLOX_GNSS _gnss;
static bool           _ready = false;

// -1 = offline (init failed)
//  0 = online, no fix
// 2+ = fix type from NAV-PVT
static int8_t _reported_fix = -1;

static void pps_isr() {
    g_state.gps.pps_micros = micros();
}

bool gps_init() {
    Wire2.begin();

    if (!_gnss.begin(Wire2)) {
        LOG_ERROR("GPS not found on I2C2 — check wiring to 0x42");
        _reported_fix = -1;
        return false;
    }

    // Read module firmware version — confirms UBX command/response works,
    // not just that the I2C address ACKs.
    if (_gnss.getModuleInfo()) {
        LOG_INFO("GPS module: %s  FW=%u.%u  protocol=%u.%u",
            _gnss.getModuleName(),
            _gnss.getFirmwareVersionHigh(), _gnss.getFirmwareVersionLow(),
            _gnss.getProtocolVersionHigh(), _gnss.getProtocolVersionLow());
    } else {
        LOG_WARN("GPS found on I2C but UBX MON-VER failed — check module");
    }

    // UBX protocol only — disable NMEA to reduce bus traffic
    _gnss.setI2COutput(COM_TYPE_UBX);

    // 10 Hz navigation updates
    _gnss.setNavigationFrequency(10);

    // Auto-request NAV-PVT so getPVT() returns fresh data each poll
    _gnss.setAutoPVT(true, VAL_LAYER_RAM);

    // Persist I2C + rate settings across power cycles
    _gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_NAVCONF);

    // PPS interrupt — fires once per UTC second once locked
    pinMode(PIN_GPS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), pps_isr, RISING);

    _ready        = true;
    _reported_fix = 0;   // online, searching
    LOG_INFO("GPS configured — searching for satellites");
    return true;
}

int8_t gps_fix_state() {
    return _reported_fix;
}

void gps_update() {
    if (!_ready) return;

    if (!_gnss.getPVT()) return;

    GpsData& gps = g_state.gps;

    gps.fix_quality = _gnss.getFixType();
    gps.satellites  = _gnss.getSIV();
    gps.valid       = (gps.fix_quality >= 3);
    _reported_fix   = (int8_t)gps.fix_quality;

    gps.lat_deg        = _gnss.getLatitude()    * 1e-7f;
    gps.lon_deg        = _gnss.getLongitude()   * 1e-7f;
    gps.altitude_msl_m = _gnss.getAltitudeMSL() * 1e-3f;
    gps.speed_mps      = _gnss.getGroundSpeed() * 1e-3f;

    gps.time_valid = _gnss.getTimeValid() && _gnss.getDateValid();
    if (gps.time_valid) {
        gps.utc_year   = _gnss.getYear();
        gps.utc_month  = _gnss.getMonth();
        gps.utc_day    = _gnss.getDay();
        gps.utc_hour   = _gnss.getHour();
        gps.utc_minute = _gnss.getMinute();
        gps.utc_second = _gnss.getSecond();
        gps.utc_ms     = _gnss.getMillisecond();
    }

    gps.timestamp_ms = millis();

#ifdef APEX_DEBUG
    static uint8_t log_div = 0;
    if (++log_div >= 10) {
        log_div = 0;
        if (gps.time_valid)
            LOG_INFO("GPS fix=%d sats=%d UTC=%04u-%02u-%02uT%02u:%02u:%02u",
                gps.fix_quality, gps.satellites,
                gps.utc_year, gps.utc_month, gps.utc_day,
                gps.utc_hour, gps.utc_minute, gps.utc_second);
        else
            LOG_INFO("GPS fix=%d sats=%d", gps.fix_quality, gps.satellites);
    }
#endif
}

void gps_utc_string(char* buf, size_t len) {
    const GpsData& gps = g_state.gps;
    if (!gps.time_valid) {
        snprintf(buf, len, "NO TIME");
        return;
    }
    snprintf(buf, len, "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
        gps.utc_year, gps.utc_month, gps.utc_day,
        gps.utc_hour, gps.utc_minute, gps.utc_second, gps.utc_ms);
}

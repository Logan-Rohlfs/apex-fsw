#include "gps.h"
#include "flight_state.h"
#include "config.h"
#include "debug.h"
#include "storage.h"

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <math.h>

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
    Wire2.setClock(GPS_I2C_CLOCK_HZ);

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

    bool config_ok = true;

    // UBX protocol only — disable NMEA to reduce bus traffic
    config_ok &= _gnss.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR);

    // Rocket flight is far outside the default portable/pedestrian filters.
    // Airborne4g is the most permissive u-blox platform model available here;
    // GPS can still drop during >4g boost, so it must remain non-critical.
    config_ok &= _gnss.setDynamicModel(GPS_DYNAMIC_MODEL, VAL_LAYER_RAM_BBR);

    // 10 Hz navigation updates
    config_ok &= _gnss.setNavigationFrequency(GPS_NAV_RATE_HZ, VAL_LAYER_RAM_BBR);

    // Auto-request NAV-PVT so getPVT() returns fresh data each poll
    config_ok &= _gnss.setAutoPVT(true, VAL_LAYER_RAM);

    // Persist I2C + rate settings across power cycles
    config_ok &= _gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_NAVCONF);

    uint8_t dynamic_model = _gnss.getDynamicModel(VAL_LAYER_RAM);
    if (dynamic_model == DYN_MODEL_UNKNOWN) {
        LOG_WARN("GPS dynamic model readback failed");
    } else if (dynamic_model != GPS_DYNAMIC_MODEL) {
        LOG_WARN("GPS dynamic model mismatch: got %u expected %u",
                 dynamic_model, (unsigned)GPS_DYNAMIC_MODEL);
        config_ok = false;
    }

    // PPS interrupt — fires once per UTC second once locked
    pinMode(PIN_GPS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), pps_isr, RISING);

    _ready        = true;
    _reported_fix = 0;   // online, searching
    if (config_ok) {
        LOG_INFO("GPS configured — %u Hz, dynamic model %u, searching for satellites",
                 GPS_NAV_RATE_HZ, (unsigned)GPS_DYNAMIC_MODEL);
    } else {
        LOG_WARN("GPS online, but one or more configuration writes failed");
    }
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

    // Brief interrupt guard: the 200 Hz fusion ISR reads multi-byte GPS
    // fields (altitude for the baro-dead fallback) — prevent torn reads.
    noInterrupts();
    gps.lat_deg        = _gnss.getLatitude()    * 1e-7f;
    gps.lon_deg        = _gnss.getLongitude()   * 1e-7f;
    gps.altitude_msl_m = _gnss.getAltitudeMSL() * 1e-3f;
    gps.speed_mps      = _gnss.getGroundSpeed() * 1e-3f;
    gps.timestamp_ms   = millis();
    interrupts();

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

#ifdef APEX_DEBUG
    static int8_t   last_logged_fix = -2;
    static uint8_t  last_logged_sats = 0xFF;
    static uint32_t last_summary_ms = 0;

    bool fix_changed = (gps.fix_quality != last_logged_fix);
    bool sats_changed = (gps.satellites != last_logged_sats);
    bool periodic_summary = (millis() - last_summary_ms >= 30000);

    if (fix_changed || periodic_summary) {
        last_logged_fix = (int8_t)gps.fix_quality;
        last_logged_sats = gps.satellites;
        last_summary_ms = millis();
        if (gps.time_valid)
            LOG_INFO("GPS fix=%d sats=%d UTC=%04u-%02u-%02uT%02u:%02u:%02u",
                gps.fix_quality, gps.satellites,
                gps.utc_year, gps.utc_month, gps.utc_day,
                gps.utc_hour, gps.utc_minute, gps.utc_second);
        else
            LOG_INFO("GPS fix=%d sats=%d", gps.fix_quality, gps.satellites);
    } else if (sats_changed && gps.fix_quality >= 3) {
        last_logged_sats = gps.satellites;
    }
#endif
}

// ─── Fix trust model ──────────────────────────────────────────────────────────
// Hardware-independent: consumes only g_state.gps, so the same logic runs in
// HIL (sensors_inject_hil writes g_state.gps; the HIL loop calls
// gps_monitor_update directly). Reference behaviour the emulator models:
// AIRBORNE4g loses the fix above 4 g of dynamics — every boost.

static bool     _trusted        = false;
static bool     _had_fix        = false;
static uint8_t  _good_epochs    = 0;
static uint32_t _last_epoch_ms  = 0;     // last counted solution timestamp
static bool     _sanity_warned  = false;

bool gps_trusted() { return _trusted; }

void gps_monitor_update(uint32_t now_ms) {
    GpsData& gps = g_state.gps;

    // Staleness: a frozen module (boost blackout, I2C upset) keeps its last
    // coordinates in g_state — do not present them as a live fix.
    const bool fresh = (gps.timestamp_ms != 0) &&
                       (now_ms - gps.timestamp_ms <= GPS_STALE_MS);
    if (!fresh && gps.valid) {
        gps.valid = false;
        gps.fix_quality = 0;
    }
    const bool fix = gps.valid && fresh;

    if (!fix) {
        if (_had_fix) {
            // Loss during BOOST is expected physics (>4 g exceeds the
            // AIRBORNE4g tracking envelope), not a fault.
            if (g_state.phase == FlightPhase::BOOST) {
                LOG_INFO("GPS fix lost (boost — expected above 4 g)");
                storage_log_event(LOG_EVENT_GPS_FIX_LOST, "boost, expected >4g");
            } else {
                LOG_WARN("GPS fix lost");
                storage_log_event(LOG_EVENT_GPS_FIX_LOST, "fix lost");
            }
        }
        _had_fix     = false;
        _trusted     = false;
        _good_epochs = 0;
        return;
    }

    // Reacquisition confirmation: count distinct solution epochs (10 Hz),
    // not calls — re-trust only after GPS_REACQUIRE_EPOCHS in a row.
    if (gps.timestamp_ms != _last_epoch_ms) {
        _last_epoch_ms = gps.timestamp_ms;
        if (_good_epochs < GPS_REACQUIRE_EPOCHS)
            _good_epochs++;
    }

    if (!_had_fix) {
        _had_fix = true;
        _good_epochs = 1;   // this epoch is the first of the streak
    }

    const bool was_trusted = _trusted;
    _trusted = (_good_epochs >= GPS_REACQUIRE_EPOCHS);
    if (_trusted && !was_trusted) {
        LOG_INFO("GPS fix trusted (%d sats)", gps.satellites);
        storage_log_event(LOG_EVENT_GPS_FIX_REGAINED, "fix trusted");
    }

    // Divergence diagnostic (does not affect trust — with baro dead, fused
    // altitude is the drifting one). Warn once per flight.
    if (_trusted && g_state.pad_pressure_pa > 0.0f && !_sanity_warned) {
        const float gps_agl = gps.altitude_msl_m - g_state.pad_altitude_msl_m;
        if (fabsf(gps_agl - g_state.fused.altitude_agl_m) > GPS_ALT_SANITY_M) {
            _sanity_warned = true;
            LOG_WARN("GPS/fused altitude diverge: gps %.0f m vs fused %.0f m AGL",
                     gps_agl, g_state.fused.altitude_agl_m);
        }
    }
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

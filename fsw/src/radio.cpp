#include "radio.h"
#include "config.h"
#include "debug.h"
#include "flight_state.h"
#include "sensors.h"
#include "gps.h"
#include "board.h"
#include "storage.h"

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

// Si4463 SPI commands used here
#define SI_PART_INFO     0x01
#define SI_POWER_UP      0x02
#define SI_FUNC_INFO     0x10
#define SI_READ_CMD_BUFF 0x44

static int8_t _status = -1;

// Si4463 frequency formula for 420-525 MHz:
//   fRF = (INTE + FRAC / 2^19) * 2 * XTAL / OUTDIV
// The Si4463 requires FRAC / 2^19 to be in [1, 2), so INTE is one less
// than the integer part of the scaled PLL ratio and FRAC carries the +1.x term.
#define RADIO_OUTDIV            8UL
#define RADIO_PLL_FRAC_SCALE    (1UL << 19)

// GFSK frames are software-framed (preamble/sync/CRC built here, not by the
// packet handler) so the SDR-side decoder fully defines the wire format:
//   0xAA x8 preamble, 0x2D 0xD4 sync, type byte, body, CRC-16-CCITT(type+body)
#define RADIO_GFSK_PREAMBLE_LEN 8U
#define RADIO_GFSK_REPEATS      10U
#define RADIO_GFSK_REPEAT_MS    100U

#define RADIO_FRAME_TYPE_TEST   0x01
#define RADIO_FRAME_TYPE_FLIGHT 0x02
#define RADIO_FRAME_TYPE_HK     0x03

// Zero-cost status packing. The low three phase bits still encode the six
// FlightPhase values; upper bits carry live operational state. The health byte
// similarly extends the existing four sensor bits with system health.
#define RADIO_PHASE_MASK              0x07
#define RADIO_STATUS_AIRBRAKES_OK     (1 << 3)
#define RADIO_STATUS_SERVO_POWER      (1 << 4)
#define RADIO_STATUS_ARM_SWITCHES     (1 << 5)
#define RADIO_STATUS_LOGGING_READY    (1 << 6)
#define RADIO_STATUS_GPS_TIME_VALID   (1 << 7)

#define RADIO_HEALTH_GPS              (1 << 4)
#define RADIO_HEALTH_RADIO            (1 << 5)
#define RADIO_HEALTH_QSPI             (1 << 6)
#define RADIO_HEALTH_SD               (1 << 7)

// Downlink bodies — little-endian, mirrored by scripts/radio_gfsk_rx.py.
// FLIGHT carries everything HORIZON needs to track plus the live-plot set,
// every beat. HOUSEKEEPING carries slow diagnostic channels once per second.
// Sensor channels are scaled int16 — classic telemetry practice, halves
// airtime vs f32 at resolutions far below sensor noise.

struct __attribute__((packed)) TelemFlight {
    char     callsign[6];   // "KG5LDI" — Part 97 ID in every frame
    uint16_t seq;
    uint8_t  phase_status;  // bits 0-2 phase; bits 3-7 operational flags
    uint8_t  health;        // bits 0-3 sensors; bits 4-7 GPS/radio/QSPI/SD
    int8_t   gps_fix;       // gps_fix_state(): -1 offline … 4 3D+DR
    uint8_t  gps_sats;
    float    gps_lat_deg;   // f32 — int scaling would cost position precision
    float    gps_lon_deg;
    int16_t  gps_alt_msl;   // 0.5 m     (±16383 m)
    int16_t  alt_agl;       // 0.1 m     (±3276 m)
    int16_t  velocity;      // 0.02 m/s  (±655 m/s)
    int16_t  pred_apogee;   // 0.1 m
    int16_t  vert_accel;    // 0.01 m/s² (±327)
    int16_t  accel_z;       // 0.01 m/s² — raw longitudinal IMU
    int16_t  roll_rate;     // 0.002 rad/s (gyro_z, ±65.5)
    uint8_t  deployment;    // airbrake 0..255 = 0..1
    uint16_t baro_pa;       // 2 Pa units (0–131070 Pa)
    int8_t   baro_temp;     // 1 °C
    int8_t   tilt_deg;      // 1°, 0..127 — angle off world-vertical
    uint16_t azimuth;       // 0.1° (0..3600) — compass direction of the tilt
};
static_assert(sizeof(TelemFlight) == 41, "flight body layout drifted");

struct __attribute__((packed)) TelemHousekeeping {
    uint16_t seq;           // shares the telemetry seq counter
    int16_t  mag[3];        // 1e-4 gauss
    int16_t  highg[3];      // 0.1 m/s² (±3276 — ADXL375 200 g = 1962)
    int16_t  gyro_xy[2];    // 0.002 rad/s — off-axis rates
    uint16_t uptime_s;
};
static_assert(sizeof(TelemHousekeeping) == 20, "housekeeping body layout drifted");

// Scale a float into a clamped int16 telemetry field.
static inline int16_t tlm_s16(float v, float scale) {
    const float s = v * scale;
    if (s >=  32767.0f) return  32767;
    if (s <= -32768.0f) return -32768;
    return (int16_t)lroundf(s);
}

static bool _gfsk_ready = false;   // modem configured for GFSK packet TX

// Telemetry TX statistics — reported over USB for the monitor's Link panel
static uint16_t _telem_seq     = 0;
static uint32_t _telem_sent    = 0;
static uint32_t _telem_skipped = 0;   // beats dropped because TX was still on air

// ─── Low-level SPI helpers ────────────────────────────────────────────────────

static inline void cs_low()  { digitalWrite(PIN_RAD_CS, LOW);  }
static inline void cs_high() { digitalWrite(PIN_RAD_CS, HIGH); }

// Poll READ_CMD_BUFF until the Si4463 returns CTS (0xFF as first byte).
// Leaves CS low and SPI transaction open so the caller can read response bytes.
// Returns false on timeout — indicates the chip isn't responding.
static bool wait_cts(uint16_t timeout_ms = 20) {
    uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        delayMicroseconds(100); // Si4463 requires min CS-high time between transactions
        cs_low();
        SPI1.transfer(SI_READ_CMD_BUFF);
        uint8_t cts = SPI1.transfer(0x00);
        if (cts == 0xFF) return true;   // CTS received, stay CS low
        cs_high();
    }
    cs_high();
    return false;
}

// Send a command buffer, then wait for CTS before returning.
// Leaves CS low so caller can immediately read response bytes if any.
static bool send_cmd(const uint8_t* buf, uint8_t len, uint16_t timeout_ms = 20) {
    cs_low();
    for (uint8_t i = 0; i < len; i++) SPI1.transfer(buf[i]);
    cs_high();
    return wait_cts(timeout_ms);
}

static uint8_t raw_cts_poll() {
    cs_low();
    SPI1.transfer(SI_READ_CMD_BUFF);
    uint8_t raw = SPI1.transfer(0x00);
    cs_high();
    return raw;
}

static bool power_up() {
    const uint8_t cmd[] = {
        SI_POWER_UP, 0x01, 0x00,
        (uint8_t)(RADIO_XTAL_HZ >> 24),
        (uint8_t)(RADIO_XTAL_HZ >> 16),
        (uint8_t)(RADIO_XTAL_HZ >>  8),
        (uint8_t)(RADIO_XTAL_HZ      )
    };
    return send_cmd(cmd, sizeof(cmd), 100);
}

static void radio_condition_sdo_pad() {
    cs_high();
    pinMode(PIN_RAD_MISO, INPUT_PULLUP);
    delay(5);

    pinMode(PIN_RAD_MISO, INPUT_PULLDOWN);
    delay(5);

    pinMode(PIN_RAD_MISO, INPUT);
}

static void calc_pll(uint32_t freq_hz, uint8_t& inte, uint32_t& frac) {
    uint32_t n_scaled = (uint32_t)(
        (((uint64_t)freq_hz * RADIO_OUTDIV * RADIO_PLL_FRAC_SCALE) + RADIO_XTAL_HZ) /
        (2ULL * RADIO_XTAL_HZ)
    );
    inte = (uint8_t)((n_scaled / RADIO_PLL_FRAC_SCALE) - 1UL);
    frac = n_scaled - ((uint32_t)inte * RADIO_PLL_FRAC_SCALE);
}

static bool radio_set_frequency(uint32_t freq_hz) {
    uint8_t inte;
    uint32_t frac;
    calc_pll(freq_hz, inte, frac);

    const uint8_t c[] = {
        0x11, 0x40, 0x04, 0x00,
        inte,
        (uint8_t)(frac >> 16),
        (uint8_t)(frac >> 8),
        (uint8_t)(frac)
    };
    LOG_INFO("Radio: PLL target=%lu Hz xtal=%lu Hz outdiv=%lu inte=0x%02X frac=0x%06lX",
             freq_hz, RADIO_XTAL_HZ, RADIO_OUTDIV, inte, (unsigned long)frac);
    return send_cmd(c, sizeof(c));
}

static bool radio_start_tx() {
    const uint8_t c[] = { 0x31, 0x00, 0x00, 0x00, 0x00 };
    return send_cmd(c, sizeof(c));
}

static bool radio_change_state_ready() {
    const uint8_t c[] = { 0x34, 0x03 };
    return send_cmd(c, sizeof(c));
}

static bool radio_configure_cw() {
    _gfsk_ready = false;   // CW marker reprograms the modem — GFSK must reconfigure
    // MODEM_CLKGEN_BAND (0x2051): OUTDIV=8 for 420-525 MHz, high-performance PLL.
    { const uint8_t c[] = { 0x11, 0x20, 0x01, 0x51, 0x0C }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // MODEM_MOD_TYPE (0x2000): CW — pure carrier, no modulation.
    { const uint8_t c[] = { 0x11, 0x20, 0x01, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // GPIO_PIN_CFG (0x13): GPIO2=RX_STATE (RXEN), GPIO3=TX_STATE (TXEN).
    // RF4463PRO routes these internal Si4463 GPIOs to its antenna switch.
    { const uint8_t c[] = { 0x13, 0x00, 0x00, 0x21, 0x20, 0x00, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // PA_MODE..PA_TC: configure the Si4463 PA block explicitly. Keep marker
    // power low on the bench so a nearby RTL-SDR does not overload.
    { const uint8_t c[] = { 0x11, 0x22, 0x04, 0x00, 0x08, RADIO_MARKER_PA_PWR, 0x00, 0x3D }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    return true;
}

// ─── 2-GFSK packet TX ─────────────────────────────────────────────────────────

static uint16_t crc16_ccitt(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// TX-only modem setup — the receiver is an SDR (or a second RF4463PRO with its
// own WDS config), so none of the Si4463 RX modem properties are needed here.
//   MODEM_DATA_RATE = bitrate * 10 with TXOSR=10x
//   MODEM_TX_NCO_MODE = xtal frequency (TXOSR field 0 = 10x)
//   MODEM_FREQ_DEV = (2^19 * outdiv * dev_hz) / (2 * xtal)   [peak, per datasheet]
static bool radio_configure_gfsk() {
    // GLOBAL_CONFIG (0x0000): FIFO_MODE=1 — unified 129-byte TX FIFO (we never
    // receive on this chip), needed for the 71-byte telemetry frame.
    { const uint8_t c[] = { 0x11, 0x00, 0x01, 0x00, 0x30 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // MODEM_CLKGEN_BAND (0x2051): OUTDIV=8 for 420-525 MHz, high-performance PLL.
    { const uint8_t c[] = { 0x11, 0x20, 0x01, 0x51, 0x0C }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // GPIO_PIN_CFG (0x13): GPIO2=RX_STATE (RXEN), GPIO3=TX_STATE (TXEN) — RF4463PRO antenna switch.
    { const uint8_t c[] = { 0x13, 0x00, 0x00, 0x21, 0x20, 0x00, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // PA_MODE..PA_TC — bench power, same as marker.
    { const uint8_t c[] = { 0x11, 0x22, 0x04, 0x00, 0x08, RADIO_MARKER_PA_PWR, 0x00, 0x3D }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // MODEM_MOD_TYPE (0x2000): 2GFSK, bits from the packet handler FIFO.
    { const uint8_t c[] = { 0x11, 0x20, 0x01, 0x00, 0x03 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // MODEM_DATA_RATE (0x2003, 3 bytes)
    {
        const uint32_t dr = RADIO_GFSK_BITRATE_BPS * 10UL;
        const uint8_t c[] = { 0x11, 0x20, 0x03, 0x03,
                              (uint8_t)(dr >> 16), (uint8_t)(dr >> 8), (uint8_t)dr };
        if (!send_cmd(c, sizeof(c))) return false; cs_high();
    }

    // MODEM_TX_NCO_MODE (0x2006, 4 bytes): TXOSR=10x, NCO = xtal.
    {
        const uint8_t c[] = { 0x11, 0x20, 0x04, 0x06,
                              (uint8_t)((RADIO_XTAL_HZ >> 24) & 0x03),
                              (uint8_t)(RADIO_XTAL_HZ >> 16),
                              (uint8_t)(RADIO_XTAL_HZ >> 8),
                              (uint8_t)RADIO_XTAL_HZ };
        if (!send_cmd(c, sizeof(c))) return false; cs_high();
    }

    // MODEM_FREQ_DEV (0x200A, 3 bytes)
    {
        const uint32_t dev = (uint32_t)(
            ((uint64_t)RADIO_PLL_FRAC_SCALE * RADIO_OUTDIV * RADIO_GFSK_DEV_HZ + RADIO_XTAL_HZ) /
            (2ULL * RADIO_XTAL_HZ));
        const uint8_t c[] = { 0x11, 0x20, 0x03, 0x0A,
                              (uint8_t)(dev >> 16), (uint8_t)(dev >> 8), (uint8_t)dev };
        if (!send_cmd(c, sizeof(c))) return false; cs_high();
    }

    // Packet handler: raw passthrough — frame bytes (preamble/sync/CRC included)
    // come straight from the FIFO with no hardware framing added.
    { const uint8_t c[] = { 0x11, 0x10, 0x01, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }   // PREAMBLE_TX_LENGTH = 0
    { const uint8_t c[] = { 0x11, 0x11, 0x01, 0x00, 0x80 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }   // SYNC_CONFIG: skip sync TX
    { const uint8_t c[] = { 0x11, 0x12, 0x01, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }   // PKT_CRC_CONFIG: none
    { const uint8_t c[] = { 0x11, 0x12, 0x02, 0x0E, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); } // FIELD_1_CONFIG / FIELD_1_CRC_CONFIG
    // PKT_FIELD_1_LENGTH is set per-frame in radio_tx_frame (lengths vary by type)

    _gfsk_ready = true;
    return true;
}

// Returns the Si4463 device state (low nibble of REQUEST_DEVICE_STATE), or
// 0xFF on CTS timeout. READY = 0x03, TX = 0x07.
static uint8_t radio_device_state() {
    const uint8_t c[] = { 0x33 };
    if (!send_cmd(c, sizeof(c))) return 0xFF;
    uint8_t state = SPI1.transfer(0x00);
    SPI1.transfer(0x00);   // current channel — unused
    cs_high();
    return state & 0x0F;
}

// Load one frame into the TX FIFO and start transmitting it. With wait=true,
// blocks until the chip returns to READY (~len * 8 / bitrate; 53 B ≈ 42 ms).
// With wait=false, returns as soon as TX starts (caller must check
// radio_device_state() == READY before the next frame).
static bool radio_tx_frame(const uint8_t* frame, uint8_t len, bool wait) {
    // PKT_FIELD_1_LENGTH — frame lengths vary by type
    {
        const uint8_t c[] = { 0x11, 0x12, 0x02, 0x0C, 0x00, len };
        if (!send_cmd(c, sizeof(c))) return false; cs_high();
    }

    // FIFO_INFO: reset TX FIFO
    { const uint8_t c[] = { 0x15, 0x01 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // WRITE_TX_FIFO
    cs_low();
    SPI1.transfer(0x66);
    for (uint8_t i = 0; i < len; i++) SPI1.transfer(frame[i]);
    cs_high();
    if (!wait_cts()) return false;
    cs_high();

    // START_TX: channel 0, return to READY when done, TX_LEN = len
    {
        const uint8_t c[] = { 0x31, 0x00, 0x30, 0x00, len };
        if (!send_cmd(c, sizeof(c))) return false; cs_high();
    }

    if (!wait) return true;

    // Poll REQUEST_DEVICE_STATE until READY (0x03)
    const uint32_t deadline = millis() + 500;
    while (millis() < deadline) {
        uint8_t state = radio_device_state();
        if (state == 0xFF) return false;
        if (state == 0x03) return true;
        delay(2);
    }
    LOG_ERROR("Radio: TX did not complete (state poll timeout)");
    return false;
}

// Assemble preamble + sync + type + body + CRC into out (must hold body_len + 13).
// Returns total frame length.
static uint8_t radio_build_frame(uint8_t* out, uint8_t type,
                                 const uint8_t* body, uint8_t body_len) {
    uint8_t* p = out;
    for (uint8_t i = 0; i < RADIO_GFSK_PREAMBLE_LEN; i++) *p++ = 0xAA;
    *p++ = 0x2D;
    *p++ = 0xD4;
    *p++ = type;
    memcpy(p, body, body_len);
    p += body_len;
    const uint16_t crc = crc16_ccitt(out + RADIO_GFSK_PREAMBLE_LEN + 2, 1 + body_len);
    *p++ = (uint8_t)(crc >> 8);
    *p++ = (uint8_t)crc;
    return (uint8_t)(p - out);
}

// ─── Init ─────────────────────────────────────────────────────────────────────

bool radio_init() {
    pinMode(PIN_RAD_CS,    OUTPUT);
    pinMode(PIN_RAD_INT1,  INPUT);
    pinMode(PIN_RAD_GPIO0, INPUT);
    pinMode(PIN_RAD_GPIO1, INPUT);
    digitalWrite(PIN_RAD_CS, HIGH);

    // RF4463PRO SDO/MISO can power up in a bad bus state on this board unless
    // the Teensy pad is biased before SPI1 takes ownership. This was confirmed
    // by removing the sequence and seeing POWER_UP CTS fail with raw=0x00.
    radio_condition_sdo_pad();

    SPI1.setMOSI(PIN_RAD_MOSI);
    SPI1.setMISO(PIN_RAD_MISO);
    SPI1.setSCK(PIN_RAD_SCK);
    SPI1.begin();
    SPI1.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0)); // 100 kHz — diagnose signal integrity

    // After POR or shutdown release, Si4463 requires POWER_UP before API reads.
    if (!power_up()) {
        uint8_t raw = raw_cts_poll();
        SPI1.endTransaction();
        LOG_ERROR("Radio: no CTS after POWER_UP (raw=0x%02X) — %s",
                  raw,
                  raw == 0x00 ? "no SPI access — check VCC, SDN low, nSEL/SCK/SDI/SDO solder joints" :
                  raw == 0xFF ? "MISO shorted high or stale response — power cycle" :
                                "unexpected — check SPI1 wiring");
        _status = -1;
        return false;
    }
    cs_high();

    // ── PART_INFO ─────────────────────────────────────────────────────────────
    // Response (after CTS byte): chipRev, part[1], part[0], pbuild,
    //                             id[1], id[0], customer, romId
    const uint8_t cmd_part[] = { SI_PART_INFO };
    if (!send_cmd(cmd_part, sizeof(cmd_part))) {
        // Read one raw CTS poll byte to distinguish floating MISO from a stuck chip
        uint8_t raw = raw_cts_poll();
        SPI1.endTransaction();
        LOG_ERROR("Radio: no CTS after PART_INFO (raw=0x%02X) — %s",
                  raw,
                  raw == 0x00 ? "chip busy but never ready — scope nSEL/SCK/SDI/SDO, check SDN and solder joints" :
                  raw == 0xFF ? "MISO shorted high or chip in bad state — power cycle" :
                                "unexpected — check SPI1 wiring");
        _status = -1;
        return false;
    }

    // Read 8 response bytes (CTS already consumed by wait_cts)
    uint8_t chip_rev = SPI1.transfer(0x00);
    uint8_t part_hi  = SPI1.transfer(0x00);
    uint8_t part_lo  = SPI1.transfer(0x00);
    uint8_t pbuild   = SPI1.transfer(0x00);
    uint8_t id_hi    = SPI1.transfer(0x00);
    uint8_t id_lo    = SPI1.transfer(0x00);
    uint8_t customer = SPI1.transfer(0x00);
    uint8_t rom_id   = SPI1.transfer(0x00);
    cs_high();
    (void)pbuild;
    (void)id_hi;
    (void)id_lo;
    (void)customer;

    uint16_t part_id = ((uint16_t)part_hi << 8) | part_lo;

    if (part_id != 0x4463) {
        SPI1.endTransaction();
        LOG_ERROR("Radio: wrong part ID 0x%04X (expected 0x4463) — wrong device or SPI wiring issue",
                  part_id);
        _status = -1;
        return false;
    }


    // ── FUNC_INFO — firmware version ──────────────────────────────────────────
    const uint8_t cmd_func[] = { SI_FUNC_INFO };
    if (send_cmd(cmd_func, sizeof(cmd_func))) {
        uint8_t rev_ext   = SPI1.transfer(0x00);
        uint8_t rev_branch= SPI1.transfer(0x00);
        uint8_t rev_int   = SPI1.transfer(0x00);
        uint8_t patch_hi  = SPI1.transfer(0x00);
        uint8_t patch_lo  = SPI1.transfer(0x00);
        uint8_t func      = SPI1.transfer(0x00);
        cs_high();
        LOG_INFO("Radio Si4463 OK — part=0x%04X rev=%u.%u.%u patch=%u func=%u romId=0x%02X",
                 part_id, rev_ext, rev_branch, rev_int,
                 (uint16_t)(patch_hi << 8 | patch_lo), func, rom_id);
    } else {
        cs_high();
        LOG_INFO("Radio Si4463 OK — part=0x%04X chipRev=0x%02X (FUNC_INFO timeout)",
                 part_id, chip_rev);
    }

    SPI1.endTransaction();
    _status = 0;
    return true;
}

int8_t radio_status() {
    return _status;
}

bool radio_marker_tx(uint32_t freq_hz) {
    if (_status < 0) {
        LOG_ERROR("Radio: marker skipped — chip not verified (run radio_init first)");
        return false;
    }

    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    if (!radio_configure_cw()) {
        cs_high();
        SPI1.endTransaction();
        return false;
    }
    if (!radio_set_frequency(freq_hz)) {
        cs_high();
        SPI1.endTransaction();
        return false;
    }
    cs_high();

    LOG_INFO("Radio marker: %.3f MHz, PA=0x%02X, 1s ON / 1s OFF, 5 cycles",
             freq_hz / 1e6f, RADIO_MARKER_PA_PWR);

    for (uint8_t i = 1; i <= 5; i++) {
        if (!radio_start_tx()) {
            cs_high();
            SPI1.endTransaction();
            return false;
        }
        cs_high();
        LOG_INFO("Radio marker: ON %u/5", i);
        delay(1000);

        if (!radio_change_state_ready()) {
            cs_high();
            SPI1.endTransaction();
            return false;
        }
        cs_high();
        LOG_INFO("Radio marker: OFF %u/5", i);
        delay(1000);
    }

    SPI1.endTransaction();
    LOG_INFO("Radio marker: done");
    return true;
}

// Configure GFSK + frequency if not already configured. Call inside an open
// SPI transaction. Leaves CS high.
static bool radio_ensure_gfsk() {
    if (_gfsk_ready) return true;
    if (!radio_configure_gfsk() || !radio_set_frequency(RADIO_FREQ_HZ)) {
        cs_high();
        _gfsk_ready = false;
        return false;
    }
    cs_high();
    return true;
}

bool radio_data_test_tx() {
    static const char payload[] = "APEX RADIO TEST";
    constexpr uint8_t body_len = 1 + sizeof(payload) - 1;   // seq + payload
    constexpr uint8_t frame_len = RADIO_GFSK_PREAMBLE_LEN + 2 + 1 + body_len + 2;
    static_assert(frame_len <= 64, "GFSK test frame must fit the TX FIFO");

    if (_status < 0) {
        LOG_ERROR("Radio: data test skipped — chip not verified (run radio_init first)");
        return false;
    }

    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    if (!radio_ensure_gfsk()) {
        SPI1.endTransaction();
        LOG_ERROR("Radio data test: GFSK config failed");
        return false;
    }

    LOG_INFO("Radio data test: 2GFSK %.3f MHz, %lu bps, dev=%lu Hz, PA=0x%02X, %u frames",
             RADIO_FREQ_HZ / 1e6f, RADIO_GFSK_BITRATE_BPS, RADIO_GFSK_DEV_HZ,
             RADIO_MARKER_PA_PWR, RADIO_GFSK_REPEATS);

    uint8_t body[body_len];
    memcpy(body + 1, payload, sizeof(payload) - 1);

    for (uint8_t seq = 1; seq <= RADIO_GFSK_REPEATS; seq++) {
        body[0] = seq;
        uint8_t frame[frame_len];
        radio_build_frame(frame, RADIO_FRAME_TYPE_TEST, body, body_len);

        if (!radio_tx_frame(frame, frame_len, true)) {
            cs_high();
            radio_change_state_ready();
            cs_high();
            SPI1.endTransaction();
            LOG_ERROR("Radio data test: failed at frame %u/%u", seq, RADIO_GFSK_REPEATS);
            return false;
        }
        LOG_INFO("Radio data test: frame %u/%u sent", seq, RADIO_GFSK_REPEATS);
        delay(RADIO_GFSK_REPEAT_MS);
    }

    SPI1.endTransaction();
    LOG_INFO("Radio data test: done");
    return true;
}

bool radio_telemetry_tx() {
    constexpr uint8_t flight_len = RADIO_GFSK_PREAMBLE_LEN + 2 + 1 + sizeof(TelemFlight) + 2;
    constexpr uint8_t hk_len     = RADIO_GFSK_PREAMBLE_LEN + 2 + 1 + sizeof(TelemHousekeeping) + 2;
    static_assert(flight_len <= 64 && hk_len <= 64, "telemetry frames must fit the TX FIFO");
    static uint32_t _last_hk_ms = 0;

    if (_status < 0) return false;

    // Radio silence (radio switch open): withhold TX entirely. The onboard
    // Si4463 stays initialized; firmware just never keys it.
    if (!board_radio_enabled()) return false;

    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    if (!radio_ensure_gfsk()) {
        SPI1.endTransaction();
        return false;
    }

    // Non-blocking: if the previous frame is still on air, skip this beat
    // rather than stall the loop or clobber the FIFO.
    if (radio_device_state() != 0x03) {
        SPI1.endTransaction();
        _telem_skipped++;
        return true;
    }

    uint8_t frame[64];
    uint8_t frame_len;

    // One beat per second carries housekeeping instead of flight data.
    if (millis() - _last_hk_ms >= 1000) {
        _last_hk_ms = millis();

        TelemHousekeeping hk;
        memset(&hk, 0, sizeof(hk));
        hk.seq        = _telem_seq++;
        hk.mag[0]     = tlm_s16(g_state.mag.x_gauss,      10000.0f);
        hk.mag[1]     = tlm_s16(g_state.mag.y_gauss,      10000.0f);
        hk.mag[2]     = tlm_s16(g_state.mag.z_gauss,      10000.0f);
        hk.highg[0]   = tlm_s16(g_state.high_g.accel_x_mss,  10.0f);
        hk.highg[1]   = tlm_s16(g_state.high_g.accel_y_mss,  10.0f);
        hk.highg[2]   = tlm_s16(g_state.high_g.accel_z_mss,  10.0f);
        hk.gyro_xy[0] = tlm_s16(g_state.imu.gyro_x_rads,    500.0f);
        hk.gyro_xy[1] = tlm_s16(g_state.imu.gyro_y_rads,    500.0f);
        hk.uptime_s   = (uint16_t)min(millis() / 1000UL, 65535UL);

        frame_len = radio_build_frame(frame, RADIO_FRAME_TYPE_HK,
                                      (const uint8_t*)&hk, sizeof(hk));
    } else {
        TelemFlight body;
        memset(&body, 0, sizeof(body));
        memcpy(body.callsign, RADIO_CALLSIGN, min(sizeof(body.callsign), strlen(RADIO_CALLSIGN)));
        body.seq         = _telem_seq++;
        body.phase_status = (uint8_t)g_state.phase & RADIO_PHASE_MASK;
        if (g_state.airbrakes_enabled) body.phase_status |= RADIO_STATUS_AIRBRAKES_OK;
        if (board_servo_powered())     body.phase_status |= RADIO_STATUS_SERVO_POWER;
        if (board_arm_switch_closed()) body.phase_status |= RADIO_STATUS_ARM_SWITCHES;
        if (storage_logging_ready())   body.phase_status |= RADIO_STATUS_LOGGING_READY;
        if (g_state.gps.time_valid)    body.phase_status |= RADIO_STATUS_GPS_TIME_VALID;

        body.health = sensors_health();
        if (gps_fix_state() >= 0)                body.health |= RADIO_HEALTH_GPS;
        if (_status >= 0)                        body.health |= RADIO_HEALTH_RADIO;
        if (storage_health() & STORAGE_OK_FLASH) body.health |= RADIO_HEALTH_QSPI;
        if (storage_health() & STORAGE_OK_SD)    body.health |= RADIO_HEALTH_SD;
        body.gps_fix     = gps_fix_state();
        body.gps_sats    = g_state.gps.satellites;
        body.gps_lat_deg = g_state.gps.lat_deg;
        body.gps_lon_deg = g_state.gps.lon_deg;
        body.gps_alt_msl = tlm_s16(g_state.gps.altitude_msl_m,       2.0f);
        body.alt_agl     = tlm_s16(g_state.fused.altitude_agl_m,    10.0f);
        body.velocity    = tlm_s16(g_state.fused.velocity_mps,      50.0f);
        body.pred_apogee = tlm_s16(g_state.fused.predicted_apogee_m, 10.0f);
        body.vert_accel  = tlm_s16(g_state.fused.accel_mps2,       100.0f);
        body.accel_z     = tlm_s16(g_state.imu.accel_z_mss,        100.0f);
        body.roll_rate   = tlm_s16(g_state.imu.gyro_z_rads,        500.0f);
        body.deployment  = (uint8_t)constrain(g_state.control.deployment_frac * 255.0f, 0.0f, 255.0f);
        body.baro_pa     = (uint16_t)constrain(g_state.baro.pressure_pa * 0.5f, 0.0f, 65535.0f);
        body.baro_temp   = (int8_t)constrain(g_state.baro.temperature_c, -128.0f, 127.0f);
        body.tilt_deg    = (int8_t)constrain(lroundf(g_state.fused.tilt_deg), 0, 127);
        body.azimuth     = (uint16_t)constrain(lroundf(g_state.fused.azimuth_deg * 10.0f), 0, 3600);

        frame_len = radio_build_frame(frame, RADIO_FRAME_TYPE_FLIGHT,
                                      (const uint8_t*)&body, sizeof(body));
    }

    bool ok = radio_tx_frame(frame, frame_len, false);
    cs_high();
    SPI1.endTransaction();
    if (ok) _telem_sent++;
    return ok;
}

void radio_telemetry_stats(uint16_t* seq, uint32_t* sent, uint32_t* skipped) {
    if (seq)     *seq     = _telem_seq;
    if (sent)    *sent    = _telem_sent;
    if (skipped) *skipped = _telem_skipped;
}

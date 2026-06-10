#include "radio.h"
#include "config.h"
#include "debug.h"

#include <Arduino.h>
#include <SPI.h>

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

// GFSK test frame: software-framed (preamble/sync/CRC built here, not by the
// packet handler) so the SDR-side decoder fully defines the wire format.
#define RADIO_GFSK_PREAMBLE_LEN 8U
#define RADIO_GFSK_REPEATS      10U
#define RADIO_GFSK_REPEAT_MS    100U

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
static bool radio_configure_gfsk(uint8_t frame_len) {
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
    {
        const uint8_t c[] = { 0x11, 0x12, 0x02, 0x0C, 0x00, frame_len };                                              // PKT_FIELD_1_LENGTH
        if (!send_cmd(c, sizeof(c))) return false; cs_high();
    }
    { const uint8_t c[] = { 0x11, 0x12, 0x02, 0x0E, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); } // FIELD_1_CONFIG / FIELD_1_CRC_CONFIG

    return true;
}

// Load one frame into the TX FIFO and transmit it, blocking until the chip
// returns to READY (~frame_len * 8 / bitrate seconds; 28 bytes ≈ 22 ms).
static bool radio_tx_frame(const uint8_t* frame, uint8_t len) {
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

    // Poll REQUEST_DEVICE_STATE until READY (0x03)
    const uint32_t deadline = millis() + 500;
    while (millis() < deadline) {
        const uint8_t c[] = { 0x33 };
        if (!send_cmd(c, sizeof(c))) return false;
        uint8_t state = SPI1.transfer(0x00);
        SPI1.transfer(0x00);   // current channel — unused
        cs_high();
        if ((state & 0x0F) == 0x03) return true;
        delay(2);
    }
    LOG_ERROR("Radio: TX did not complete (state poll timeout)");
    return false;
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

bool radio_data_test_tx() {
    static const char payload[] = "APEX RADIO TEST";
    constexpr uint8_t payload_len = sizeof(payload) - 1;

    // Frame: 0xAA preamble, 0x2D 0xD4 sync, seq byte, payload, CRC-16-CCITT
    // over seq+payload (big-endian). Must match scripts/radio_gfsk_rx.py.
    constexpr uint8_t frame_len = RADIO_GFSK_PREAMBLE_LEN + 2 + 1 + payload_len + 2;
    static_assert(frame_len <= 64, "GFSK test frame must fit the TX FIFO");

    if (_status < 0) {
        LOG_ERROR("Radio: data test skipped — chip not verified (run radio_init first)");
        return false;
    }

    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    if (!radio_configure_gfsk(frame_len) || !radio_set_frequency(RADIO_FREQ_HZ)) {
        cs_high();
        SPI1.endTransaction();
        LOG_ERROR("Radio data test: GFSK config failed");
        return false;
    }
    cs_high();

    LOG_INFO("Radio data test: 2GFSK %.3f MHz, %lu bps, dev=%lu Hz, PA=0x%02X, %u frames",
             RADIO_FREQ_HZ / 1e6f, RADIO_GFSK_BITRATE_BPS, RADIO_GFSK_DEV_HZ,
             RADIO_MARKER_PA_PWR, RADIO_GFSK_REPEATS);

    uint8_t frame[frame_len];
    uint8_t* p = frame;
    for (uint8_t i = 0; i < RADIO_GFSK_PREAMBLE_LEN; i++) *p++ = 0xAA;
    *p++ = 0x2D;
    *p++ = 0xD4;
    uint8_t* body = p;            // seq + payload (CRC computed over this)
    *p++ = 0;                     // seq — patched per frame
    memcpy(p, payload, payload_len);
    p += payload_len;

    for (uint8_t seq = 1; seq <= RADIO_GFSK_REPEATS; seq++) {
        body[0] = seq;
        const uint16_t crc = crc16_ccitt(body, 1 + payload_len);
        p[0] = (uint8_t)(crc >> 8);
        p[1] = (uint8_t)crc;

        if (!radio_tx_frame(frame, frame_len)) {
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

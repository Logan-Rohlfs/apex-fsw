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

// Si4463 frequency formula for 420-480 MHz:
//   fRF = XTAL * (INTE + FRAC / 2^19) * 2 / OUTDIV
#define RADIO_OUTDIV            8UL
#define RADIO_PLL_FRAC_SCALE    (1UL << 19)

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
    inte = (uint8_t)(n_scaled >> 19);
    frac = n_scaled & (RADIO_PLL_FRAC_SCALE - 1UL);
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
    // MODEM_CLKGEN_BAND (0x2051): OUTDIV=8 for 420-480 MHz, high-performance PLL.
    { const uint8_t c[] = { 0x11, 0x20, 0x01, 0x51, 0x0C }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // MODEM_MOD_TYPE (0x2000): CW — pure carrier, no modulation.
    { const uint8_t c[] = { 0x11, 0x20, 0x01, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // GPIO_PIN_CFG (0x13): GPIO2=RX_STATE (RXEN), GPIO3=TX_STATE (TXEN).
    // RF4463PRO routes these internal Si4463 GPIOs to its antenna switch.
    { const uint8_t c[] = { 0x13, 0x00, 0x00, 0x21, 0x20, 0x00, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    // PA_MODE..PA_TC: configure the Si4463 PA block explicitly instead of
    // relying on post-POWER_UP defaults. PA_PWR_LVL=0x20 is still bench-moderate.
    { const uint8_t c[] = { 0x11, 0x22, 0x04, 0x00, 0x08, 0x20, 0x00, 0x3D }; if (!send_cmd(c, sizeof(c))) return false; cs_high(); }

    return true;
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

    LOG_INFO("Radio marker: %.3f MHz, 1s ON / 1s OFF, 5 cycles",
             freq_hz / 1e6f);

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

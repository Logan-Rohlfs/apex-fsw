#include "radio.h"
#include "config.h"
#include "debug.h"

#include <Arduino.h>
#include <SPI.h>

// Si4463 SPI commands used here
#define SI_NOP           0x00
#define SI_PART_INFO     0x01
#define SI_FUNC_INFO     0x10
#define SI_READ_CMD_BUFF 0x44

static int8_t _status = -1;

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
static bool send_cmd(const uint8_t* buf, uint8_t len) {
    cs_low();
    for (uint8_t i = 0; i < len; i++) SPI1.transfer(buf[i]);
    cs_high();
    return wait_cts();
}

// ─── Init ─────────────────────────────────────────────────────────────────────

bool radio_init() {
    pinMode(PIN_RAD_CS,    OUTPUT);
    pinMode(PIN_RAD_INT1,  INPUT);
    pinMode(PIN_RAD_GPIO0, INPUT);   // CTS indicator (after GPIO config)
    pinMode(PIN_RAD_GPIO1, INPUT);
    digitalWrite(PIN_RAD_CS, HIGH);

    SPI1.setMOSI(26);
    SPI1.setMISO(1);
    SPI1.setSCK(27);
    SPI1.begin();
    SPI1.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0)); // 100 kHz — diagnose signal integrity

    // NOP clears any leftover state from a previous run
    cs_low();
    SPI1.transfer(SI_NOP);
    cs_high();
    delay(5);

    // ── PART_INFO ─────────────────────────────────────────────────────────────
    // Safe to call before POWER_UP — Si4463 responds in Boot state.
    // Response (after CTS byte): chipRev, part[1], part[0], pbuild,
    //                             id[1], id[0], customer, romId
    const uint8_t cmd_part[] = { SI_PART_INFO };
    if (!send_cmd(cmd_part, sizeof(cmd_part))) {
        // Read one raw CTS poll byte to distinguish floating MISO from a stuck chip
        cs_low();
        SPI1.transfer(SI_READ_CMD_BUFF);
        uint8_t raw = SPI1.transfer(0x00);
        cs_high();
        SPI1.endTransaction();
        LOG_ERROR("Radio: no CTS after PART_INFO (raw=0x%02X) — %s",
                  raw,
                  raw == 0x00 ? "chip busy but never ready — scope MOSI/SCK, check solder joints" :
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

// ─── Test TX ──────────────────────────────────────────────────────────────────
// Boots the Si4463, configures a CW carrier at RADIO_FREQ_HZ, and starts TX.
// Call once from setup() under APEX_DEBUG to verify the RF chain.
// The carrier stays on until the board is reset.
//
// Frequency math (OUTDIV=8 for 420-480 MHz, XTAL=26 MHz):
//   fRF = XTAL × (INTE + FRAC/2^19) × 2 / OUTDIV
//   433.920 MHz → INTE=66 (0x42), FRAC=396754 (0x060DD2)
bool radio_test_tx() {
    if (_status < 0) {
        LOG_ERROR("Radio: test TX skipped — chip not verified (run radio_init first)");
        return false;
    }

    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    // POWER_UP: boot Si4463 from ROM with crystal oscillator
    {
        const uint8_t xtal = RADIO_XTAL_HZ;  // suppress unused warning
        (void)xtal;
        const uint8_t cmd[] = {
            0x02, 0x01, 0x00,
            (uint8_t)(RADIO_XTAL_HZ >> 24),
            (uint8_t)(RADIO_XTAL_HZ >> 16),
            (uint8_t)(RADIO_XTAL_HZ >>  8),
            (uint8_t)(RADIO_XTAL_HZ      )
        };
        cs_low();
        for (uint8_t i = 0; i < sizeof(cmd); i++) SPI1.transfer(cmd[i]);
        cs_high();
    }
    delay(10); // Si4463 boot takes ~6 ms
    if (!wait_cts(100)) {
        SPI1.endTransaction();
        LOG_ERROR("Radio: no CTS after POWER_UP — check 3V3_2 rail and crystal");
        return false;
    }
    cs_high();

    // MODEM_CLKGEN_BAND (0x2051): OUTDIV=8 for 420-480 MHz, high-performance PLL
    { const uint8_t c[] = { 0x11, 0x20, 0x01, 0x51, 0x0C }; if (!send_cmd(c, sizeof(c))) { cs_high(); SPI1.endTransaction(); return false; } cs_high(); }

    // FREQ_CONTROL (0x4000-0x4003): 433.920 MHz with 26 MHz XTAL → INTE=0x42, FRAC=0x060DD2
    { const uint8_t c[] = { 0x11, 0x40, 0x04, 0x00, 0x42, 0x06, 0x0D, 0xD2 }; if (!send_cmd(c, sizeof(c))) { cs_high(); SPI1.endTransaction(); return false; } cs_high(); }

    // MODEM_MOD_TYPE (0x2000): CW — pure carrier, no modulation
    { const uint8_t c[] = { 0x11, 0x20, 0x01, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) { cs_high(); SPI1.endTransaction(); return false; } cs_high(); }

    // GPIO_PIN_CFG (0x13): GPIO2=RX_STATE (RXEN), GPIO3=TX_STATE (TXEN)
    // RF4463PRO antenna switch requires these to be driven or PA output is blocked.
    { const uint8_t c[] = { 0x13, 0x00, 0x00, 0x12, 0x11, 0x00, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) { cs_high(); SPI1.endTransaction(); return false; } cs_high(); }

    // PA_PWR_LVL (0x2201): 0x08 ≈ 5 dBm — safe for bench, SDR will hear it at 1 m
    { const uint8_t c[] = { 0x11, 0x22, 0x01, 0x01, 0x08 }; if (!send_cmd(c, sizeof(c))) { cs_high(); SPI1.endTransaction(); return false; } cs_high(); }

    // START_TX: channel 0, start immediately, tx_len=0 (CW stays on until reset)
    { const uint8_t c[] = { 0x31, 0x00, 0x00, 0x00, 0x00 }; if (!send_cmd(c, sizeof(c))) { cs_high(); SPI1.endTransaction(); return false; } cs_high(); }

    SPI1.endTransaction();
    LOG_INFO("Radio: CW carrier ON at %lu Hz — tune SDR to %.3f MHz",
             RADIO_FREQ_HZ, RADIO_FREQ_HZ / 1e6f);
    return true;
}

# Device Reference

## NAND Flash — 128MB QSPI (Protosupplied Teensy 4.1 back pads)

**Library:** `LittleFS` (built into Teensyduino)  
**Interface:** QSPI (internal, no pin config needed)

```cpp
#include <LittleFS.h>

LittleFS_QPINAND fs;

// Mount
fs.begin();

// Write
File f = fs.open("log.txt", FILE_WRITE);
f.printf("data: %lu\n", millis());
f.close();

// Read
File f = fs.open("log.txt", FILE_READ);
while (f.available()) Serial.write(f.read());
f.close();

// Stats
fs.totalSize();  // ~131,661,824 bytes
fs.usedSize();
```

---

## ICM-45686 — 6-Axis IMU (ACC1)

**Library:** `tdk-invn-oss/ICM45686`  
**Build flag:** `-DICM45686` (required in `platformio.ini`)  
**Interface:** SPI0 — MOSI=11, MISO=12, SCK=13, CS=10  
**Interrupt:** ACC1_INT1 → Pin 9  
**Class name:** `ICM456xx` (not `ICM45686`)

```cpp
#include <ICM45686.h>

ICM456xx imu(SPI, 10); // SPI bus, CS pin

// Init
imu.begin();                       // returns 0 on success
imu.startAccel(800, 16);           // odr Hz, fsr g
imu.startGyro(800, 2000);          // odr Hz, fsr dps

// Read
inv_imu_sensor_data_t data;
imu.getDataFromRegisters(data);    // returns 0 on success

// Convert (adjust divisor to match configured FSR)
float ax = data.accel_data[0] * (16.0f / 32768.0f) * 9.81f;  // m/s²
float gx = data.gyro_data[0]  * (2000.0f / 32768.0f);        // dps
```

---

## MMC5983MA — Magnetometer (MAG)

**Library:** `https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library`  
**Interface:** I2C1 — SDA1=17, SCL1=16 → `Wire1`

```cpp
#include <SparkFun_MMC5983MA_Arduino_Library.h>

SFE_MMC5983MA mag;

// Init
Wire1.begin();
mag.begin(Wire1);   // returns true on success

// Read (18-bit, centered at 131072)
uint32_t rawX, rawY, rawZ;
mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

// Convert to Gauss
float mx = (rawX - 131072.0f) / 16384.0f;
```

---

## ADXL375 — High-G Accelerometer (ACC2)

**Library:** `adafruit/Adafruit ADXL375`  
**Interface:** I2C0 — SDA=18, SCL=19  
**Address:** 0x53 (SDO tied to GND)  
**Interrupt:** ACC2_INT1 → Pin 8  
**Range:** ±200g

```cpp
#include <Adafruit_ADXL375.h>

Adafruit_ADXL375 accel(12345); // arbitrary sensor ID

// Init
accel.begin(0x53);  // returns false if not found

// Read (m/s²)
sensors_event_t event;
accel.getEvent(&event);
event.acceleration.x;
event.acceleration.y;
event.acceleration.z;
```

---

## MAX-M10S — GPS / GNSS

**Library:** `sparkfun/SparkFun u-blox GNSS v3`  
**Interface:** I2C2 — SDA2=25, SCL2=24 → `Wire2` (primary); UART7 TX=28/RX=29 also available  
**Address:** 0x42 (u-blox default)  
**PPS:** GPS_PPS → Pin 30 (1 Hz pulse, attach interrupt for UTC time sync)  
**Note:** No antenna — do not transmit. I2C communication and UTC time work without antenna.

```cpp
#include <SparkFun_u-blox_GNSS_v3.h>

SFE_UBLOX_GNSS gnss;

// Init
Wire2.begin();
gnss.begin(Wire2);               // returns true on success

// UTC time (only valid after fix)
if (gnss.getTimeValid() && gnss.getDateValid()) {
    uint16_t year   = gnss.getYear();
    uint8_t  month  = gnss.getMonth();
    uint8_t  day    = gnss.getDay();
    uint8_t  hour   = gnss.getHour();
    uint8_t  minute = gnss.getMinute();
    uint8_t  second = gnss.getSecond();
    uint16_t ms     = gnss.getMillisecond();
}

// Position + velocity (UBX NAV-PVT — prefer over NMEA)
gnss.getPVT();                   // triggers a NAV-PVT query
gnss.getLatitude();              // degrees × 1e-7
gnss.getLongitude();             // degrees × 1e-7
gnss.getAltitudeMSL();          // mm above MSL
gnss.getNedNorthVel();          // mm/s north
gnss.getNedDownVel();           // mm/s down (negate for up)
gnss.getSIV();                   // satellites in view
gnss.getFixType();               // 0=no fix, 3=3D fix

// PPS interrupt for precise time sync
pinMode(30, INPUT);
attachInterrupt(digitalPinToInterrupt(30), pps_isr, RISING);
```

---

## RF4463PRO-433 — UHF Telemetry Radio

**Chip:** Silicon Labs Si4463  
**Interface:** SPI1 — MOSI=26, MISO=1, SCK=27, CS=PIN_RAD_CS(0)  
**Pins:** RAD_INT1=2 (nIRQ), RAD_GPIO0=5, RAD_GPIO1=4  
**SDN:** RF4463PRO SDN is active-high shutdown. It must be held low for SPI access.  
**Crystal:** Firmware uses 30 MHz. NiceRF confirms 10 ppm but the public module page is sparse on frequency; verify against the can marking or WDS/vendor sample.  
**Allocated channel:** 441.480 MHz center, 125 kHz bandwidth.  
**Firmware setting:** `RADIO_FREQ_HZ` in `src/config.h` sets the RF center frequency. `RADIO_CHANNEL_BW_HZ` records the allocated bandwidth for the future packet modem configuration.  
**Marker power:** `RADIO_MARKER_PA_PWR` keeps the CW bench marker low enough for nearby RTL-SDR inspection; raise only after range testing with proper attenuation/antenna setup.  
**Note:** Transmit-only downlink. **No antenna — do not enable TX PA.** SPI register reads are safe.  
**Config:** Si4463 normally uses a WDS-generated `radio_config.h` init array. Keep any hand-written command subset small and documented.

```cpp
// No turnkey Arduino library — Si4463 is configured via a WDS-generated
// radio_config.h array loaded during init. Key points:
//
// 1. After POR/shutdown release, send POWER_UP and wait for CTS.
// 2. Poll READ_CMD_BUFF (0x44) for CTS after every SPI command before
//    sending the next one. Missing CTS polling silently drops commands.
// 3. RF4463PRO uses Si4463 GPIO2/GPIO3 for the internal antenna switch:
//    GPIO2=RX_STATE (0x21), GPIO3=TX_STATE (0x20).
// 4. On this board, bias the Teensy MISO/SDO pad before SPI1 takes ownership.
//    Without that startup conditioning, POWER_UP CTS can fail with raw=0x00.
// 5. FREQ_CONTROL uses fRF = (INTE + FRAC/2^19) * 2 * XTAL / OUTDIV.
//    FRAC/2^19 must be in [1, 2), so use INTE=floor(N)-1 and FRAC=(N-INTE)*2^19.
// 6. Do not call SET_TX_POWER or START_TX without an antenna connected.
// 7. SPI1 on Teensy 4.1: use SPI1.begin(), pass SPI1 to your driver.
// 8. Bench commands in monitor builds:
//    RADIO_MARKER = CW carrier at RADIO_FREQ_HZ (currently 441.480 MHz).
//    RADIO_DATA_TEST = 10x 2-GFSK frames (10 kbps, ±25 kHz deviation):
//    preamble 0xAA x 8, sync 0x2D 0xD4, seq byte, payload "APEX RADIO TEST",
//    CRC-16-CCITT. Decode from a laptop/ground Pi with:
//    python sim/scripts/radio_gfsk_rx.py --duration 4 --gain 10
//    (or use the monitor's RTL-SDR source, which decodes live).

// Safe bench test — read chip part info (no TX involved):
//   Send POWER_UP, wait CTS, then send 0x01 (PART_INFO command)
//   Receive CTS + chip ID bytes
//   Si4463 returns chip ID 0x4463.
```

---

## BMP581 — Barometric Pressure + Temperature

**Library:** `sparkfun/SparkFun BMP581 Arduino Library`  
**Interface:** I2C0 — SDA=18, SCL=19  
**Address:** 0x46 (ADR/SDO tied to GND)  
**Interrupt:** BAR_INT1 → Pin 7

```cpp
#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>

BMP581 baro;

// Init
Wire.begin();
baro.beginI2C(0x46);  // BMP5_OK (0) on success

// Read
bmp5_sensor_data data;
baro.getSensorData(&data);
data.pressure;     // Pa
data.temperature;  // °C

// Conversions
float tempF = data.temperature * 9.0f / 5.0f + 32.0f;
float altM  = 44330.0f * (1.0f - powf(data.pressure / 101325.0f, 1.0f / 5.255f));
```

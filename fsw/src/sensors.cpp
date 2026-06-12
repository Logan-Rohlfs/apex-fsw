#include "sensors.h"
#include "config.h"
#include "debug.h"

#include <Wire.h>
#include <SPI.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_ADXL375.h>
#include <ICM45686.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

// ─── Hardware Instances ───────────────────────────────────────────────────────

static BMP581           _baro;
static Adafruit_ADXL375 _highg(12345);
static ICM456xx         _imu(SPI, PIN_ACC1_CS);
static SFE_MMC5983MA    _mag;

// ─── Staging Buffers ──────────────────────────────────────────────────────────
// Written by update functions (called from timer ISRs).
// Read by get functions via atomic copy.

static ImuData   _imu_buf;
static HighGData _highg_buf;
static BaroData  _baro_buf;
static MagData   _mag_buf;

static volatile bool _imu_ready   = false;
static volatile bool _highg_ready = false;
static volatile bool _baro_ready  = false;
static volatile bool _mag_ready   = false;

static uint8_t _health = 0;

// ─── Init ─────────────────────────────────────────────────────────────────────

bool sensors_init() {
    Wire.begin();
    Wire1.begin();

    // BMP581 ─────────────────────────────────────────────────────────────────
    if (_baro.beginI2C(BMP581_ADDR) == BMP5_OK) {
        _health |= SENSOR_OK_BARO;
        LOG_INFO("BMP581 ready");
    } else {
        LOG_ERROR("BMP581 not found");
    }

    // ADXL375 ────────────────────────────────────────────────────────────────
    if (_highg.begin(ADXL375_ADDR)) {
        _highg.setDataRate(ADXL3XX_DATARATE_800_HZ);
        _highg.writeRegister(ADXL3XX_REG_FIFO_CTL, 0x00);  // bypass FIFO
        _health |= SENSOR_OK_HIGHG;
        LOG_INFO("ADXL375 ready");
    } else {
        LOG_ERROR("ADXL375 not found");
    }

    // ICM-45686 ──────────────────────────────────────────────────────────────
    if (_imu.begin() == 0) {
        _imu.startAccel(IMU_ACCEL_ODR_HZ, IMU_ACCEL_FSR_G);
        _imu.startGyro(IMU_GYRO_ODR_HZ,   IMU_GYRO_FSR_DPS);
        _health |= SENSOR_OK_IMU;
        LOG_INFO("ICM-45686 ready");
    } else {
        LOG_ERROR("ICM-45686 not found");
    }

    // MMC5983MA (Wire1: SDA1=17, SCL1=16) ────────────────────────────────────
    if (_mag.begin(Wire1)) {
        _health |= SENSOR_OK_MAG;
        LOG_INFO("MMC5983MA ready");
    } else {
        LOG_ERROR("MMC5983MA not found");
    }

    return (_health == (SENSOR_OK_IMU | SENSOR_OK_HIGHG | SENSOR_OK_BARO | SENSOR_OK_MAG));
}

// ─── Update ───────────────────────────────────────────────────────────────────

void sensors_update_imu() {
    if (!(_health & SENSOR_OK_IMU)) return;

    inv_imu_sensor_data_t raw;
    if (_imu.getDataFromRegisters(raw) != 0) return;

    constexpr float ACCEL_SCALE = (IMU_ACCEL_FSR_G  / 32768.0f) * 9.81f;
    constexpr float GYRO_SCALE  = (IMU_GYRO_FSR_DPS / 32768.0f) * (PI / 180.0f);

    _imu_buf.accel_x_mss  = raw.accel_data[0] * ACCEL_SCALE;
    _imu_buf.accel_y_mss  = raw.accel_data[1] * ACCEL_SCALE;
    _imu_buf.accel_z_mss  = raw.accel_data[2] * ACCEL_SCALE;
    _imu_buf.gyro_x_rads  = raw.gyro_data[0]  * GYRO_SCALE;
    _imu_buf.gyro_y_rads  = raw.gyro_data[1]  * GYRO_SCALE;
    _imu_buf.gyro_z_rads  = raw.gyro_data[2]  * GYRO_SCALE;
    _imu_buf.timestamp_ms = millis();
    _imu_ready = true;
}

void sensors_update_highg() {
    if (!(_health & SENSOR_OK_HIGHG)) return;

    sensors_event_t event;
    _highg.getEvent(&event);

    _highg_buf.accel_x_mss  = event.acceleration.x;
    _highg_buf.accel_y_mss  = event.acceleration.y;
    _highg_buf.accel_z_mss  = event.acceleration.z;
    _highg_buf.timestamp_ms = millis();
    _highg_ready = true;
}

void sensors_update_baro() {
    if (!(_health & SENSOR_OK_BARO)) return;

    bmp5_sensor_data raw;
    if (_baro.getSensorData(&raw) != BMP5_OK) return;

    _baro_buf.pressure_pa    = raw.pressure;
    _baro_buf.temperature_c  = raw.temperature;
    _baro_buf.timestamp_ms   = millis();
    _baro_ready = true;
}

void sensors_update_mag() {
    if (!(_health & SENSOR_OK_MAG)) return;

    uint32_t rx, ry, rz;
    if (!_mag.getMeasurementXYZ(&rx, &ry, &rz)) return;

    constexpr float MAG_SCALE = 1.0f / 16384.0f;
    constexpr float MAG_ZERO  = 131072.0f;

    _mag_buf.x_gauss     = (rx - MAG_ZERO) * MAG_SCALE;
    _mag_buf.y_gauss     = (ry - MAG_ZERO) * MAG_SCALE;
    _mag_buf.z_gauss     = (rz - MAG_ZERO) * MAG_SCALE;
    _mag_buf.timestamp_ms = millis();
    _mag_ready = true;
}

// ─── Get (atomic copy) ────────────────────────────────────────────────────────

bool sensors_get_imu(ImuData& out) {
    if (!_imu_ready) return false;
    noInterrupts();
    out = _imu_buf;
    _imu_ready = false;
    interrupts();
    return true;
}

bool sensors_get_highg(HighGData& out) {
    if (!_highg_ready) return false;
    noInterrupts();
    out = _highg_buf;
    _highg_ready = false;
    interrupts();
    return true;
}

bool sensors_get_baro(BaroData& out) {
    if (!_baro_ready) return false;
    noInterrupts();
    out = _baro_buf;
    _baro_ready = false;
    interrupts();
    return true;
}

bool sensors_get_mag(MagData& out) {
    if (!_mag_ready) return false;
    noInterrupts();
    out = _mag_buf;
    _mag_ready = false;
    interrupts();
    return true;
}

uint8_t sensors_health() {
    return _health;
}

// ─── HIL injection ────────────────────────────────────────────────────────────
#ifdef APEX_HIL

void sensors_init_hil() {
    _health = SENSOR_OK_IMU | SENSOR_OK_HIGHG | SENSOR_OK_BARO | SENSOR_OK_MAG;
}

void sensors_inject_hil(const SimPacket& pkt) {
    static uint8_t _tick = 0;
    _tick++;

    // Timestamps are millis() — exactly what the real sensor update paths
    // write. The freshness consumers (gps_monitor_update staleness window,
    // fusion's BARO_DEAD_MS fallback) all compare against millis(), so
    // sim-time stamps would make every injected sample look permanently
    // stale. pkt.sim_time_ms is still echoed in the TeensyPacket reply.
    uint32_t ts = millis();

    noInterrupts();

    // IMU — every tick (100 Hz)
    _imu_buf.accel_x_mss  = pkt.accel_x_mss;
    _imu_buf.accel_y_mss  = pkt.accel_y_mss;
    _imu_buf.accel_z_mss  = pkt.accel_z_mss;
    _imu_buf.gyro_x_rads  = pkt.gyro_x_rads;
    _imu_buf.gyro_y_rads  = pkt.gyro_y_rads;
    _imu_buf.gyro_z_rads  = pkt.gyro_z_rads;
    _imu_buf.timestamp_ms = ts;
    _imu_ready = true;

    // High-G — every tick. Telemega replay mirrors ICM accel here;
    // firmware switches to high-G only above 14g which barely occurs in
    // this flight (peak 14.10g at t=1.12s — noted limitation).
    _highg_buf.accel_x_mss  = pkt.highg_x_mss;
    _highg_buf.accel_y_mss  = pkt.highg_y_mss;
    _highg_buf.accel_z_mss  = pkt.highg_z_mss;
    _highg_buf.timestamp_ms = ts;
    _highg_ready = true;

    interrupts();

    // Baro — every 2nd tick → 50 Hz equivalent
    if ((_tick & 0x01) == 0) {
        noInterrupts();
        _baro_buf.pressure_pa   = pkt.baro_pa;
        // temperature_c not in SimPacket — leave previous value unchanged
        _baro_buf.timestamp_ms  = ts;
        _baro_ready = true;
        interrupts();
    }

    // Mag — every 4th tick → 25 Hz equivalent
    if ((_tick & 0x03) == 0) {
        noInterrupts();
        _mag_buf.x_gauss      = pkt.mag_x_gauss;
        _mag_buf.y_gauss      = pkt.mag_y_gauss;
        _mag_buf.z_gauss      = pkt.mag_z_gauss;
        _mag_buf.timestamp_ms = ts;
        _mag_ready = true;
        interrupts();
    }

    // GPS — write directly to g_state (bypasses staging path, async).
    // On an invalid packet the fix flags are cleared and timestamp_ms is NOT
    // advanced, so gps_monitor_update() sees the loss exactly as it would
    // with real hardware going silent (stale solution → fix lost).
    if (pkt.gps_valid) {
        g_state.gps.altitude_msl_m = pkt.gps_alt_msl_m;
        g_state.gps.valid          = true;
        g_state.gps.fix_quality    = 3;
        g_state.gps.satellites     = 12;
        g_state.gps.timestamp_ms   = ts;
    } else {
        g_state.gps.valid       = false;
        g_state.gps.fix_quality = 0;
        g_state.gps.satellites  = 3;   // tracking but no nav solution
    }
}

#endif // APEX_HIL

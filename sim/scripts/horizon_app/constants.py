"""Static configuration: plot-group layouts, phase colors, app pages, limits.

Pure data — no Qt, no theme. The multi-hue colors in the *_PLOT_GROUPS lists are
the categorical data palette (one hue per trace so 3-4 overlaid series stay
distinguishable); they are intentionally independent of the red/white/grey chrome
theme. Phase colors are tuned to read on both light and dark backgrounds.
"""

from __future__ import annotations

from radio_gfsk_rx import (
    BITRATE_BPS,
    DEFAULT_FREQ_HZ,
    DEVIATION_HZ,
)

# Expected flight-computer TX channel (matches fsw RADIO_FREQ_HZ / GFSK params).
EXPECTED_TX_HZ = DEFAULT_FREQ_HZ
EXPECTED_TX_BW_HZ = 2 * (DEVIATION_HZ + BITRATE_BPS // 2)   # Carson bandwidth

# ─── Plot group definitions ───────────────────────────────────────────────────
# (group_title, [keys...], [hex_colors...])
# Keys matched against incoming >key:value lines. Unmatched keys go to "Other".

PLOT_GROUPS = [
    ("Fusion",    ["alt_agl", "velocity", "pred_apogee", "vert_accel"],
                  ["#00d4ff", "#ff6b35", "#a8ff3e", "#bb88ff"]),
    ("Airbrakes", ["deployment", "servo_angle"],            ["#00d4ff", "#ff9933"]),
    ("IMU Accel", ["accel_x", "accel_y", "accel_z"],       ["#ff4444", "#44ff44", "#4488ff"]),
    ("IMU Gyro",  ["gyro_x",  "gyro_y",  "gyro_z"],        ["#ff8844", "#88ff44", "#4488ff"]),
    ("High-G",    ["highg_x", "highg_y", "highg_z"],       ["#ff44aa", "#44ffaa", "#aa44ff"]),
    ("Baro Alt",  ["baro_alt"],                               ["#ffdd44"]),
    ("Baro Temp", ["baro_temp"],                             ["#ff9922"]),
    ("Mag",       ["mag_x",   "mag_y",   "mag_z"],          ["#ff9933", "#33ff99", "#9933ff"]),
]

# Ground-station layout for the RTL-SDR source — keys match the FLIGHT/HK
# telemetry frames (radio_gfsk_rx.py), tuned for flight watching + HORIZON ops.
RADIO_PLOT_GROUPS = [
    ("Altitude",         ["alt_agl", "baro_alt", "pred_apogee", "gps_alt_msl"],
                         ["#00d4ff", "#ffdd44", "#a8ff3e", "#ff44aa"]),
    ("Velocity / Accel", ["velocity", "vert_accel", "accel_z"],
                         ["#ff6b35", "#44ff44", "#4488ff"]),
    ("Roll / Airbrake",  ["roll_rate", "deployment"],
                         ["#ff8844", "#00d4ff"]),
    ("Attitude",         ["tilt_deg", "azimuth_deg"], ["#ffaa44", "#44aaff"]),
    ("Link",             ["packet_loss_pct", "radio_rx_quality", "radio_freq_offset_khz"],
                         ["#ff4444", "#44ffaa", "#ffdd44"]),
]

# HIL layout — closed-loop sim view. est_* comes from the flight computer's
# TeensyPackets, true_* from the RocketPy state, sim_* is what the emulator
# injects. Dim gray = ground truth "ghost" under the FC's estimate.
HIL_PLOT_GROUPS = [
    ("State Estimation", ["est_alt", "true_alt", "pred_apogee", "fake_est_alt", "fake_pred_apogee"],
                         ["#00d4ff", "#8888aa", "#a8ff3e", "#ff44aa", "#ffaa44"]),
    ("Velocity",         ["est_vel", "true_vel", "fake_est_vel"],
                         ["#ff6b35", "#8888aa", "#ff44aa"]),
    ("Airbrakes",        ["deployment", "fake_deployment"],
                         ["#00d4ff", "#ff44aa"]),
    ("Injected Accel",   ["sim_accel_x", "sim_accel_y", "sim_accel_z"],
                         ["#ff4444", "#44ff44", "#4488ff"]),
    ("Injected Gyro",    ["sim_gyro_x", "sim_gyro_y", "sim_gyro_z"],
                         ["#ff8844", "#88ff44", "#4488ff"]),
    ("Injected Baro",    ["sim_baro_pa"],
                         ["#ffdd44"]),
    ("Loop Latency",     ["hil_lat_ms", "fake_hil_lat_ms"],
                         ["#ff44aa", "#44ffaa"]),
    ("Fake Shadow Delta", ["fake_delta_alt", "fake_delta_vel", "fake_delta_deployment"],
                          ["#ff44aa", "#44ffaa", "#ffaa44"]),
]

HIL_EVENT_STYLES = {
    "launch":          ("Launch", "#44dd88"),
    "burnout":         ("Burnout", "#ff9933"),
    "first_airbrake":  ("First airbrake", "#00d4ff"),
    "apogee":          ("Apogee", "#ff44aa"),
}

# Flight-info reference values — mirror fsw/src/config.h / airbrakes.yaml
TARGET_APOGEE_M = 3048.0    # 10,000 ft AGL
MACH_GATE_MPS   = 240.0     # Mach 0.7 — deployment allowed below this
FT_PER_M        = 3.28084

# App pages (tab index → source mode)
PAGES = [
    ("Sensors", "serial"),
    ("Radio", "radio"),
    ("HIL", "hil"),
    ("Logs", "logs"),
]

# Sensor health bitmask (must match config.h)
SENSOR_BITS = {"IMU": 0, "HG": 1, "BAR": 2, "MAG": 3}

GPS_FIX_LABELS = {-1: "OFFLINE", 0: "SEARCHING", 1: "DR", 2: "2D", 3: "3D", 4: "3D+DR"}

PHASE_COLORS = {
    "IDLE":    "#555566",
    "ARMED":   "#2277dd",
    "BOOST":   "#dd5500",
    "COAST":   "#22aa44",
    "DESCENT": "#ccaa00",
    "LANDED":  "#884499",
    "UNKNOWN": "#333333",
}

MAX_POINTS     = 6000   # rolling buffer depth per series (~60s at 100 Hz)
PLOT_UPDATE_HZ = 25     # UI repaint rate
DEFAULT_WINDOW_S = 30

# Spectrum / waterfall (RTL-SDR)
FFT_BINS       = 1024   # spectrum resolution
SPECTRUM_AVG   = 6      # FFTs averaged per displayed frame
SPECTRUM_FPS   = 30     # spectrum frames emitted per second
WATERFALL_ROWS = 100    # waterfall history depth (rows of FFT_BINS)

KNOWN_COMMANDS = [
    "ARM",
    "DISARM",
    "FORMAT_QSPI_ERASE_ALL",   # explicit GUI-confirmed QSPI erase/reformat
    "LIST_LOGS",
    "RADIO_DATA_TEST",
    "RADIO_MARKER",
    "RADIO_MARKER_441",
    "TELEM_ON",
    "TELEM_OFF",
]

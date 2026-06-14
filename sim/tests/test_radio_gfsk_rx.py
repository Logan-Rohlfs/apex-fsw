import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import radio_gfsk_rx as rx  # noqa: E402


def test_flight_status_bits_decode_without_changing_body_size():
    phase_status = (
        3
        | rx.STATUS_AIRBRAKES_OK
        | rx.STATUS_SERVO_POWER
        | rx.STATUS_ARM_SWITCHES
        | rx.STATUS_LOGGING_READY
        | rx.STATUS_GPS_TIME_VALID
    )
    health = 0x0F | rx.HEALTH_GPS | rx.HEALTH_RADIO | rx.HEALTH_QSPI | rx.HEALTH_SD
    body = rx.FLIGHT_STRUCT.pack(
        b"KG5LDI", 42, phase_status, health, 3, 12,
        33.5, -101.8,
        2000, 1000, 500, 1200, 981, 1500, 25,
        128, 50000, 24,
    )

    assert len(body) == 38
    decoded = rx.parse_flight(body)
    assert decoded["phase"] == 3
    assert decoded["phase_name"] == "COAST"
    assert decoded["airbrakes_authorized"]
    assert decoded["servo_powered"]
    assert decoded["arm_switches_closed"]
    assert decoded["logging_ready"]
    assert decoded["gps_time_valid"]
    assert decoded["gps_healthy"]
    assert decoded["radio_healthy"]
    assert decoded["qspi_healthy"]
    assert decoded["sd_healthy"]


def test_phase_mask_ignores_status_upper_bits():
    body = rx.FLIGHT_STRUCT.pack(
        b"KG5LDI", 1, 2 | rx.STATUS_SERVO_POWER, 0, -1, 0,
        0.0, 0.0, *([0] * 7), 0, 0, 0,
    )
    decoded = rx.parse_flight(body)
    assert decoded["phase_name"] == "BOOST"
    assert decoded["servo_powered"]
    assert not decoded["airbrakes_authorized"]

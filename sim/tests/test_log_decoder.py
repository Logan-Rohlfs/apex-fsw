from __future__ import annotations

import csv
from pathlib import Path

from apex_sim.logs import decoder


def _record(rec_type: int, seq: int, boot_id: int, flight_id: int,
            time_ms: int, payload: bytes) -> bytes:
    header = decoder.HEADER.pack(
        decoder.MAGIC,
        decoder.VERSION,
        rec_type,
        len(payload),
        seq,
        boot_id,
        flight_id,
        time_ms,
        0,
    )
    crc = decoder.crc16_ccitt(header + payload)
    header = decoder.HEADER.pack(
        decoder.MAGIC,
        decoder.VERSION,
        rec_type,
        len(payload),
        seq,
        boot_id,
        flight_id,
        time_ms,
        crc,
    )
    return header + payload


def _boot_payload() -> bytes:
    return decoder.BOOT_V1.pack(
        0x00000002,  # debug build flag
        0x1234ABCD,  # config hash
        decoder.HEADER.size,
        decoder.BOOT_V1.size,
        decoder.EVENT.size,
        decoder.SAMPLE_V1.size,
        200,
        100,
        100,
        304800,
        100,
        0x03,
    )


def _event_payload(event_id: int, phase: int, detail: str) -> bytes:
    raw = detail.encode("utf-8")[:47]
    raw += b"\0" * (48 - len(raw))
    return decoder.EVENT.pack(event_id, phase, 0x03, 3, 0, raw)


def _sample_payload(sample_ms: int, phase: int, flighty: bool = True) -> bytes:
    floats = [
        71.4, 0.8, -0.4,
        0.01, 0.001, 0.004,
        72.0,
        91315.8, 22.5,
        2.1 if flighty else 0.3,
        8.7 if flighty else 0.0,
        4180.2 if flighty else 0.3,
        62.0 if flighty else 0.0,
        0.083 if flighty else 0.0,
        1132.2, 278.9, -1.2, -31.3,
        32.100001, -106.900002, 1401.2,
    ]
    return decoder.SAMPLE_V1.pack(
        sample_ms,
        phase,
        0x03,
        3,
        12,
        0,
        1 if flighty else 0,
        1,
        2026,
        6,
        12,
        18,
        44,
        3,
        281,
        0,
        *floats,
    )


def _write_log(path: Path) -> None:
    blob = b"junk"  # prove the decoder can resync to the magic word
    blob += _record(decoder.REC_BOOT, 0, 42, 0, 1000, _boot_payload())
    blob += _record(decoder.REC_EVENT, 1, 42, 0, 2000,
                    _event_payload(2, 1, "armed"))
    blob += _record(decoder.REC_EVENT, 2, 42, 2, 10170,
                    _event_payload(4, 2, "accel"))
    blob += _record(decoder.REC_SAMPLE, 3, 42, 2, 10020,
                    _sample_payload(10020, 1, flighty=False))
    blob += _record(decoder.REC_SAMPLE, 4, 42, 2, 10170,
                    _sample_payload(10170, 2, flighty=True))
    path.write_bytes(blob)


def test_decode_file_resyncs_and_parses_payloads(tmp_path):
    log_path = tmp_path / "BOOT_00042.APXLOG"
    _write_log(log_path)

    records, stats = decoder.decode_file(log_path)

    assert stats.records == 5
    assert stats.resync_bytes == 4
    assert stats.bad_crc == 0
    assert records[0].payload["config_hash"] == 0x1234ABCD
    assert records[2].payload["event"] == "LAUNCH_DETECTED"
    assert records[-1].payload["utc"] == "2026-06-12T18:44:03.281Z"
    assert records[-1].payload["phase"] == "BOOST"


def test_export_logs_writes_one_wide_csv_per_flight(tmp_path):
    log_path = tmp_path / "BOOT_00042.APXLOG"
    out_dir = tmp_path / "export"
    _write_log(log_path)

    written = decoder.export_logs([log_path], out_dir, include_ground=False)

    assert len(written) == 1
    csv_path = written[0]
    assert csv_path.name == "Flight_02_2026-06-12T18-44-03-281.csv"
    rows = list(csv.DictReader(csv_path.open()))
    assert [row["record_type"] for row in rows] == ["EVENT", "SAMPLE", "SAMPLE"]
    assert rows[0]["event"] == "LAUNCH_DETECTED"
    assert rows[-1]["utc"] == "2026-06-12T18:44:03.281Z"
    assert rows[-1]["local_time"]
    assert rows[-1]["local_time"] != rows[-1]["utc"]
    assert rows[-1]["phase"] == "BOOST"
    assert rows[-1]["deploy"]
    assert (csv_path.parent / "metadata.json").exists()
    assert (csv_path.parent / log_path.name).exists()

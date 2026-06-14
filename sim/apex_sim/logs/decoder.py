"""Decode Apex firmware binary logs and export Excel-friendly CSV files."""

from __future__ import annotations

import csv
import json
import shutil
import struct
import zlib
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Iterator, Optional, Union

MAGIC = 0x4C585041  # "APXL" little-endian
VERSION = 1
RAW_PAGE_MAGIC = 0x52585041  # "APXR" little-endian
RAW_PAGE_VERSION = 1

REC_BOOT = 1
REC_EVENT = 2
REC_SAMPLE = 3

RECORD_NAMES = {
    REC_BOOT: "BOOT",
    REC_EVENT: "EVENT",
    REC_SAMPLE: "SAMPLE",
}

EVENT_NAMES = {
    1: "BOOT",
    2: "ARMED",
    3: "DISARMED",
    4: "LAUNCH_DETECTED",
    5: "PHASE",
    6: "CONTROL_ACTIVE",
    7: "STORAGE_FAULT",
    8: "HIL_SESSION_END",
}

PHASE_NAMES = {
    0: "IDLE",
    1: "ARMED",
    2: "BOOST",
    3: "COAST",
    4: "DESCENT",
    5: "LANDED",
}

HEADER = struct.Struct("<IBBHIIIIH")
RAW_PAGE = struct.Struct("<IHHIIIIHHI")
BOOT_V1 = struct.Struct("<IIIIIIIIIIIB3x")
BOOT_V0 = struct.Struct("<IIIIIIB3x")
EVENT = struct.Struct("<BBBBH48s")
SAMPLE_V1 = struct.Struct("<IBBBBHBBHBBBBBHB" + "f" * 21)
SAMPLE_V0 = struct.Struct("<IBBBBHBB" + "f" * 21)

CSV_COLUMNS = [
    "record_type", "seq", "boot_id", "flight_id", "time_ms",
    "sample_ms", "phase", "event", "event_detail",
    "utc", "local_time", "gps_fix", "gps_sats", "storage_health", "storage_faults",
    "config_hash", "build_flags", "rate_fusion_hz", "rate_state_hz",
    "rate_control_hz", "log_flight_hz",
    "alt_m", "vel_mps", "pred_apogee_m", "vert_accel_mps2",
    "ax_mss", "ay_mss", "az_mss",
    "gx_rads", "gy_rads", "gz_rads",
    "highg_x_mss", "baro_pa", "baro_temp_c",
    "deploy", "control_active",
    "pid_error_m", "pid_p", "pid_i", "pid_d",
    "gps_lat_deg", "gps_lon_deg", "gps_alt_msl_m",
]


@dataclass
class LogRecord:
    path: Path
    offset: int
    record_type: int
    seq: int
    boot_id: int
    flight_id: int
    time_ms: int
    payload: dict

    @property
    def type_name(self) -> str:
        return RECORD_NAMES.get(self.record_type, str(self.record_type))


@dataclass
class DecodeStats:
    bytes_read: int = 0
    records: int = 0
    bad_crc: int = 0
    bad_version: int = 0
    resync_bytes: int = 0
    truncated: int = 0
    unsupported: int = 0
    duplicates: int = 0     # records dropped as (boot_id, seq) duplicates
    raw_pages: int = 0
    files: list[str] = field(default_factory=list)

    def merge(self, other: "DecodeStats") -> None:
        self.bytes_read += other.bytes_read
        self.records += other.records
        self.bad_crc += other.bad_crc
        self.bad_version += other.bad_version
        self.resync_bytes += other.resync_bytes
        self.truncated += other.truncated
        self.unsupported += other.unsupported
        self.duplicates += other.duplicates
        self.raw_pages += other.raw_pages
        self.files.extend(other.files)


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_file(path: Union[Path, str]) -> tuple[list[LogRecord], DecodeStats]:
    path = Path(path)
    data = path.read_bytes()
    stats = DecodeStats(bytes_read=len(data), files=[str(path)])
    data = _unwrap_raw_pages(data, stats)
    records: list[LogRecord] = []
    i = 0
    while i + HEADER.size <= len(data):
        magic = struct.unpack_from("<I", data, i)[0]
        if magic != MAGIC:
            i += 1
            stats.resync_bytes += 1
            continue

        header_bytes = data[i:i + HEADER.size]
        (magic, version, rec_type, length, seq,
         boot_id, flight_id, time_ms, crc) = HEADER.unpack(header_bytes)
        total = HEADER.size + length
        if i + total > len(data):
            stats.truncated += 1
            break
        if version != VERSION:
            stats.bad_version += 1
            i += 1
            continue

        frame = bytearray(data[i:i + total])
        struct.pack_into("<H", frame, HEADER.size - 2, 0)
        if crc16_ccitt(frame) != crc:
            stats.bad_crc += 1
            i += 1
            continue

        payload_bytes = data[i + HEADER.size:i + total]
        payload = _parse_payload(rec_type, payload_bytes)
        if payload is None:
            stats.unsupported += 1
            i += total
            continue
        records.append(LogRecord(
            path=path,
            offset=i,
            record_type=rec_type,
            seq=seq,
            boot_id=boot_id,
            flight_id=flight_id,
            time_ms=time_ms,
            payload=payload,
        ))
        stats.records += 1
        i += total
    return records, stats


def _unwrap_raw_pages(data: bytes, stats: DecodeStats) -> bytes:
    if len(data) < RAW_PAGE.size:
        return data
    first_magic = struct.unpack_from("<I", data, 0)[0]
    if first_magic != RAW_PAGE_MAGIC:
        return data

    payload = bytearray()
    page_size = 2048
    for offset in range(0, len(data) - RAW_PAGE.size + 1, page_size):
        page = data[offset:offset + page_size]
        if page[:16] == b"\xff" * 16:
            break
        (magic, version, header_size, _boot_id, _flight_id, _page_seq,
         _time_ms, payload_len, _flags, crc32) = RAW_PAGE.unpack_from(page, 0)
        if magic != RAW_PAGE_MAGIC:
            break
        if version != RAW_PAGE_VERSION or header_size != RAW_PAGE.size:
            stats.bad_version += 1
            break
        if payload_len > page_size - header_size:
            stats.truncated += 1
            break
        check = bytearray(page[:header_size + payload_len])
        struct.pack_into("<I", check, RAW_PAGE.size - 4, 0)
        if zlib.crc32(check) & 0xFFFFFFFF != crc32:
            stats.bad_crc += 1
            break
        payload.extend(page[header_size:header_size + payload_len])
        stats.raw_pages += 1
    return bytes(payload)


def decode_files(paths: Iterable[Union[Path, str]]) -> tuple[list[LogRecord], DecodeStats]:
    all_records: list[LogRecord] = []
    stats = DecodeStats()
    for path in paths:
        records, one = decode_file(path)
        all_records.extend(records)
        stats.merge(one)
    all_records.sort(key=lambda r: (r.boot_id, r.seq, r.time_ms, str(r.path), r.offset))

    # De-duplicate by (boot_id, seq). seq is monotonic per boot, so the same
    # record appearing in more than one file — the QSPI black box, the SD live
    # log, and the post-landing SD dump can all contain it — collapses to one
    # row. This lets the firmware mirror/dump to SD freely with no bookkeeping
    # to avoid overlap; correctness is resolved here at decode.
    deduped: list[LogRecord] = []
    seen: set = set()
    for rec in all_records:
        key = (rec.boot_id, rec.seq)
        if key in seen:
            stats.duplicates += 1
            continue
        seen.add(key)
        deduped.append(rec)
    return deduped, stats


def export_logs(paths: Iterable[Union[Path, str]], out_dir: Union[Path, str],
                include_ground: bool = True) -> list[Path]:
    paths = [Path(p) for p in paths]
    records, stats = decode_files(paths)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    groups: dict[tuple[str, int, int], list[LogRecord]] = {}
    for record in records:
        if record.flight_id:
            key = ("Flight", record.flight_id, record.boot_id)
        elif include_ground:
            key = ("Ground", 0, record.boot_id)
        else:
            continue
        groups.setdefault(key, []).append(record)

    written: list[Path] = []
    for key, group in sorted(groups.items(), key=lambda kv: kv[0]):
        kind, flight_id, boot_id = key
        folder_name = _group_name(kind, flight_id, boot_id, group)
        folder = out_dir / folder_name
        folder.mkdir(parents=True, exist_ok=True)
        csv_path = folder / f"{folder_name}.csv"
        _write_csv(csv_path, group)
        _write_metadata(folder / "metadata.json", group, stats)
        for raw in sorted({r.path for r in group}):
            dst = folder / raw.name
            if raw.exists() and raw.resolve() != dst.resolve():
                shutil.copy2(raw, dst)
        written.append(csv_path)
    return written


def iter_log_paths(inputs: Iterable[Union[Path, str]]) -> Iterator[Path]:
    for item in inputs:
        path = Path(item)
        if path.is_dir():
            yield from sorted(path.rglob("*.APXLOG"))
        else:
            yield path


def _parse_payload(rec_type: int, data: bytes) -> Optional[dict]:
    if rec_type == REC_BOOT:
        return _parse_boot(data)
    if rec_type == REC_EVENT:
        return _parse_event(data)
    if rec_type == REC_SAMPLE:
        return _parse_sample(data)
    return None


def _parse_boot(data: bytes) -> Optional[dict]:
    if len(data) >= BOOT_V1.size:
        values = BOOT_V1.unpack(data[:BOOT_V1.size])
        keys = [
            "build_flags", "config_hash", "log_header_size",
            "boot_payload_size", "event_payload_size", "sample_payload_size",
            "rate_fusion_hz", "rate_state_hz", "rate_control_hz",
            "target_apogee_cm", "log_flight_hz", "storage_health",
        ]
        return dict(zip(keys, values))
    if len(data) >= BOOT_V0.size:
        values = BOOT_V0.unpack(data[:BOOT_V0.size])
        keys = [
            "build_flags", "rate_fusion_hz", "rate_state_hz",
            "rate_control_hz", "target_apogee_cm", "log_flight_hz",
            "storage_health",
        ]
        payload = dict(zip(keys, values))
        payload["config_hash"] = None
        return payload
    return None


def _parse_event(data: bytes) -> Optional[dict]:
    if len(data) < EVENT.size:
        return None
    event_id, phase, storage_health, gps_fix, storage_faults, detail = EVENT.unpack(data[:EVENT.size])
    return {
        "event_id": event_id,
        "event": EVENT_NAMES.get(event_id, str(event_id)),
        "phase_id": phase,
        "phase": PHASE_NAMES.get(phase, str(phase)),
        "storage_health": storage_health,
        "gps_fix": _signed_u8(gps_fix),
        "storage_faults": storage_faults,
        "detail": detail.split(b"\0", 1)[0].decode("utf-8", errors="replace"),
    }


def _parse_sample(data: bytes) -> Optional[dict]:
    if len(data) >= SAMPLE_V1.size:
        values = SAMPLE_V1.unpack(data[:SAMPLE_V1.size])
        prefix = values[:16]
        floats = values[16:]
        sample = _sample_from_parts(prefix, floats, has_utc=True)
        return sample
    if len(data) >= SAMPLE_V0.size:
        values = SAMPLE_V0.unpack(data[:SAMPLE_V0.size])
        prefix = values[:8]
        floats = values[8:]
        return _sample_from_parts(prefix, floats, has_utc=False)
    return None


def _sample_from_parts(prefix, floats, has_utc: bool) -> dict:
    if has_utc:
        (sample_ms, phase, storage_health, gps_fix, gps_sats,
         storage_faults, control_active, utc_valid, utc_year, utc_month,
         utc_day, utc_hour, utc_minute, utc_second, utc_ms, _reserved) = prefix
    else:
        (sample_ms, phase, storage_health, gps_fix, gps_sats,
         storage_faults, control_active, _reserved) = prefix
        utc_valid = 0
        utc_year = utc_month = utc_day = utc_hour = utc_minute = utc_second = utc_ms = 0

    float_keys = [
        "ax_mss", "ay_mss", "az_mss",
        "gx_rads", "gy_rads", "gz_rads",
        "highg_x_mss", "baro_pa", "baro_temp_c",
        "alt_m", "vel_mps", "pred_apogee_m", "vert_accel_mps2",
        "deploy", "pid_error_m", "pid_p", "pid_i", "pid_d",
        "gps_lat_deg", "gps_lon_deg", "gps_alt_msl_m",
    ]
    payload = dict(zip(float_keys, floats))
    payload.update({
        "sample_ms": sample_ms,
        "phase_id": phase,
        "phase": PHASE_NAMES.get(phase, str(phase)),
        "storage_health": storage_health,
        "gps_fix": _signed_u8(gps_fix),
        "gps_sats": gps_sats,
        "storage_faults": storage_faults,
        "control_active": control_active,
        "utc_valid": utc_valid,
        "utc_year": utc_year,
        "utc_month": utc_month,
        "utc_day": utc_day,
        "utc_hour": utc_hour,
        "utc_minute": utc_minute,
        "utc_second": utc_second,
        "utc_ms": utc_ms,
        "utc": _utc_string(utc_valid, utc_year, utc_month, utc_day,
                           utc_hour, utc_minute, utc_second, utc_ms),
    })
    payload["local_time"] = _local_time_string(payload["utc"])
    return payload


def _write_csv(path: Path, records: list[LogRecord]) -> None:
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for record in records:
            writer.writerow(_csv_row(record))


def _csv_row(record: LogRecord) -> dict:
    row = {
        "record_type": record.type_name,
        "seq": record.seq,
        "boot_id": record.boot_id,
        "flight_id": record.flight_id,
        "time_ms": record.time_ms,
    }
    payload = record.payload
    if record.record_type == REC_BOOT:
        row.update({
            "event": "BOOT",
            "config_hash": _hex_or_blank(payload.get("config_hash")),
            "build_flags": _hex_or_blank(payload.get("build_flags")),
            "rate_fusion_hz": payload.get("rate_fusion_hz"),
            "rate_state_hz": payload.get("rate_state_hz"),
            "rate_control_hz": payload.get("rate_control_hz"),
            "log_flight_hz": payload.get("log_flight_hz"),
            "storage_health": payload.get("storage_health"),
        })
    elif record.record_type == REC_EVENT:
        row.update({
            "phase": payload.get("phase"),
            "event": payload.get("event"),
            "event_detail": payload.get("detail"),
            "gps_fix": payload.get("gps_fix"),
            "storage_health": payload.get("storage_health"),
            "storage_faults": payload.get("storage_faults"),
        })
    elif record.record_type == REC_SAMPLE:
        row.update(payload)
    return row


def _write_metadata(path: Path, records: list[LogRecord], stats: DecodeStats) -> None:
    boot_records = [r for r in records if r.record_type == REC_BOOT]
    sample_records = [r for r in records if r.record_type == REC_SAMPLE]
    events = [r for r in records if r.record_type == REC_EVENT]
    metadata = {
        "record_count": len(records),
        "boot_ids": sorted({r.boot_id for r in records}),
        "flight_ids": sorted({r.flight_id for r in records}),
        "source_files": sorted({str(r.path) for r in records}),
        "first_time_ms": min((r.time_ms for r in records), default=None),
        "last_time_ms": max((r.time_ms for r in records), default=None),
        "first_utc": _first_utc(sample_records),
        "events": [
            {
                "time_ms": r.time_ms,
                "event": r.payload.get("event"),
                "detail": r.payload.get("detail"),
            }
            for r in events
        ],
        "boot": boot_records[0].payload if boot_records else {},
        "decode_stats": {
            "bytes_read": stats.bytes_read,
            "records": stats.records,
            "bad_crc": stats.bad_crc,
            "bad_version": stats.bad_version,
            "resync_bytes": stats.resync_bytes,
            "truncated": stats.truncated,
            "unsupported": stats.unsupported,
        },
    }
    path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")


def _group_name(kind: str, flight_id: int, boot_id: int, records: list[LogRecord]) -> str:
    utc = _first_utc([r for r in records if r.record_type == REC_SAMPLE])
    if kind == "Flight":
        suffix = _safe_time(utc) if utc else f"BOOT_{boot_id:05d}"
        return f"Flight_{flight_id:02d}_{suffix}"
    return f"Ground_BOOT_{boot_id:05d}"


def _first_utc(records: list[LogRecord]) -> Optional[str]:
    for record in records:
        utc = record.payload.get("utc")
        if utc:
            return utc
    return None


def _utc_string(valid, year, month, day, hour, minute, second, ms) -> str:
    if not valid or not year:
        return ""
    return f"{year:04d}-{month:02d}-{day:02d}T{hour:02d}:{minute:02d}:{second:02d}.{ms:03d}Z"


def _local_time_string(utc: str) -> str:
    if not utc:
        return ""
    try:
        dt = datetime.strptime(utc, "%Y-%m-%dT%H:%M:%S.%fZ")
    except ValueError:
        return ""
    local = dt.replace(tzinfo=timezone.utc).astimezone()
    return local.strftime("%I:%M:%S.%f")[:-3] + local.strftime(" %p %Z")


def _safe_time(utc: str) -> str:
    return utc.replace(":", "-").replace(".", "-").rstrip("Z")


def _hex_or_blank(value) -> str:
    return "" if value is None else f"0x{int(value):08X}"


def _signed_u8(value: int) -> int:
    return value - 256 if value >= 128 else value

"""Background worker threads — the firmware-facing I/O layer.

Relocated verbatim from the original horizon.py. These own every byte that
crosses to/from the flight computer (USB serial), the RTL-SDR (rtl_sdr pipe +
GFSK decode), and the HIL closed loop (RocketPy ↔ Teensy/fake). The line
protocol (>key:value / !key:value / #LEVEL:), worker signals, and
configure()/start()/stop() signatures are the integration contract and are
unchanged. No theming lives here.
"""

from __future__ import annotations

import csv
import shutil
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtCore import QThread, pyqtSignal

from radio_gfsk_rx import (
    DEFAULT_FREQ_HZ,
    DEFAULT_SAMPLE_RATE_HZ,
    DecodeStats,
    FRAME_TYPE_FLIGHT,
    FRAME_TYPE_HK,
    find_frames,
    load_u8_iq,
)

from .constants import FFT_BINS, SPECTRUM_AVG, SPECTRUM_FPS
from .paths import _SIM_ROOT


class SerialWorker(QThread):
    line_received  = pyqtSignal(str)
    status_changed = pyqtSignal(str, bool)   # (message, is_error)
    reconnected    = pyqtSignal()            # Teensy reset detected

    def __init__(self):
        super().__init__()
        self._port     = ""
        self._baud     = 115200
        self._running  = False
        self._ser: serial.Serial | None = None

    def configure(self, port: str, baud: int):
        self._port = port
        self._baud = baud

    def send_bytes(self, data: bytes):
        """Write bytes to the serial port. Thread-safe (pyserial acquires its own lock)."""
        ser = self._ser
        if ser is not None and ser.is_open:
            try:
                ser.write(data)
            except serial.SerialException:
                pass

    def _read_loop(self, ser):
        """Inner read loop. Returns True if we should reconnect, False to exit."""
        buf = ""
        while self._running:
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException:
                return True   # device disconnected — try to reconnect
            if chunk:
                buf += chunk.decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.rstrip("\r")
                    if line:
                        self.line_received.emit(line)
        return False

    def run(self):
        self._running = True
        first_connect = True

        while self._running:
            self.status_changed.emit(
                f"{'Connecting' if first_connect else 'Reconnecting'} to {self._port}…", False)
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.05)
                self._ser = ser
                self.status_changed.emit(f"Connected  {self._port} @ {self._baud}", False)
                if not first_connect:
                    self.reconnected.emit()
                first_connect = False

                should_reconnect = self._read_loop(ser)
                self._ser = None
                ser.close()

                if should_reconnect and self._running:
                    self.status_changed.emit(
                        f"Lost connection — waiting for {self._port}…", False)
                    # Wait for the port to reappear (Teensy re-enumerates after reset)
                    while self._running:
                        time.sleep(0.3)
                        ports = [p.device for p in serial.tools.list_ports.comports()]
                        if self._port in ports:
                            break
            except serial.SerialException:
                if self._running:
                    # Port not available yet — keep polling
                    time.sleep(0.5)

        self._ser = None
        self._running = False

    def stop(self):
        self._running = False
        self.wait(3000)


class ReplayWorker(QThread):
    """Replay a decoded flight-log CSV through the >key:value pipeline so a
    recorded flight scrolls across the Sensors page exactly like a live one.

    Emits the same line protocol SerialWorker does — it plugs into _on_line with
    no special casing — paced by the log's own sample timestamps × a speed
    multiplier (0 = as fast as possible). Long idle gaps (the armed pad sit) are
    compressed to _MAX_GAP_S so playback stays watchable.
    """

    line_received  = pyqtSignal(str)
    status_changed = pyqtSignal(str, bool)   # (message, is_error)
    progress       = pyqtSignal(float)       # 0..1 fraction streamed

    # CSV column -> >plot key (Sensors-page PLOT_GROUPS + GPS).
    _PLOT_MAP = {
        "alt_m": "alt_agl", "vel_mps": "velocity", "pred_apogee_m": "pred_apogee",
        "vert_accel_mps2": "vert_accel", "ax_mss": "accel_x", "ay_mss": "accel_y",
        "az_mss": "accel_z", "gx_rads": "gyro_x", "gy_rads": "gyro_y",
        "gz_rads": "gyro_z", "highg_x_mss": "highg_x", "baro_temp_c": "baro_temp",
        "deploy": "deployment", "gps_alt_msl_m": "gps_alt_msl",
        "gps_lat_deg": "gps_lat", "gps_lon_deg": "gps_lon",
    }
    # CSV column -> !state key (emitted only when the value changes).
    _STATE_MAP = {"phase": "phase", "gps_fix": "gps_fix",
                  "gps_sats": "gps_sats", "storage_health": "health"}
    _MAX_GAP_S = 1.0

    def __init__(self):
        super().__init__()
        self._path = ""
        self._speed = 1.0
        self._running = False

    def configure(self, csv_path, speed: float = 1.0):
        self._path = str(csv_path)
        self._speed = max(0.0, float(speed))

    def run(self):
        self._running = True
        try:
            with open(self._path, newline="") as f:
                rows = list(csv.DictReader(f))
        except OSError as exc:
            self.status_changed.emit(f"Replay: cannot open log — {exc}", True)
            self._running = False
            return

        name = Path(self._path).stem
        total = len(rows) or 1
        speed_txt = "max" if self._speed == 0 else f"{self._speed:g}×"
        self.status_changed.emit(f"Replaying {name} ({speed_txt})", False)

        prev_ms = None
        last_state: dict = {}
        for i, row in enumerate(rows):
            if not self._running:
                break

            ts = row.get("sample_ms") or row.get("time_ms") or ""
            try:
                cur_ms = float(ts)
            except ValueError:
                cur_ms = prev_ms if prev_ms is not None else 0.0

            # Pace against the log's own timeline (compressing long gaps).
            if prev_ms is not None and self._speed > 0:
                dt = (cur_ms - prev_ms) / 1000.0 / self._speed
                if dt > 0:
                    time.sleep(min(dt, self._MAX_GAP_S))
            prev_ms = cur_ms

            rtype = (row.get("record_type") or "").upper()
            if rtype == "EVENT":
                ev = (row.get("event") or "").strip()
                detail = (row.get("event_detail") or "").strip()
                self.line_received.emit(
                    f"#INFO: [{cur_ms / 1000:.1f}s] EVENT {ev} {detail}".rstrip())
                continue
            if rtype and rtype != "SAMPLE":
                continue   # BOOT record etc.

            for col, key in self._PLOT_MAP.items():
                v = row.get(col)
                if v in (None, "", "nan"):
                    continue
                try:
                    self.line_received.emit(f">{key}:{float(v):.4f}")
                except ValueError:
                    pass

            baro = row.get("baro_pa")
            if baro not in (None, "", "nan"):
                try:
                    pa = float(baro)
                    self.line_received.emit(f">baro_pa:{pa:.0f}")
                    if pa > 0:
                        alt = 44330.0 * (1.0 - (pa / 101325.0) ** (1.0 / 5.255))
                        self.line_received.emit(f">baro_alt:{alt:.1f}")
                except ValueError:
                    pass

            for col, key in self._STATE_MAP.items():
                v = row.get(col)
                if v in (None, ""):
                    continue
                v = v.strip()
                if last_state.get(key) != v:
                    last_state[key] = v
                    self.line_received.emit(f"!{key}:{v}")

            if i % 50 == 0:
                self.progress.emit(i / total)

        self.progress.emit(1.0)
        if self._running:
            self.status_changed.emit(f"Replay complete — {name}", False)
        self._running = False

    def stop(self):
        self._running = False
        self.wait(3000)


class RadioWorker(QThread):
    line_received  = pyqtSignal(str)
    # Telemetry data lines carry the frame's true reception time (monotonic s),
    # back-computed from its sample position — frames are decoded in 0.5 s
    # batches, and batch-time stamps would collapse 5 frames onto one plot x.
    line_received_at = pyqtSignal(str, float)
    status_changed = pyqtSignal(str, bool)
    spectrum_ready = pyqtSignal(object)   # float32 dB array, FFT_BINS long, DC-centered

    def __init__(self):
        super().__init__()
        self._running = False
        self._proc: subprocess.Popen | None = None
        self._freq_hz = DEFAULT_FREQ_HZ
        self._sample_rate = DEFAULT_SAMPLE_RATE_HZ
        self._gain = "10"
        self._ppm = 0
        self._device = "0"
        self._packet_count = 0
        self._seen_frames: dict[tuple, float] = {}   # (ftype, seq, payload) → last-seen
        self._recent_seqs: deque = deque(maxlen=50)    # for packet-loss %
        self._frame_times: deque = deque(maxlen=20)    # reception times, for rate
        self._last_telem_log_s = 0.0
        self._last_radio_diag_s = 0.0
        self._last_spec_s = 0.0
        self._decode_busy = False
        self._fft_window = np.hanning(FFT_BINS).astype(np.float32)
        # Hann window power normalization (sum of squares)
        self._win_norm = float(np.sum(self._fft_window ** 2))

    def configure(self, freq_hz: int, sample_rate: int, gain: str, ppm: int, device: str):
        self._freq_hz = freq_hz
        self._sample_rate = sample_rate
        self._gain = gain.strip() or "auto"
        self._ppm = ppm
        self._device = device.strip() or "0"

    def run(self):
        rtl_sdr = shutil.which("rtl_sdr")
        if rtl_sdr is None:
            self.status_changed.emit("rtl_sdr not found — install rtl-sdr or check PATH", True)
            return

        cmd = [
            rtl_sdr,
            "-d", self._device,
            "-f", str(self._freq_hz),
            "-s", str(self._sample_rate),
            "-p", str(self._ppm),
            # Small output blocks — rtl_sdr's default is 256 KiB, which at low
            # sample rates means one write every ~0.5 s and a ~2 fps spectrum.
            "-b", "8192",
            "-",
        ]
        if self._gain.lower() != "auto":
            cmd[1:1] = ["-g", self._gain]

        self._running = True
        self._packet_count = 0
        self._seen_frames.clear()
        self._recent_seqs.clear()
        self._frame_times.clear()

        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except OSError as exc:
            self.status_changed.emit(f"Failed to start rtl_sdr: {exc}", True)
            self._running = False
            return

        self.status_changed.emit(
            f"RTL-SDR RX {self._freq_hz / 1e6:.3f} MHz @ {self._sample_rate} S/s gain={self._gain}",
            False,
        )
        self.line_received.emit(
            "#INFO: radio rx waiting for 2-GFSK packets "
            "(TELEM_ON telemetry or RADIO_DATA_TEST; RADIO_MARKER is CW spectrum-only)")

        raw = bytearray()
        max_bytes = int(self._sample_rate * 2 * 4)   # unsigned 8-bit IQ, ~4 s rolling window
        min_bytes = int(self._sample_rate * 2 * 1)   # GFSK frames are ~22 ms; 1 s is plenty
        read_size = 65536
        last_decode_s = 0.0

        try:
            while self._running:
                if self._proc.stdout is None:
                    break
                # read1 returns as soon as any data is available — a plain
                # read(n) would block until n bytes arrive, capping the
                # spectrum rate at the pipe's block cadence.
                chunk = self._proc.stdout.read1(read_size)
                if not chunk:
                    if self._proc.poll() is not None:
                        break
                    time.sleep(0.05)
                    continue

                raw.extend(chunk)
                if len(raw) > max_bytes:
                    del raw[:len(raw) - max_bytes]

                now = time.monotonic()

                spec_bytes = FFT_BINS * 2 * SPECTRUM_AVG
                if len(raw) >= spec_bytes and now - self._last_spec_s >= 1.0 / SPECTRUM_FPS:
                    self._last_spec_s = now
                    self._emit_spectrum(raw[-spec_bytes:])

                # Decode runs on its own thread — it chews through the whole
                # rolling buffer (~seconds of CPU) and must never stall reads,
                # or the rtl_sdr pipe backs up and the spectrum stutters.
                if (len(raw) >= min_bytes and now - last_decode_s >= 0.25
                        and not self._decode_busy):
                    last_decode_s = now
                    self._decode_busy = True
                    threading.Thread(
                        target=self._decode_job, args=(bytes(raw), now),
                        daemon=True,
                    ).start()
        finally:
            self._terminate_proc()
            self._running = False

    def _decode_job(self, raw: bytes, now_s: float):
        try:
            self._try_decode(raw, now_s)
        finally:
            self._decode_busy = False

    def _emit_spectrum(self, tail: bytearray):
        """Averaged power spectrum of the newest IQ samples (runs in worker thread)."""
        iq = np.frombuffer(tail, dtype=np.uint8).astype(np.float32)
        iq = (iq - 127.5) * (1.0 / 127.5)
        samples = iq[0::2] + 1j * iq[1::2]

        segs = samples[: FFT_BINS * SPECTRUM_AVG].reshape(SPECTRUM_AVG, FFT_BINS)
        spec = np.fft.fft(segs * self._fft_window, axis=1)
        power = np.mean(np.abs(spec) ** 2, axis=0) / self._win_norm
        db = 10.0 * np.log10(power + 1e-12)
        self.spectrum_ready.emit(np.fft.fftshift(db).astype(np.float32))

    def _try_decode(self, raw: bytes, now_s: float):
        stats = DecodeStats()
        try:
            samples = load_u8_iq(raw)
            frames = find_frames(samples, self._sample_rate, stats=stats)
        except Exception as exc:
            self.line_received.emit(f"#WARN: radio rx decode error: {exc}")
            return

        if not frames and stats.candidates and now_s - self._last_radio_diag_s >= 3.0:
            self._last_radio_diag_s = now_s
            detail = (
                f"#WARN: radio rx saw sync-like energy but no valid packets: "
                f"candidates={stats.candidates} bad_crc={stats.bad_crc} "
                f"unknown_type={stats.unknown_type} best_q={stats.best_quality:.2f}")
            if stats.best_type is not None:
                detail += f" best_type=0x{stats.best_type:02X}"
            if stats.best_crc_rx is not None and stats.best_crc_calc is not None:
                detail += f" crc_rx=0x{stats.best_crc_rx:04X} crc_calc=0x{stats.best_crc_calc:04X}"
            self.line_received.emit(detail)

        # The rolling buffer re-decodes each frame for several seconds —
        # report each (seq, payload) once, then age the dedupe entries out.
        for key, seen_s in list(self._seen_frames.items()):
            if now_s - seen_s > 30.0:
                del self._seen_frames[key]

        n_samples = len(samples)
        for fr in frames:
            key = (fr.ftype, fr.seq, fr.payload)
            if key in self._seen_frames:
                self._seen_frames[key] = now_s
                continue
            self._seen_frames[key] = now_s
            self._packet_count += 1

            # True reception time of this frame (monotonic): the buffer
            # snapshot was taken at now_s and ends "now".
            t_frame = now_s - (n_samples - fr.sample_index) / self._sample_rate

            if fr.ftype in (FRAME_TYPE_FLIGHT, FRAME_TYPE_HK):
                self._emit_telemetry(fr, now_s, t_frame)
            else:
                try:
                    text = fr.payload.decode("ascii")
                except UnicodeDecodeError:
                    text = fr.payload.hex(" ")
                self.line_received.emit(
                    f"#INFO: radio test frame seq={fr.seq} payload=\"{text}\" crc=OK "
                    f"quality={fr.quality:.2f} offset={fr.freq_offset_hz / 1e3:+.2f} kHz"
                )

            self.line_received_at.emit(f">radio_packet_count:{self._packet_count}", t_frame)
            self.line_received_at.emit(f">radio_rx_quality:{fr.quality:.3f}", t_frame)
            self.line_received_at.emit(f">radio_freq_offset_khz:{fr.freq_offset_hz / 1e3:.3f}", t_frame)
            self.line_received.emit("!radio_rx:1")

    def _track_loss(self, seq: int) -> float:
        """Packet-loss % over the last ~50 frames, from seq-number gaps."""
        if self._recent_seqs and abs(seq - self._recent_seqs[-1]) > 1000:
            self._recent_seqs.clear()   # counter reset (reboot / u16 wrap)
        self._recent_seqs.append(seq)
        if len(self._recent_seqs) < 2:
            return 0.0
        span = max(self._recent_seqs) - min(self._recent_seqs) + 1
        return max(0.0, 100.0 * (1.0 - len(set(self._recent_seqs)) / span))

    def _emit_telemetry(self, fr, now_s: float, t_frame: float):
        """Route a FLIGHT/HK frame into the plot/state pipeline, stamped with
        its true reception time so plot points land where the frame arrived."""
        t = fr.parse()
        emit = lambda line: self.line_received_at.emit(line, t_frame)
        loss = self._track_loss(t['seq'])
        emit(f">packet_loss_pct:{loss:.1f}")

        # Link panel stats — measured packet rate from true reception times
        self._frame_times.append(t_frame)
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            rate = (len(self._frame_times) - 1) / span if span > 0 else 0.0
        else:
            rate = 0.0
        emit(f"!rx_seq:{t['seq']}")
        emit(f"!rx_rate:{rate:.1f} Hz")
        emit(f"!rx_loss:{loss:.1f} %")
        emit(f"!rx_quality:{fr.quality:.2f}")
        emit(f"!rx_offset:{fr.freq_offset_hz / 1e3:+.2f} kHz")
        emit(f"!rx_count:{self._packet_count}")

        if fr.ftype == FRAME_TYPE_HK:
            for axis in "xyz":
                emit(f">mag_{axis}:{t[f'mag_{axis}_gauss']:.4f}")
                emit(f">highg_{axis}:{t[f'highg_{axis}_mss']:.2f}")
            emit(f">gyro_x:{t['gyro_x_rads']:.4f}")
            emit(f">gyro_y:{t['gyro_y_rads']:.4f}")
            return

        # FLIGHT frame — same keys the USB serial stream uses, plus extras
        baro_alt = 44330.0 * (1.0 - (max(t['baro_pa'], 1.0) / 101325.0) ** (1.0 / 5.255))
        emit(f">alt_agl:{t['alt_agl_m']:.2f}")
        emit(f">baro_alt:{baro_alt:.1f}")
        emit(f">baro_pa:{t['baro_pa']:.0f}")
        emit(f">baro_temp:{t['baro_temp_c']:.1f}")
        emit(f">velocity:{t['velocity_mps']:.3f}")
        emit(f">pred_apogee:{t['pred_apogee_m']:.1f}")
        emit(f">vert_accel:{t['vert_accel_mps2']:.3f}")
        emit(f">accel_z:{t['accel_z_mss']:.3f}")
        emit(f">roll_rate:{t['roll_rate_rads']:.4f}")
        emit(f">tilt_deg:{t['tilt_deg']:.1f}")
        emit(f">azimuth_deg:{t['azimuth_deg']:.1f}")
        emit(f">deployment:{t['deployment_frac']:.3f}")
        emit(f">gps_alt_msl:{t['gps_alt_msl_m']:.1f}")
        emit(f">gps_lat:{t['gps_lat_deg']:.6f}")
        emit(f">gps_lon:{t['gps_lon_deg']:.6f}")
        emit(f"!phase:{t['phase_name']}")
        emit(f"!health:{t['health']}")
        emit(f"!airbrakes_authorized:{int(t['airbrakes_authorized'])}")
        emit(f"!servo_powered:{int(t['servo_powered'])}")
        emit(f"!arm_switches_closed:{int(t['arm_switches_closed'])}")
        emit(f"!logging_ready:{int(t['logging_ready'])}")
        emit(f"!gps_time_valid:{int(t['gps_time_valid'])}")
        emit(f"!gps_healthy:{int(t['gps_healthy'])}")
        emit(f"!radio_healthy:{int(t['radio_healthy'])}")
        emit(f"!qspi_healthy:{int(t['qspi_healthy'])}")
        emit(f"!sd_healthy:{int(t['sd_healthy'])}")
        emit(f"!gps_fix:{t['gps_fix']}")
        emit(f"!gps_sats:{t['gps_sats']}")
        emit("!radio:0")   # receiving telemetry proves the TX radio is alive

        # Log line rate-limited — telemetry arrives continuously
        if now_s - self._last_telem_log_s >= 5.0:
            self._last_telem_log_s = now_s
            emit(
                f"#INFO: telemetry {t['callsign']} seq={t['seq']} phase={t['phase_name']} "
                f"alt={t['alt_agl_m']:.1f}m gps={t['gps_sats']}sats "
                f"quality={fr.quality:.2f} offset={fr.freq_offset_hz / 1e3:+.2f} kHz"
            )

    def _terminate_proc(self):
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)

    def stop(self):
        self._running = False
        self._terminate_proc()
        self.wait(3000)


# ─── HIL worker ───────────────────────────────────────────────────────────────

class _HilAborted(Exception):
    """Raised from the tick callback to unwind RocketPy when the user stops."""


class HilWorker(QThread):
    """Runs the closed-loop HIL session (apex_sim.hil.runner) in a thread.

    Every controller tick is translated into the app's native line
    protocol and emitted as one batch, so the existing routing, plots, state
    panel, and log handle HIL data exactly like the other sources. The
    callback runs inside the 100 Hz control loop — it only formats strings
    and emits one queued signal.
    """

    lines_received = pyqtSignal(list)        # batch of protocol lines per tick
    status_changed = pyqtSignal(str, bool)   # (message, is_error)

    def __init__(self):
        super().__init__()
        self._running = False
        self._port = ""
        self._fake = True
        self._speed = 1.0
        self._noise = True
        self._record_csv = True
        self._compare_fake = False
        self._sensor_seed = None
        self._sensor_delay_ms = 0.0
        self._full_flight = False
        self._pad_time = 6.0
        self._post_landed_time = 5.0
        self._tick_count = 0
        self._last_gps_fix = 3   # receiver model starts locked on the pad
        self._first_deployment_seen = False

    def configure(self, port: str, fake: bool, speed: float, noise: bool = True,
                  record_csv: bool = True, compare_fake: bool = False,
                  sensor_seed=None, sensor_delay_ms: float = 0.0,
                  full_flight: bool = False, pad_time: float = 6.0,
                  post_landed_time: float = 5.0):
        self._port = port
        self._fake = fake
        self._speed = speed
        self._noise = noise
        self._record_csv = record_csv
        self._compare_fake = compare_fake and not fake
        self._sensor_seed = sensor_seed
        self._sensor_delay_ms = sensor_delay_ms
        self._full_flight = full_flight
        self._pad_time = pad_time
        self._post_landed_time = post_landed_time

    def stop(self):
        self._running = False
        self.wait(5000)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _csv_header(self, compare_fake: bool):
        header = [
            "t_s", "sim_ms", "true_alt_agl_m", "true_vel_z_mps",
            "sim_accel_x_mss", "sim_accel_y_mss", "sim_accel_z_mss",
            "sim_gyro_x_rads", "sim_gyro_y_rads", "sim_gyro_z_rads",
            "sim_baro_pa",
            "est_alt_agl_m", "est_vel_mps", "pred_apogee_m",
            "deployment_frac", "phase", "latency_ms",
        ]
        if compare_fake:
            header.extend([
                "fake_est_alt_agl_m", "fake_est_vel_mps", "fake_pred_apogee_m",
                "fake_deployment_frac", "fake_phase", "fake_latency_ms",
                "fake_minus_primary_alt_m", "fake_minus_primary_vel_mps",
                "fake_minus_primary_deployment",
            ])
        return header

    def _write_csv_row(self, writer, row, compare_fake: bool):
        from apex_sim.hil.protocol import PHASE_NAMES

        if writer is None:
            return
        reply = row.reply
        out = [
            f"{row.t_s:.3f}",
            str(row.sim_ms),
            f"{row.true_alt_agl_m:.2f}",
            f"{row.true_vel_z_mps:.3f}",
            f"{row.sensors.accel_x_mss:.3f}",
            f"{row.sensors.accel_y_mss:.3f}",
            f"{row.sensors.accel_z_mss:.3f}",
            f"{row.sensors.gyro_x_rads:.4f}",
            f"{row.sensors.gyro_y_rads:.4f}",
            f"{row.sensors.gyro_z_rads:.4f}",
            f"{row.sensors.baro_pa:.1f}",
        ]
        if reply is None:
            out.extend(["", "", "", "", "MISS", f"{row.latency_ms:.2f}"])
        else:
            out.extend([
                f"{reply.est_alt_agl_m:.2f}",
                f"{reply.est_vel_mps:.3f}",
                f"{reply.pred_apogee_m:.1f}",
                f"{reply.deployment_frac:.4f}",
                PHASE_NAMES.get(reply.phase, str(reply.phase)),
                f"{row.latency_ms:.2f}",
            ])

        if compare_fake:
            fake_pkt = row.shadow_replies.get("fake")
            if fake_pkt is None:
                out.extend([""] * 9)
            else:
                out.extend([
                    f"{fake_pkt.est_alt_agl_m:.2f}",
                    f"{fake_pkt.est_vel_mps:.3f}",
                    f"{fake_pkt.pred_apogee_m:.1f}",
                    f"{fake_pkt.deployment_frac:.4f}",
                    PHASE_NAMES.get(fake_pkt.phase, str(fake_pkt.phase)),
                    f"{row.shadow_latencies_ms.get('fake', 0.0):.2f}",
                ])
                if reply is None:
                    out.extend(["", "", ""])
                else:
                    out.extend([
                        f"{fake_pkt.est_alt_agl_m - reply.est_alt_agl_m:.2f}",
                        f"{fake_pkt.est_vel_mps - reply.est_vel_mps:.3f}",
                        f"{fake_pkt.deployment_frac - reply.deployment_frac:.4f}",
                    ])
        writer.writerow(out)

    def _emit_tick(self, row, link, last_phase):
        from apex_sim.hil.protocol import PHASE_NAMES

        self._tick_count += 1
        lines = [
            f">true_alt:{row.true_alt_agl_m:.2f}",
            f">true_vel:{row.true_vel_z_mps:.3f}",
            f">sim_accel_x:{row.sensors.accel_x_mss:.3f}",
            f">sim_accel_y:{row.sensors.accel_y_mss:.3f}",
            f">sim_accel_z:{row.sensors.accel_z_mss:.3f}",
            f">sim_gyro_x:{row.sensors.gyro_x_rads:.4f}",
            f">sim_gyro_y:{row.sensors.gyro_y_rads:.4f}",
            f">sim_gyro_z:{row.sensors.gyro_z_rads:.4f}",
            f">sim_baro_pa:{row.sensors.baro_pa:.1f}",
            f">hil_lat_ms:{row.latency_ms:.2f}",
            f"!hil_ticks:{self._tick_count}",
            f"!hil_lat:{row.latency_ms:.1f} ms",
        ]
        if row.reply is not None:
            lines += [
                f">est_alt:{row.reply.est_alt_agl_m:.2f}",
                f">est_vel:{row.reply.est_vel_mps:.3f}",
                f">pred_apogee:{row.reply.pred_apogee_m:.1f}",
                f">deployment:{row.reply.deployment_frac:.4f}",
            ]
            phase = PHASE_NAMES.get(row.reply.phase, "?")
            if phase != last_phase[0]:
                last_phase[0] = phase
                lines.append(f"!phase:{phase}")
                event = {
                    "BOOST": "launch",
                    "COAST": "burnout",
                    "DESCENT": "apogee",
                }.get(phase)
                if event is not None:
                    lines.append(f"!hil_event:{event}")
            if (not self._first_deployment_seen
                    and row.reply.deployment_frac > 0.005):
                self._first_deployment_seen = True
                lines.append("!hil_event:first_airbrake")
        # GPS fix state from the receiver model (drops >4 g, reacquires) —
        # emitted on change so the state-panel badge tracks boost blackouts.
        gps_fix = 3 if row.sensors.gps_valid else 0
        if gps_fix != self._last_gps_fix:
            self._last_gps_fix = gps_fix
            lines.append(f"!gps_fix:{gps_fix}")
            lines.append(f"#INFO: GPS {'fix re-acquired' if gps_fix else 'fix lost (dynamics > 4 g)'}")
        fake_pkt = row.shadow_replies.get("fake")
        if fake_pkt is not None:
            lines += [
                f">fake_est_alt:{fake_pkt.est_alt_agl_m:.2f}",
                f">fake_est_vel:{fake_pkt.est_vel_mps:.3f}",
                f">fake_pred_apogee:{fake_pkt.pred_apogee_m:.1f}",
                f">fake_deployment:{fake_pkt.deployment_frac:.4f}",
                f">fake_hil_lat_ms:{row.shadow_latencies_ms.get('fake', 0.0):.2f}",
            ]
            if row.reply is not None:
                d_alt = fake_pkt.est_alt_agl_m - row.reply.est_alt_agl_m
                d_vel = fake_pkt.est_vel_mps - row.reply.est_vel_mps
                d_dep = fake_pkt.deployment_frac - row.reply.deployment_frac
                lines += [
                    f">fake_delta_alt:{d_alt:.2f}",
                    f">fake_delta_vel:{d_vel:.3f}",
                    f">fake_delta_deployment:{d_dep:.4f}",
                    f"!hil_shadow:dAlt {d_alt:+.1f} m  dVel {d_vel:+.2f} m/s  dDep {d_dep * 100:+.1f}%",
                ]
        # Forward flight-computer ASCII (#INFO transitions, warnings) live.
        lines.extend(link.drain_lines())
        self.lines_received.emit(lines)

    # ── session ───────────────────────────────────────────────────────────────

    def run(self):
        self._first_deployment_seen = False
        self._running = True
        self._tick_count = 0
        self._last_gps_fix = 3
        fake = None
        fake_shadow = None
        link = None
        shadow_links = {}
        csv_file = None
        csv_writer = None
        out_path = None
        try:
            self.status_changed.emit("HIL: building RocketPy sim…", False)
            import yaml
            from apex_sim.config.loader import load_environment
            from apex_sim.hil.emulator import SensorErrors
            from apex_sim.hil.link import HilLink
            from apex_sim.hil.runner import run_closed_loop
            from apex_sim.sim.environment import build_environment
            from apex_sim.sim.rocket import build_rocket

            env_cfg = load_environment()
            env_cfg.atmosphere.model_override = "standard_atmosphere"
            env = build_environment(env_cfg)
            rocket = build_rocket()
            with (_SIM_ROOT / "config" / "airbrakes.yaml").open() as fh:
                airbrakes_cfg = yaml.safe_load(fh)
            if not self._running:
                raise _HilAborted()

            if self._record_csv:
                out_dir = _SIM_ROOT / "output"
                out_dir.mkdir(parents=True, exist_ok=True)
                out_path = out_dir / f"hil_gui_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                csv_file = out_path.open("w", newline="")
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(self._csv_header(self._compare_fake))
                self.lines_received.emit([f"#INFO: HIL CSV recording -> {out_path}"])
            else:
                self.lines_received.emit(["#INFO: HIL CSV recording disabled"])

            if self._fake:
                from apex_sim.hil.fake_teensy import FakeTeensy
                fake = FakeTeensy()
                port = fake.port
            else:
                port = self._port
            link = HilLink(port)
            if not self._fake:
                link.reset_input()
            self.status_changed.emit(f"HIL: waiting for flight computer on {port}…", False)
            if not link.wait_for_line("READY", timeout=15.0):
                self.status_changed.emit(
                    "HIL: no #HIL_READY — flash the teensy41_hil build", True)
                return
            self.lines_received.emit(["#INFO: HIL flight computer ready"])

            if self._compare_fake:
                from apex_sim.hil.fake_teensy import FakeTeensy
                fake_shadow = FakeTeensy()
                fake_link = HilLink(fake_shadow.port)
                if not fake_link.wait_for_line("READY", timeout=5.0):
                    fake_link.close()
                    self.status_changed.emit("HIL: fake shadow did not become ready", True)
                    return
                shadow_links["fake"] = fake_link
                self.lines_received.emit([
                    "#INFO: HIL fake-shadow comparison enabled (real hardware primary)",
                    "!hil_shadow:waiting for paired packets",
                ])
            else:
                self.lines_received.emit(["#INFO: HIL fake-shadow comparison disabled"])

            self.status_changed.emit("HIL: warming up on pad…", False)
            last_phase = ["ARMED"]
            sensor_kwargs = {
                "seed": self._sensor_seed,
                "errors": SensorErrors(
                    delay_ticks=max(0, int(round(self._sensor_delay_ms / 10.0)))),
            }

            def tick(row):
                if not self._running:
                    raise _HilAborted()
                self._write_csv_row(csv_writer, row, self._compare_fake)
                self._emit_tick(row, link, last_phase)

            # Full flight: fly through descent + landing (needs a long cap so
            # the main chute reaches the ground). Otherwise stop at apogee.
            result = run_closed_loop(
                link, env, rocket, env_cfg, airbrakes_cfg,
                speed=self._speed, warmup_s=self._pad_time,
                terminate_on_apogee=not self._full_flight,
                max_time=600.0 if self._full_flight else 120.0,
                post_landed_s=self._post_landed_time,
                noise=self._noise,
                sensor_kwargs=sensor_kwargs,
                shadow_links=shadow_links,
                tick_cb=tick)

            apogee_ft = (result.flight.apogee - env.elevation) * 3.28084
            replies = [r.reply for r in result.rows if r.reply is not None]
            max_dep = max((r.deployment_frac for r in replies), default=0.0)
            summary = [
                f"#INFO: HIL flight complete — apogee {apogee_ft:.0f} ft AGL, "
                f"max deployment {max_dep * 100:.0f}%",
                f"#INFO: {len(result.rows)} ticks, {result.missed} missed, "
                f"{result.crc_errors} CRC errors",
                f"!hil_ticks:{len(result.rows)}",
                f"!hil_missed:{result.missed}",
                f"!hil_crc:{result.crc_errors}",
            ]
            if self._compare_fake:
                pairs = [(r.reply, r.shadow_replies.get("fake")) for r in result.rows
                         if r.reply is not None and r.shadow_replies.get("fake") is not None]
                if pairs:
                    max_alt_delta = max(abs(f.est_alt_agl_m - p.est_alt_agl_m)
                                        for p, f in pairs)
                    max_vel_delta = max(abs(f.est_vel_mps - p.est_vel_mps)
                                        for p, f in pairs)
                    max_dep_delta = max(abs(f.deployment_frac - p.deployment_frac)
                                        for p, f in pairs)
                    phase_mismatch = sum(1 for p, f in pairs if p.phase != f.phase)
                    summary.extend([
                        f"#INFO: fake shadow delta max |alt|={max_alt_delta:.1f} m, "
                        f"|vel|={max_vel_delta:.2f} m/s, "
                        f"|deploy|={max_dep_delta * 100:.1f}%, "
                        f"phase mismatches={phase_mismatch}/{len(pairs)}",
                        f"!hil_shadow:max dAlt {max_alt_delta:.1f} m  "
                        f"dVel {max_vel_delta:.2f} m/s  dDep {max_dep_delta * 100:.1f}%",
                    ])
                else:
                    summary.append("#WARN: fake shadow comparison produced no paired replies")
            if out_path is not None:
                summary.append(f"#INFO: HIL CSV saved -> {out_path}")
            self.lines_received.emit(summary)
            self.status_changed.emit("HIL: flight complete", False)
        except _HilAborted:
            self.status_changed.emit("HIL: stopped", False)
        except Exception as exc:  # noqa: BLE001 — surface anything in the UI
            self.status_changed.emit(f"HIL error: {exc}", True)
        finally:
            if csv_file is not None:
                csv_file.close()
            if link is not None:
                link.close()
            for shadow in shadow_links.values():
                shadow.close()
            if fake is not None:
                fake.close()
            if fake_shadow is not None:
                fake_shadow.close()
            self._running = False


# ─── Log operations worker ────────────────────────────────────────────────────

class LogOpsWorker(QThread):
    """Runs one log pull/decode/export job off the UI thread.

    MTP pulls can block for minutes (mtp-getfile allows 300 s per file) and
    decoding large logs takes seconds — neither may stall the UI. One job at
    a time; the Logs page disables its action buttons while a job runs
    (spawn-and-skip, same idiom as the SDR decode path).
    """

    progress = pyqtSignal(str)
    done = pyqtSignal(object)      # job return value
    failed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._job = None

    def start_job(self, job) -> bool:
        """job(progress_cb) -> result. Returns False if one is running."""
        if self.isRunning():
            return False
        self._job = job
        self.start()
        return True

    def run(self):
        try:
            self.done.emit(self._job(self.progress.emit))
        except Exception as exc:  # noqa: BLE001 — surfaced in the UI
            self.failed.emit(str(exc))


def flight_summaries(records) -> list:
    """Per-flight rows for the Logs table from decoded LogRecords.

    Returns [(label, boot_id, n_records, start_utc, duration_s, max_alt_m,
    n_events)], flights first, ground groups last.
    """
    groups: dict = {}
    for rec in records:
        groups.setdefault((rec.flight_id, rec.boot_id), []).append(rec)

    rows = []
    for (fid, bid), recs in sorted(groups.items(),
                                   key=lambda kv: (kv[0][0] == 0, kv[0])):
        times = [r.time_ms for r in recs]
        max_alt = None
        start_utc = ""
        n_events = 0
        for r in recs:
            p = r.payload or {}
            alt = p.get("alt_m")
            if alt is not None and (max_alt is None or alt > max_alt):
                max_alt = alt
            if not start_utc and p.get("utc"):
                start_utc = str(p["utc"])
            if r.record_type == 2:   # LOG_REC_EVENT
                n_events += 1
        label = f"Flight {fid}" if fid else "Ground"
        duration_s = (max(times) - min(times)) / 1000.0 if times else 0.0
        rows.append((label, bid, len(recs), start_utc, duration_s,
                     max_alt, n_events))
    return rows



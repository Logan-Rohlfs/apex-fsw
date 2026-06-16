"""Offscreen smoke tests for the HORIZON ground app (scripts/horizon.py).

Run headless: QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/ -q
(Set automatically below.)
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import pytest
from PyQt5.QtWidgets import QApplication

import horizon
from test_log_decoder import _write_log


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture()
def win(app):
    w = horizon.MainWindow()
    w.show()
    app.processEvents()
    yield w
    w.close()
    app.processEvents()


def _wait_for_log_job(app, win, timeout_s: float = 15.0):
    """Wait for the LogOpsWorker job to finish and its signals to deliver."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents()
        if not win._log_ops.isRunning() and not win.log_busy_label.text():
            break
        time.sleep(0.01)
    for _ in range(5):
        app.processEvents()
    assert not win._log_ops.isRunning(), "log job did not finish in time"
    assert not win.log_busy_label.text(), "busy state was not cleared"


# ── Branding ──────────────────────────────────────────────────────────────────

def test_branding(app, win):
    assert "HORIZON" in win.windowTitle()
    titles = [win.tab_bar.tabText(i) for i in range(win.tab_bar.count())]
    assert titles == ["Sensors", "Radio", "HIL", "Logs"]


# ── Line-protocol routing on each data page ──────────────────────────────────

def test_routing_per_page(app, win):
    # Sensors page (serial source)
    win.tab_bar.setCurrentIndex(0)
    app.processEvents()
    win._on_line(">alt_agl:123.4")
    win._on_line("!phase:BOOST")
    win._on_line("#INFO: hello from serial")
    group = win._key_to_group_serial["alt_agl"]
    assert len(group._t["alt_agl"]) == 1
    assert win.state_panel.phase_label.text() == "BOOST"
    assert "hello from serial" in win.log_view.toPlainText()

    # Radio page — same key routes to the radio layout instead
    win.tab_bar.setCurrentIndex(1)
    app.processEvents()
    win._on_line(">alt_agl:200.0")
    radio_group = win._key_to_group_radio["alt_agl"]
    assert len(radio_group._t["alt_agl"]) == 1
    assert len(group._t["alt_agl"]) == 1   # serial buffer untouched

    # HIL page
    win.tab_bar.setCurrentIndex(2)
    app.processEvents()
    win._on_line(">est_alt:55.0")
    hil_group = win._key_to_group_hil["est_alt"]
    assert len(hil_group._t["est_alt"]) == 1

    # Unknown key lands in the overflow group instead of being dropped
    win._on_line(">mystery_key:1.0")
    assert win._overflow_group is not None
    assert len(win._overflow_group._t["mystery_key"]) == 1

    # Logs page builds and shows the pipeline panel
    win.tab_bar.setCurrentIndex(3)
    app.processEvents()
    assert win.log_decode_panel.isVisible()
    assert not win.state_panel.isVisible()


# ── Logs page: device → archive → CSV pipeline ───────────────────────────────

def _mtp_mock(log_path: Path, file_id: int = 42):
    """Fake _run_mtp_tool for the libmtp device path (no real MTP device).

    mtp-files lists log_path; mtp-getfile copies its bytes to the dest;
    mtp-detect (capacity) reports unavailable. The flight computer only
    serves logs over libmtp — mounted volumes are not used."""
    content = log_path.read_bytes()
    listing = (f"File ID: {file_id}\n"
               f"   Filename: {log_path.name}\n"
               f"   File size: {len(content)}\n"
               f"   Storage ID: 0x00010001\n")

    def run(args, timeout_s=30):
        if args[0] == "mtp-files":
            return 0, listing
        if args[0] == "mtp-getfile":
            Path(args[2]).write_bytes(content)
            return 0, ""
        return None, ""          # mtp-detect / anything else: unavailable
    return run


def test_logs_page_end_to_end(app, win, tmp_path, monkeypatch):
    archive = tmp_path / "raw_logs"
    monkeypatch.setattr("horizon_app.paths._RAW_LOG_ARCHIVE", archive)

    # Device file served over libmtp (mtp-files / mtp-getfile mocked).
    log_path = tmp_path / "BOOT_00042.APXLOG"
    _write_log(log_path)
    monkeypatch.setattr(win, "_run_mtp_tool", _mtp_mock(log_path))

    win.tab_bar.setCurrentIndex(3)
    app.processEvents()

    # 1 · Flight Computer — Refresh lists the device file
    win._refresh_device_files()
    _wait_for_log_job(app, win)
    assert win.device_table.rowCount() == 1
    assert win.device_table.item(0, 0).text() == "BOOT_00042.APXLOG"
    assert "KiB" in win.device_table.item(0, 1).text() or \
           "B" in win.device_table.item(0, 1).text()
    # col 2 = archive status (missing until pulled); col 3 = source storage
    # label, here the QSPI flash served over libmtp.
    assert win.device_table.item(0, 2).text() == "missing"
    assert "APEX-FLASH" in win.device_table.item(0, 3).text()

    # Pull Selected copies into the laptop archive
    win.device_table.selectAll()
    win._pull_selected_device_files()
    _wait_for_log_job(app, win)
    pulled = list(archive.rglob("*.APXLOG"))
    assert len(pulled) == 1
    assert pulled[0].read_bytes() == log_path.read_bytes()

    # 2 · Laptop Archive list shows the pulled file with its size
    assert win.local_log_list.count() == 1
    assert "BOOT_00042.APXLOG" in win.local_log_list.item(0).text()
    assert "—" in win.local_log_list.item(0).text()   # size separator present

    # Pulling again skips the unchanged file (size dedupe)
    win._pull_selected_device_files()
    _wait_for_log_job(app, win)
    assert len(list(archive.rglob("*.APXLOG"))) == 1

    # 3 · Export Selected to CSV decodes and fills the flights table
    out_dir = tmp_path / "exports"
    win.log_output_field.setText(str(out_dir))
    win.local_log_list.selectAll()
    win._export_logs(all_files=False)
    _wait_for_log_job(app, win)

    labels = [win.flights_table.item(r, 0).text()
              for r in range(win.flights_table.rowCount())]
    assert "Flight 2" in labels
    csvs = list(out_dir.rglob("*.csv"))
    assert any(p.name.startswith("Flight_02") for p in csvs)

    # Export All works without a selection
    win.local_log_list.clearSelection()
    win._export_logs(all_files=True)
    _wait_for_log_job(app, win)
    assert win.flights_table.rowCount() >= 1


def test_logs_pull_all_without_refresh(app, win, tmp_path, monkeypatch):
    """Pull All discovers and pulls in one job when no Refresh was done."""
    archive = tmp_path / "raw_logs"
    monkeypatch.setattr("horizon_app.paths._RAW_LOG_ARCHIVE", archive)
    log_path = tmp_path / "BOOT_00007.APXLOG"
    _write_log(log_path)
    monkeypatch.setattr(win, "_run_mtp_tool", _mtp_mock(log_path, file_id=7))

    win.tab_bar.setCurrentIndex(3)
    app.processEvents()
    assert win._device_entries == []
    win._pull_all_device_files(export_after=False)
    _wait_for_log_job(app, win)
    assert len(list(archive.rglob("*.APXLOG"))) == 1


# ── Fullscreen perf: offscreen-group refresh skipping ────────────────────────

def test_offscreen_groups_skip_but_repaint_on_return(app, win):
    win.tab_bar.setCurrentIndex(0)
    app.processEvents()
    group = win._key_to_group_serial["alt_agl"]
    key = "alt_agl"

    def n_points():
        # Raw data handed to the curve (getData() would apply clip-to-view)
        xd = group.curves[key].xData
        return 0 if xd is None else len(xd)

    win._on_line(f">{key}:1.0")
    win._refresh_plots()
    assert n_points() == 1

    # Hide the page; data keeps accumulating but no curve work happens
    win.tab_bar.setCurrentIndex(1)
    app.processEvents()
    assert not group.isVisible()
    win.tab_bar.setCurrentIndex(0)   # routing needs the serial page active
    app.processEvents()
    win.tab_bar.setCurrentIndex(1)
    app.processEvents()
    base = time.monotonic() - win._t_start   # same timebase _on_line uses
    for i in range(5):
        group.push(key, base + i * 0.01, float(i))
    win._refresh_plots()
    assert group._was_offscreen        # marked skipped
    assert n_points() == 1             # not repainted while hidden

    # Back on the Sensors page: full repaint with all accumulated points
    win.tab_bar.setCurrentIndex(0)
    app.processEvents()
    win._refresh_plots()
    assert not group._was_offscreen
    assert n_points() == 6

    # Groups scrolled out of the viewport are also skipped
    last_group = win._serial_groups[-1]
    assert last_group.isVisible()
    assert last_group.visibleRegion().isEmpty()
    win._refresh_plots()
    assert last_group._was_offscreen


def test_plot_height_is_bounded(app, win):
    group = win._serial_groups[0]
    assert group.plot.minimumHeight() == 160
    assert group.plot.maximumHeight() == 260

"""Logs tab — the device-file pipeline (Flight Computer → Laptop Archive → CSV
Exports) and all MTP/device/decode/export operations.

Relocated verbatim from horizon.py as a mixin on MainWindow, so every `self.`
reference and method name is preserved (the test suite drives these directly).
Theme colors are qualified to `theme.*`; the heavy device I/O is unchanged.
"""

from __future__ import annotations

import re
import shutil
import subprocess
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtGui import QColor, QDesktopServices
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QLineEdit, QTextEdit,
    QListWidget, QListWidgetItem, QAbstractItemView,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QFileDialog, QMessageBox, QInputDialog,
)

from . import theme
from .theme import mono_font
from .workers import flight_summaries
from .constants import FT_PER_M
from . import paths


class LogPanelMixin:
    def _logs_table_style(self) -> str:
        return (
            f"QTableWidget {{ background:{theme.INSET}; color:{theme.TEXT};"
            f"  border:1px solid {theme.BORDER}; gridline-color:{theme.BORDER}; }}"
            f"QHeaderView::section {{ background:{theme.SURFACE}; color:{theme.TEXT_DIM};"
            f"  border:none; padding:2px 6px; }}")

    @staticmethod
    def _fmt_size(n) -> str:
        if not isinstance(n, (int, float)):
            return "—"
        if n >= 1024 * 1024:
            return f"{n / (1024 * 1024):.1f} MiB"
        if n >= 1024:
            return f"{n / 1024:.1f} KiB"
        return f"{int(n)} B"

    def _build_log_decode_panel(self) -> QWidget:
        """Logs page — a top-to-bottom device-file pipeline in three sections:
        files on the flight computer → raw binaries archived on the laptop →
        decoded CSV exports."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        def section_hint(text: str) -> QLabel:
            hint = QLabel(text)
            hint.setFont(mono_font(8))
            hint.setStyleSheet(f"color:{theme.TEXT_DIM};")
            return hint

        # ── 1 · Flight Computer — .APXLOG files present on the device ───────
        device_box = QGroupBox("1 · Flight Computer  —  files on the device")
        device_layout = QVBoxLayout(device_box)
        device_layout.setContentsMargins(8, 6, 8, 8)
        device_layout.setSpacing(6)
        device_layout.addWidget(section_hint(
            "Raw .APXLOG files on the Teensy's storage. Refresh lists the FC "
            "over MTP and marks which files are already archived locally. Pull "
            "actions skip archived files and only transfer missing APXLOGs."))

        self.device_capacity_label = QLabel("Capacity: —")
        self.device_capacity_label.setFont(mono_font(8))
        self.device_capacity_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        self.device_capacity_label.setWordWrap(True)
        device_layout.addWidget(self.device_capacity_label)

        self.device_table = QTableWidget(0, 4)
        self.device_table.setHorizontalHeaderLabels(["File", "Size", "Local", "Source"])
        self.device_table.verticalHeader().setVisible(False)
        self.device_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.device_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.device_table.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.device_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.device_table.setFont(mono_font(9))
        self.device_table.setFixedHeight(140)
        self.device_table.setStyleSheet(self._logs_table_style())
        device_layout.addWidget(self.device_table)

        device_row = QHBoxLayout()
        device_row.setSpacing(6)
        refresh_device_btn = QPushButton("Refresh")
        refresh_device_btn.setFixedWidth(110)
        refresh_device_btn.clicked.connect(self._refresh_device_files)
        device_row.addWidget(refresh_device_btn)
        device_row.addWidget(QLabel("Storage:"))
        self.device_filter_combo = QComboBox()
        self.device_filter_combo.addItem("All", "all")
        self.device_filter_combo.addItem("QSPI", "qspi")
        self.device_filter_combo.addItem("SD", "sd")
        self.device_filter_combo.setFixedWidth(100)
        self.device_filter_combo.currentIndexChanged.connect(
            self._render_device_table)
        device_row.addWidget(self.device_filter_combo)
        select_all_device_btn = QPushButton("Select All")
        select_all_device_btn.setFixedWidth(110)
        select_all_device_btn.clicked.connect(self.device_table.selectAll)
        device_row.addWidget(select_all_device_btn)
        device_row.addStretch()
        pull_sel_btn = QPushButton("Pull Selected Missing")
        pull_sel_btn.setFixedWidth(170)
        pull_sel_btn.clicked.connect(self._pull_selected_device_files)
        device_row.addWidget(pull_sel_btn)
        pull_all_btn = QPushButton("Pull All Missing")
        pull_all_btn.setFixedWidth(150)
        pull_all_btn.clicked.connect(lambda: self._pull_all_device_files(export_after=False))
        device_row.addWidget(pull_all_btn)
        pull_all_export_btn = QPushButton("Pull Missing + Export")
        pull_all_export_btn.setFixedWidth(180)
        pull_all_export_btn.clicked.connect(lambda: self._pull_all_device_files(export_after=True))
        device_row.addWidget(pull_all_export_btn)
        delete_device_btn = QPushButton("Delete Selected")
        delete_device_btn.setFixedWidth(130)
        delete_device_btn.setToolTip(
            "Delete selected APXLOG files from the flight computer.\n"
            "Requires local-copy checks and typed confirmation.")
        delete_device_btn.clicked.connect(self._delete_selected_device_files)
        device_row.addWidget(delete_device_btn)
        format_qspi_btn = QPushButton("Format QSPI")
        format_qspi_btn.setFixedWidth(120)
        format_qspi_btn.setToolTip(
            "Erase and reformat the flight computer's QSPI flash.\n"
            "Requires local-copy checks and typed confirmation.")
        format_qspi_btn.clicked.connect(self._format_qspi_flash)
        device_row.addWidget(format_qspi_btn)
        device_layout.addLayout(device_row)
        layout.addWidget(device_box)

        # ── 2 · Laptop Archive — raw binaries pulled onto this machine ──────
        archive_box = QGroupBox("2 · Laptop Archive  —  raw .APXLOG binaries on this laptop")
        archive_layout = QVBoxLayout(archive_box)
        archive_layout.setContentsMargins(8, 6, 8, 8)
        archive_layout.setSpacing(6)

        archive_row = QHBoxLayout()
        archive_row.setSpacing(6)
        self.log_archive_label = QLabel(str(paths._RAW_LOG_ARCHIVE))
        self.log_archive_label.setFont(mono_font(9))
        self.log_archive_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        archive_row.addWidget(self.log_archive_label, stretch=1)

        refresh_archive_btn = QPushButton("Refresh")
        refresh_archive_btn.setFixedWidth(110)
        refresh_archive_btn.clicked.connect(lambda: self._refresh_local_log_choices(select_all=False))
        archive_row.addWidget(refresh_archive_btn)

        select_all_btn = QPushButton("Select All")
        select_all_btn.setFixedWidth(110)
        select_all_btn.clicked.connect(self._select_all_local_logs)
        archive_row.addWidget(select_all_btn)

        add_external_btn = QPushButton("Add external…")
        add_external_btn.setFixedWidth(110)
        add_external_btn.setToolTip(
            "Copy .APXLOG files from anywhere on disk into the archive\n"
            "(e.g. logs someone sent you).")
        add_external_btn.clicked.connect(self._add_external_logs)
        archive_row.addWidget(add_external_btn)
        delete_local_btn = QPushButton("Delete Selected Local")
        delete_local_btn.setFixedWidth(150)
        delete_local_btn.setToolTip(
            "Move selected local APXLOG copies to raw_logs_deleted.\n"
            "Requires device-copy checks and typed confirmation.")
        delete_local_btn.clicked.connect(self._delete_selected_local_logs)
        archive_row.addWidget(delete_local_btn)
        archive_layout.addLayout(archive_row)
        archive_layout.addWidget(section_hint(
            "FC pulls are saved under output/raw_logs/flight_computer by "
            "storage/parent/file name. HORIZON still recognizes older archive "
            "copies anywhere under raw_logs and sorts by first valid UTC."))

        self.local_log_list = QListWidget()
        self.local_log_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.local_log_list.setMinimumHeight(110)
        archive_layout.addWidget(self.local_log_list)

        export_row = QHBoxLayout()
        export_row.setSpacing(6)
        export_row.addStretch()
        export_sel_btn = QPushButton("Export Selected to CSV")
        export_sel_btn.setFixedWidth(170)
        export_sel_btn.clicked.connect(lambda: self._export_logs(all_files=False))
        export_row.addWidget(export_sel_btn)
        export_all_btn = QPushButton("Export All")
        export_all_btn.setFixedWidth(130)
        export_all_btn.clicked.connect(lambda: self._export_logs(all_files=True))
        export_row.addWidget(export_all_btn)
        archive_layout.addLayout(export_row)
        layout.addWidget(archive_box)

        # ── 3 · CSV Exports — the decoded, final results ─────────────────────
        exports_box = QGroupBox("3 · CSV Exports  —  decoded flight data (final CSVs)")
        exports_layout = QVBoxLayout(exports_box)
        exports_layout.setContentsMargins(8, 6, 8, 8)
        exports_layout.setSpacing(6)

        output_row = QHBoxLayout()
        output_row.setSpacing(6)
        out_label = QLabel("Export folder:")
        out_label.setFont(mono_font(9))
        out_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        output_row.addWidget(out_label)
        self.log_output_field = QLineEdit(str(paths._SIM_ROOT / "output" / "log_exports"))
        output_row.addWidget(self.log_output_field, stretch=1)
        output_btn = QPushButton("Choose")
        output_btn.setFixedWidth(110)
        output_btn.clicked.connect(self._browse_log_output)
        output_row.addWidget(output_btn)
        open_folder_btn = QPushButton("Open Export Folder")
        open_folder_btn.setFixedWidth(150)
        open_folder_btn.clicked.connect(self._open_export_folder)
        output_row.addWidget(open_folder_btn)
        exports_layout.addLayout(output_row)

        # Per-flight summary of the last decode — one CSV per flight.
        self.flights_table = QTableWidget(0, 7)
        self.flights_table.setHorizontalHeaderLabels(
            ["Flight", "Boot", "Records", "Start (UTC)", "Duration",
             "Max alt", "Events"])
        self.flights_table.verticalHeader().setVisible(False)
        self.flights_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.flights_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.flights_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.flights_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self.flights_table.setFont(mono_font(9))
        self.flights_table.setFixedHeight(150)
        self.flights_table.setStyleSheet(self._logs_table_style())
        exports_layout.addWidget(self.flights_table)
        layout.addWidget(exports_box)

        # Operations log — progress lines from the worker, kept small
        ops_row = QHBoxLayout()
        ops_label = QLabel("Operations log")
        ops_label.setFont(mono_font(8))
        ops_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        ops_row.addWidget(ops_label)
        self.log_busy_label = QLabel("")
        self.log_busy_label.setFont(mono_font(9))
        self.log_busy_label.setStyleSheet(f"color:{theme.AMBER};")
        ops_row.addWidget(self.log_busy_label)
        ops_row.addStretch()
        layout.addLayout(ops_row)

        self.log_decode_view = QTextEdit()
        self.log_decode_view.setReadOnly(True)
        self.log_decode_view.setFont(mono_font(9))
        self.log_decode_view.setMaximumHeight(130)
        self.log_decode_view.setStyleSheet(
            f"background:{theme.INSET}; color:{theme.TEXT}; border:1px solid {theme.BORDER};")
        layout.addWidget(self.log_decode_view, stretch=1)

        self._log_action_buttons = [
            refresh_device_btn, select_all_device_btn, pull_sel_btn,
            pull_all_btn, pull_all_export_btn, delete_device_btn,
            format_qspi_btn, self.device_filter_combo,
            refresh_archive_btn, select_all_btn, add_external_btn,
            delete_local_btn, export_sel_btn, export_all_btn,
        ]
        self._refresh_local_log_choices(select_all=False)
        return panel

    def _populate_flights_table(self, rows: list):
        self.flights_table.setRowCount(len(rows))
        for i, (label, bid, n, utc, dur_s, max_alt, events) in enumerate(rows):
            alt_txt = (f"{max_alt * FT_PER_M:,.0f} ft" if max_alt is not None
                       else "—")
            cells = [label, str(bid), str(n), utc or "—", f"{dur_s:,.1f} s",
                     alt_txt, str(events)]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col >= 1:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.flights_table.setItem(i, col, item)

    def _open_export_folder(self):
        out = self._last_export_dir or Path(
            self.log_output_field.text().strip() or (paths._SIM_ROOT / "output" / "log_exports"))
        if not out.exists():
            self._log(f"[horizon] Export folder does not exist yet: {out}")
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(out)))

    def _set_log_ops_busy(self, busy: bool, msg: str = ""):
        for btn in self._log_action_buttons:
            btn.setEnabled(not busy)
        self.log_busy_label.setText(msg if busy else "")
    def _set_log_decode_text(self, text: str):
        self.log_decode_view.setPlainText(text)
        self.log_decode_view.verticalScrollBar().setValue(
            self.log_decode_view.verticalScrollBar().maximum())

    def _append_log_decode_text(self, text: str):
        current = self.log_decode_view.toPlainText()
        self._set_log_decode_text((current + "\n" if current else "") + text)

    def _local_log_paths(self) -> list[Path]:
        if not paths._RAW_LOG_ARCHIVE.exists():
            return []
        return sorted(
            path for path in paths._RAW_LOG_ARCHIVE.rglob("*")
            if path.is_file() and path.suffix.upper() == ".APXLOG"
        )

    def _local_log_info(self, path: Path) -> dict:
        try:
            stat = path.stat()
            signature = (stat.st_size, stat.st_mtime_ns)
        except OSError:
            signature = (0, 0)
        cache_key = str(path)
        cached = self._local_log_info_cache.get(cache_key)
        if cached and cached[0] == signature:
            return cached[1]

        info = {
            "path": path,
            "size": signature[0],
            "records": 0,
            "boot_ids": [],
            "flight_ids": [],
            "first_utc": None,
            "sort_dt": None,
            "sort_fallback": signature[1],
            "decode_error": None,
        }
        try:
            from apex_sim.logs.decoder import REC_SAMPLE, decode_file
            records, stats = decode_file(path)
            info["records"] = stats.records
            info["boot_ids"] = sorted({r.boot_id for r in records})
            info["flight_ids"] = sorted({r.flight_id for r in records if r.flight_id})
            for record in records:
                if record.record_type != REC_SAMPLE:
                    continue
                utc = record.payload.get("utc")
                if not utc:
                    continue
                info["first_utc"] = utc
                try:
                    info["sort_dt"] = datetime.strptime(utc, "%Y-%m-%dT%H:%M:%S.%fZ")
                except ValueError:
                    pass
                break
        except Exception as exc:  # noqa: BLE001 — shown in item tooltip
            info["decode_error"] = str(exc)

        self._local_log_info_cache[cache_key] = (signature, info)
        return info

    def _format_local_log_item(self, path: Path, info: dict) -> str:
        try:
            rel = path.relative_to(paths._RAW_LOG_ARCHIVE)
        except ValueError:
            rel = path
        if info.get("first_utc"):
            start = str(info["first_utc"]).replace("T", " ").replace(".000Z", "Z")
        else:
            start = "no UTC in data"

        boot_ids = info.get("boot_ids") or []
        flight_ids = info.get("flight_ids") or []
        boot = "boot " + ",".join(str(b) for b in boot_ids) if boot_ids else "boot ?"
        flights = "flight " + ",".join(str(f) for f in flight_ids) if flight_ids else "ground"
        size_txt = self._fmt_size(info.get("size", 0))
        return f"{start}  —  {boot}  —  {flights}  —  {rel}  —  {size_txt}"

    def _refresh_local_log_choices(self, select_all: bool = False,
                                   focus_paths: list[Path] | None = None) -> int:
        focus = {str(path.resolve()) for path in (focus_paths or [])}
        if not focus and not select_all and hasattr(self, "local_log_list"):
            focus = {
                str(Path(item.data(Qt.UserRole)).resolve())
                for item in self.local_log_list.selectedItems()
            }

        paths = self._local_log_paths()
        infos = [(path, self._local_log_info(path)) for path in paths]
        infos.sort(
            key=lambda item: (
                item[1].get("sort_dt") is not None,
                item[1].get("sort_dt") or datetime.fromtimestamp(
                    item[1].get("sort_fallback", 0) / 1_000_000_000),
                item[0].name,
            ),
            reverse=True,
        )
        self.local_log_list.blockSignals(True)
        self.local_log_list.clear()
        for path, info in infos:
            item = QListWidgetItem(self._format_local_log_item(path, info))
            item.setData(Qt.UserRole, str(path))
            tooltip = [
                str(path),
                f"records: {info.get('records', 0)}",
                f"boot_ids: {info.get('boot_ids') or 'unknown'}",
                f"flight_ids: {info.get('flight_ids') or 'none'}",
                f"first_utc: {info.get('first_utc') or 'not found'}",
            ]
            if info.get("decode_error"):
                tooltip.append(f"decode_error: {info['decode_error']}")
            item.setToolTip("\n".join(str(x) for x in tooltip))
            self.local_log_list.addItem(item)
            if select_all or str(path.resolve()) in focus:
                item.setSelected(True)
        self.local_log_list.blockSignals(False)
        if hasattr(self, "device_table") and self._device_entries:
            self._populate_device_table(self._device_entries)
        return len(infos)

    def _select_all_local_logs(self):
        if self.local_log_list.count() == 0:
            self._refresh_local_log_choices(select_all=False)
        self.local_log_list.selectAll()

    def _selected_local_log_paths(self) -> list[Path]:
        return [Path(item.data(Qt.UserRole))
                for item in self.local_log_list.selectedItems()]

    def _local_log_index(self) -> dict[tuple[str, int], list[Path]]:
        index: dict[tuple[str, int], list[Path]] = {}
        for path in self._local_log_paths():
            try:
                size = path.stat().st_size
            except OSError:
                continue
            index.setdefault((path.name, size), []).append(path)
        return index

    @staticmethod
    def _safe_archive_part(value: object, fallback: str) -> str:
        text = str(value or fallback).strip() or fallback
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", text)

    def _entry_archive_path(self, entry: dict, archive: Path = paths._RAW_LOG_ARCHIVE) -> Path:
        filename = Path(str(entry.get("name") or entry.get("filename") or "unknown.APXLOG")).name
        if entry.get("kind") == "mtp":
            storage = self._safe_archive_part(entry.get("storage_id"), "device")
            parent = self._safe_archive_part(entry.get("parent_id"), "root")
            return archive / "flight_computer" / f"storage_{storage}" / f"parent_{parent}" / filename

        src = Path(str(entry.get("src", filename)))
        root = Path(str(entry.get("root", src.parent)))
        try:
            rel = src.relative_to(root)
        except ValueError:
            rel = Path(filename)
        volume = self._safe_archive_part(root.parent.name, "mounted_volume")
        return archive / "flight_computer" / volume / rel

    def _find_local_copy_for_entry(self, entry: dict,
                                   index: dict[tuple[str, int], list[Path]] | None = None) -> Path | None:
        size = entry.get("size")
        expected = self._entry_archive_path(entry)
        if expected.exists():
            if not isinstance(size, int) or expected.stat().st_size == size:
                return expected
        if not isinstance(size, int):
            return None
        name = Path(str(entry.get("name", ""))).name
        matches = (index or self._local_log_index()).get((name, size), [])
        return matches[0] if matches else None

    def _annotate_device_entries(self, entries: list[dict]) -> list[dict]:
        local_index = self._local_log_index()
        annotated: list[dict] = []
        for entry in entries:
            item = dict(entry)
            archive_path = self._entry_archive_path(item)
            local_copy = self._find_local_copy_for_entry(item, local_index)
            item["archive_path"] = str(archive_path)
            item["local_path"] = str(local_copy) if local_copy else ""
            item["local_status"] = "archived" if local_copy else "missing"
            annotated.append(item)
        return annotated

    def _missing_device_entries(self, entries: list[dict]) -> list[dict]:
        local_index = self._local_log_index()
        return [entry for entry in entries
                if self._find_local_copy_for_entry(entry, local_index) is None]

    def _device_entry_matches_local_path(self, entry: dict, path: Path) -> bool:
        try:
            size = path.stat().st_size
        except OSError:
            return False
        return Path(str(entry.get("name", ""))).name == path.name and entry.get("size") == size

    @staticmethod
    def _device_entry_key(entry: dict) -> tuple:
        if entry.get("kind") == "mtp":
            return ("mtp", entry.get("id"))
        return ("volume", entry.get("src"))

    def _confirm_dangerous_delete(self, title: str, body: str,
                                  extra_warning: str = "",
                                  phrase: str = paths._DELETE_CONFIRM_PHRASE) -> bool:
        lines = [
            body,
            "",
            "This cannot happen from one click. The next step requires an exact typed confirmation.",
        ]
        if extra_warning:
            lines.extend(["", extra_warning])
        lines.extend([
            "",
            f'Type exactly: "{phrase}"',
        ])
        QMessageBox.warning(self, title, "\n".join(lines))
        text, ok = QInputDialog.getText(
            self, title, f'Type exactly:\n{phrase}')
        if not ok:
            return False
        return text.strip() == phrase

    def _add_external_logs(self):
        """Copy .APXLOG files from anywhere on disk into the laptop archive."""
        files, _ = QFileDialog.getOpenFileNames(
            self, "Add external Apex binary logs to the archive", str(paths._SIM_ROOT),
            "Apex logs (*.APXLOG *.apxlog);;All files (*)")
        if not files:
            return
        added: list[Path] = []
        skipped = 0
        dest_root = paths._RAW_LOG_ARCHIVE / "external"
        dest_root.mkdir(parents=True, exist_ok=True)
        for item in files:
            src = Path(item)
            dst = dest_root / src.name
            if dst.exists() and dst.stat().st_size == src.stat().st_size:
                skipped += 1
                continue
            dst = self._dedupe_path(dst)
            shutil.copy2(src, dst)
            added.append(dst)
        self._log(f"[horizon] Added {len(added)} external log(s) to the archive"
                  + (f", {skipped} already present" if skipped else ""))
        self._refresh_local_log_choices(focus_paths=added or None)

    def _browse_log_output(self):
        folder = QFileDialog.getExistingDirectory(
            self, "Select log export folder", self.log_output_field.text().strip() or str(paths._SIM_ROOT))
        if folder:
            self.log_output_field.setText(folder)

    @staticmethod
    def _dedupe_path(dst: Path) -> Path:
        """Free destination path — appends _1, _2, … if dst already exists."""
        if not dst.exists():
            return dst
        stem, suffix = dst.stem, dst.suffix
        n = 1
        while True:
            alt = dst.with_name(f"{stem}_{n}{suffix}")
            if not alt.exists():
                return alt
            n += 1

    # ── Device file listing / per-file pulls (run inside LogOpsWorker) ──────
    # Entries are plain dicts so job closures never touch Qt objects:
    #   volume: {"kind","name","size","src","root","source"}
    #   mtp:    {"kind","name","size","id","parent_id","storage_id","source"}

    def _list_device_entries(self, progress) -> tuple[str, list[dict], str, list[dict]]:
        """Worker-thread helper: list .APXLOG files present on the device.
        libmtp only. Returns
        (mode, entries, raw_mtp_listing, capacity rows)."""

        progress("Listing via libmtp (close OpenMTP first)…")
        code, listing = self._run_mtp_tool(["mtp-files"], timeout_s=30)
        mtp_failure = self._mtp_listing_failure(listing)
        if mtp_failure:
            progress("libmtp did not find a usable MTP device.")
            raise RuntimeError(mtp_failure)
        if code == 0:
            capacity = self._mtp_capacity(progress)
            entries = []
            for e in self._parse_mtp_files_output(listing):
                storage_id = e.get("storage_id")
                entries.append({
                    "kind": "mtp",
                    "name": str(e.get("filename", e.get("id"))),
                    "size": e.get("size"),
                    "id": e.get("id"),
                    "parent_id": e.get("parent_id"),
                    "storage_id": storage_id,
                    "source": self._mtp_storage_label(storage_id),
                })
            if entries:
                return "libmtp", entries, listing, capacity
            progress("libmtp listed no APXLOG files.")
            return "libmtp", entries, listing, capacity
        else:
            mtp_error = listing if code is None else (
                listing.strip() or f"mtp-files exited with status {code}")
            progress("libmtp did not list files.")
            raise RuntimeError(mtp_error)

    @staticmethod
    def _mtp_storage_label(storage_id: object) -> str:
        try:
            value = int(str(storage_id), 0)
        except (TypeError, ValueError):
            return f"MTP {storage_id or 'unknown'}"
        store_index = (value >> 16) - 1
        if store_index == 0:
            return "APEX-FLASH (QSPI)"
        if store_index == 1:
            return "APEX-SD"
        return f"MTP storage {storage_id}"

    @staticmethod
    def _mtp_listing_failure(text: str) -> str:
        if not text:
            return ""
        interface_claim_failed = "libusb_claim_interface" in text
        no_device = "No Devices have been found" in text
        failure_patterns = [
            "No Devices have been found",
            "LIBMTP PANIC",
            "Unable to open raw device",
            "Unable to initialize device",
            "libusb_claim_interface",
        ]
        if not any(pattern in text for pattern in failure_patterns):
            return ""
        if interface_claim_failed:
            specific = [
                "libmtp found the Teensy, but macOS/libusb refused to claim the MTP interface.",
                "",
                "Most likely another process has the MTP/PTP interface open. If HORIZON is connected over serial, click Disconnect before refreshing Logs; MTP does not need the serial link. Also close OpenMTP, Android File Transfer, Image Capture, Finder import windows, or any other camera/media-transfer app. If it still sticks, unplug the FC, wait a few seconds, plug it back in, and refresh again.",
            ]
        elif no_device:
            specific = [
                "libmtp did not see any MTP device.",
                "",
                "Make sure the FC is powered, the USB cable supports data, and the firmware was built with USB_MTPDISK_SERIAL.",
            ]
        else:
            specific = [
                "libmtp could not open the flight computer as an MTP device.",
            ]
        hints = [
            *specific,
            "",
            "This is a USB/MTP connection failure, not an empty APXLOG folder.",
            "",
            "Raw mtp-files output:",
            text.strip(),
        ]
        return "\n".join(hints)

    def _mtp_capacity(self, progress) -> list[dict]:
        """Read storage capacity/free-space from mtp-detect if available."""
        code, out = self._run_mtp_tool(["mtp-detect"], timeout_s=30)
        if code is None or code != 0:
            progress("libmtp capacity unavailable — mtp-detect did not complete.")
            return []
        rows = self._parse_mtp_detect_capacity(out)
        if not rows:
            progress("libmtp capacity unavailable — mtp-detect did not report storage sizes.")
        return rows

    @staticmethod
    def _parse_intish(value: str) -> int | None:
        value = value.strip()
        try:
            return int(value, 0)
        except ValueError:
            return None

    def _parse_mtp_detect_capacity(self, text: str) -> list[dict]:
        rows: list[dict] = []
        current: dict[str, object] | None = None

        def finish_current():
            if current and (current.get("total") is not None or current.get("free") is not None):
                rows.append(dict(current))

        for line in text.splitlines():
            storage = re.search(r"\bStorage ID:\s*(0x[0-9a-fA-F]+|\d+)", line, re.IGNORECASE)
            if storage:
                finish_current()
                current = {"storage_id": storage.group(1)}
                continue

            if current is None:
                continue

            desc = re.search(r"\b(?:Storage Description|StorageDescription):\s*(.+?)\s*$",
                             line, re.IGNORECASE)
            if desc:
                current["name"] = desc.group(1).strip()

            total = re.search(r"\b(?:Max Capacity|MaxCapacity):\s*(0x[0-9a-fA-F]+|\d+)",
                              line, re.IGNORECASE)
            if total:
                current["total"] = self._parse_intish(total.group(1))

            free = re.search(r"\b(?:Free Space.*?|FreeSpaceInBytes):\s*(0x[0-9a-fA-F]+|\d+)",
                             line, re.IGNORECASE)
            if free:
                current["free"] = self._parse_intish(free.group(1))

        finish_current()
        return rows

    def _pull_entries(self, entries: list[dict], progress) -> dict:
        """Copy only device logs that are not already present locally."""
        archive = paths._RAW_LOG_ARCHIVE
        paths._FC_LOG_ARCHIVE.mkdir(parents=True, exist_ok=True)
        copied: list[Path] = []
        skipped: list[str] = []
        errors: list[str] = []
        mode = "mounted volume"
        local_index = self._local_log_index()
        pending: list[dict] = []

        for entry in entries:
            existing = self._find_local_copy_for_entry(entry, local_index)
            if existing is not None:
                skipped.append(str(existing))
            else:
                pending.append(entry)

        if not pending:
            progress("All selected FC logs are already archived locally; nothing to pull.")

        for i, entry in enumerate(pending):
            progress(f"Pulling missing log {entry['name']} ({i + 1}/{len(pending)})…")
            dst = self._entry_archive_path(entry, archive)
            dst.parent.mkdir(parents=True, exist_ok=True)

            if entry["kind"] == "volume":
                src = Path(entry["src"])
                expected_size = entry.get("size")
                if dst.exists():
                    if isinstance(expected_size, int) and dst.stat().st_size == expected_size:
                        skipped.append(str(dst))
                        continue
                    dst = self._dedupe_path(dst)
                shutil.copy2(src, dst)
                copied.append(dst)
                local_index.setdefault((dst.name, dst.stat().st_size), []).append(dst)
                continue

            # MTP entry — mtp-getfile can block for minutes per file.
            mode = "libmtp"
            expected_size = entry.get("size")
            if dst.exists():
                if isinstance(expected_size, int) and dst.stat().st_size == expected_size:
                    skipped.append(str(dst))
                    continue
                if expected_size is None:
                    skipped.append(str(dst))
                    continue
                dst = self._dedupe_path(dst)
            tmp = dst.with_suffix(dst.suffix + ".part")
            if tmp.exists():
                tmp.unlink()
            code, out = self._run_mtp_tool(
                ["mtp-getfile", str(entry.get("id")), str(tmp)], timeout_s=300)
            if code is None or code != 0:
                if tmp.exists():
                    tmp.unlink()
                errors.append(
                    f"{entry['name']}: "
                    f"{out.strip() or f'mtp-getfile exited with status {code}'}")
                continue
            if isinstance(expected_size, int) and tmp.stat().st_size != expected_size:
                got = tmp.stat().st_size
                tmp.unlink()
                errors.append(
                    f"{entry['name']}: downloaded {got} bytes, expected {expected_size}")
                continue
            tmp.replace(dst)
            copied.append(dst)
            local_index.setdefault((dst.name, dst.stat().st_size), []).append(dst)

        if errors:
            raise RuntimeError("Some MTP downloads failed:\n" + "\n".join(errors))
        return {"kind": "pull", "mode": mode, "copied": copied,
                "skipped": skipped, "listing": "",
                "n_requested": len(entries), "n_missing": len(pending)}

    def _trash_local_paths(self, paths: list[Path], progress) -> dict:
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        trash_root = paths._DELETED_LOG_ARCHIVE / stamp
        moved: list[tuple[Path, Path]] = []
        errors: list[str] = []
        for i, src in enumerate(paths):
            progress(f"Moving local log {src.name} ({i + 1}/{len(paths)})…")
            try:
                if not src.exists():
                    errors.append(f"{src}: file no longer exists")
                    continue
                try:
                    rel = src.relative_to(paths._RAW_LOG_ARCHIVE)
                except ValueError:
                    rel = Path(src.name)
                dst = self._dedupe_path(trash_root / rel)
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(src), str(dst))
                moved.append((src, dst))
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{src}: {exc}")
        if errors:
            raise RuntimeError("Some local files could not be moved:\n" + "\n".join(errors))
        return {"kind": "delete_local", "moved": moved, "trash_root": trash_root}

    def _delete_device_entries(self, entries: list[dict], progress) -> dict:
        deleted: list[str] = []
        deleted_keys: list[tuple] = []
        rescued: list[Path] = []
        errors: list[str] = []
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        rescue_root = paths._DELETED_LOG_ARCHIVE / stamp / "from_device"

        for i, entry in enumerate(entries):
            progress(f"Deleting device log {entry['name']} ({i + 1}/{len(entries)})…")
            if entry["kind"] == "mtp":
                code, out = self._run_mtp_tool(
                    ["mtp-delfile", "-n", str(entry.get("id"))], timeout_s=60)
                if code is None or code != 0:
                    errors.append(
                        f"{entry['name']}: "
                        f"{out.strip() or f'mtp-delfile exited with status {code}'}")
                    continue
                deleted.append(str(entry["name"]))
                deleted_keys.append(self._device_entry_key(entry))
                continue

            try:
                src = Path(entry["src"])
                root = Path(entry["root"])
                rel = src.relative_to(root)
                dst = self._dedupe_path(rescue_root / root.parent.name / rel)
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(src), str(dst))
                rescued.append(dst)
                deleted.append(str(entry["name"]))
                deleted_keys.append(self._device_entry_key(entry))
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{entry['name']}: {exc}")

        if errors:
            raise RuntimeError("Some device files could not be deleted:\n" + "\n".join(errors))
        return {"kind": "delete_device", "deleted": deleted,
                "deleted_keys": deleted_keys, "rescued": rescued}

    def _run_mtp_tool(self, args: list[str], timeout_s: float) -> tuple[int | None, str]:
        exe = shutil.which(args[0])
        if exe is None:
            return None, f"{args[0]} not found. Install libmtp first, e.g. `brew install libmtp`."
        try:
            proc = subprocess.run(
                [exe, *args[1:]],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            out = exc.stdout or ""
            return None, f"{args[0]} timed out after {timeout_s:.0f}s.\n{out}".strip()
        except OSError as exc:
            return None, str(exc)
        return proc.returncode, proc.stdout or ""

    def _parse_mtp_files_output(self, text: str) -> list[dict[str, object]]:
        files: list[dict[str, object]] = []
        current: dict[str, object] | None = None

        def finish_current():
            if current and str(current.get("filename", "")).upper().endswith(".APXLOG"):
                files.append(dict(current))

        for line in text.splitlines():
            file_id = re.search(r"\bFile ID:\s*(\d+)", line, re.IGNORECASE)
            if file_id:
                finish_current()
                current = {"id": int(file_id.group(1))}

            if current is None:
                continue

            filename = re.search(r"\bFilename:\s*(.+?)\s*$", line, re.IGNORECASE)
            if filename:
                current["filename"] = filename.group(1).strip()

            size = re.search(r"\bFile size:?\s*(\d+)", line, re.IGNORECASE)
            if size:
                current["size"] = int(size.group(1))

            parent = re.search(r"\bParent ID:\s*(\d+)", line, re.IGNORECASE)
            if parent:
                current["parent_id"] = parent.group(1)

            storage = re.search(r"\bStorage ID:\s*(0x[0-9a-fA-F]+|\d+)", line, re.IGNORECASE)
            if storage:
                current["storage_id"] = storage.group(1)

        finish_current()
        return files

    def _start_log_job(self, job, busy_msg: str) -> bool:
        if not self._log_ops.start_job(job):
            self._log("[horizon] A log operation is already running")
            return False
        self._set_log_ops_busy(True, busy_msg)
        self._set_log_decode_text(busy_msg)
        return True

    def _format_qspi_flash(self):
        """Erase/reformat the flight computer QSPI flash via guarded serial command."""
        if not self._worker.isRunning():
            self._append_log_decode_text(
                "Not connected over USB serial.\n"
                "Click Connect (this toolbar) to open the flight computer's USB "
                "serial link, then format QSPI.")
            self._log("[horizon] Format QSPI: connect to the flight computer first")
            return

        local_index = self._local_log_index()
        extra_parts: list[str] = []
        if self._device_entries:
            missing_local = [
                entry for entry in self._device_entries
                if self._find_local_copy_for_entry(entry, local_index) is None
            ]
            if missing_local:
                names = "\n".join(
                    f"  {e['name']} ({self._fmt_size(e.get('size'))})"
                    for e in missing_local)
                extra_parts.append(
                    "WARNING: HORIZON cannot find local archive copies for "
                    "these onboard files. Pull them to the laptop before "
                    "formatting unless you are absolutely sure another copy "
                    f"exists:\n{names}")
        else:
            extra_parts.append(
                "WARNING: The Flight Computer file list is empty or stale. "
                "Click Refresh if you want HORIZON to check for local archive "
                "copies before formatting.")

        extra_parts.append(
            "This runs a full low-level erase of the entire QSPI NAND (every "
            "block, not just the filesystem header), so it also clears any "
            "block-level corruption. It wipes all APXLOG files and boot/flight "
            "counters, then starts a fresh log session. The erase blocks the "
            "flight computer for several seconds.")

        if not self._confirm_dangerous_delete(
            "Format QSPI flash",
            "Full-erase and reformat the flight computer's QSPI flash?",
            "\n\n".join(extra_parts),
            phrase=paths._FORMAT_QSPI_CONFIRM_PHRASE):
            self._set_log_decode_text("QSPI format canceled.")
            return

        self._worker.send_bytes(b"FORMAT_QSPI_ERASE_ALL\n")
        self._append_log_decode_text(
            "Sent FORMAT_QSPI_ERASE_ALL.\n"
            "Watch the log panel for the firmware's proof line:\n"
            "  '#INFO: QSPI LittleFS erased — fresh BOOT_00001 ...'\n"
            "Then click Refresh. APEX-FLASH should contain only the fresh "
            "session; APEX-SD is not erased and may still list older logs.")
        self._log("[horizon] Sent QSPI low-level format command")

    def _refresh_device_files(self):
        """List .APXLOG files present on the flight computer — in the worker
        thread (mtp-files alone can block for tens of seconds)."""

        # The table is only the latest mtp-files snapshot. Clear the previous
        # snapshot while refreshing; never substitute laptop archive entries.
        self._device_entries = []
        self._visible_device_entries = []
        self.device_table.clearContents()
        self.device_table.setRowCount(0)

        def job(progress):
            mode, entries, listing, capacity = self._list_device_entries(progress)
            return {"kind": "device_list", "mode": mode,
                    "entries": entries, "listing": listing,
                    "capacity": capacity}

        self._start_log_job(job, "Listing files on flight computer…")

    def _populate_device_table(self, entries: list[dict]):
        self._device_entries = self._annotate_device_entries(entries)
        self._render_device_table()

    def _render_device_table(self, *_args):
        mode = (self.device_filter_combo.currentData()
                if hasattr(self, "device_filter_combo") else "all")
        if mode == "qspi":
            visible = [entry for entry in self._device_entries
                       if entry.get("source") == "APEX-FLASH (QSPI)"]
        elif mode == "sd":
            visible = [entry for entry in self._device_entries
                       if entry.get("source") == "APEX-SD"]
        else:
            visible = list(self._device_entries)

        self._visible_device_entries = visible
        self.device_table.clearContents()
        self.device_table.setRowCount(len(visible))
        for i, entry in enumerate(visible):
            status = str(entry.get("local_status", "missing"))
            cells = [str(entry["name"]), self._fmt_size(entry.get("size")),
                     status, str(entry.get("source", ""))]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col == 1:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                if col == 2:
                    item.setForeground(QColor(theme.GOOD if status == "archived" else theme.AMBER))
                    tooltip = entry.get("local_path") or entry.get("archive_path") or ""
                    item.setToolTip(str(tooltip))
                self.device_table.setItem(i, col, item)

    def _set_device_capacity(self, rows: list[dict] | None):
        rows = rows or []
        if not rows:
            self.device_capacity_label.setText(
                "Capacity: unavailable from device (QSPI nominal: 128 MiB / 1 Gbit)")
            return
        parts = []
        for row in rows:
            total = row.get("total")
            free = row.get("free")
            used = (total - free
                    if isinstance(total, int) and isinstance(free, int)
                    else None)
            label = str(row.get("name") or row.get("storage_id") or "storage")
            sid = row.get("storage_id")
            if sid and sid != label:
                label = f"{label} ({sid})"
            if isinstance(total, int) and isinstance(free, int):
                parts.append(
                    f"{label}: {self._fmt_size(used)} used / {self._fmt_size(total)} total "
                    f"({self._fmt_size(free)} free)")
            elif isinstance(total, int):
                parts.append(f"{label}: {self._fmt_size(total)} total")
            elif isinstance(free, int):
                parts.append(f"{label}: {self._fmt_size(free)} free")
        self.device_capacity_label.setText("Capacity: " + "   |   ".join(parts))

    def _selected_device_entries(self) -> list[dict]:
        rows = sorted({idx.row() for idx in self.device_table.selectedIndexes()})
        return [self._visible_device_entries[r] for r in rows
                if 0 <= r < len(self._visible_device_entries)]

    def _start_pull_job(self, entries: list[dict], export_after: bool):
        """Pull missing copies of the given device entries into the archive."""
        self._log_export_after_pull = export_after
        snapshot = [dict(e) for e in entries]
        missing = self._missing_device_entries(snapshot)
        if not missing:
            local_paths = [self._find_local_copy_for_entry(entry) for entry in snapshot]
            focus_paths = [path for path in local_paths if path is not None]
            self._set_log_decode_text(
                f"All {len(snapshot)} selected FC log file(s) are already archived locally.\n"
                f"Laptop archive: {paths._RAW_LOG_ARCHIVE}")
            self._populate_device_table(self._device_entries)
            self._refresh_local_log_choices(focus_paths=focus_paths or None)
            if export_after:
                self._log_export_after_pull = False
                self._export_logs(summary_only=False, all_files=False)
            return

        def job(progress):
            return self._pull_entries(snapshot, progress)

        self._start_log_job(job, f"Pulling {len(missing)} missing log(s) from flight computer…")

    def _pull_selected_device_files(self):
        entries = self._selected_device_entries()
        if not entries:
            self._set_log_decode_text(
                "No device files selected — Refresh the Flight Computer list "
                "and select rows first.")
            return
        self._start_pull_job(entries, export_after=False)

    def _pull_all_device_files(self, export_after: bool = False):
        if self._device_entries:
            self._start_pull_job(self._visible_device_entries, export_after)
            return
        # No Refresh yet — discover and pull everything in one job.
        self._log_export_after_pull = export_after

        def job(progress):
            mode, entries, listing, capacity = self._list_device_entries(progress)
            if not entries:
                return {"kind": "pull", "mode": mode, "copied": [],
                        "skipped": [], "listing": listing, "n_requested": 0,
                        "n_missing": 0, "capacity": capacity, "entries": []}
            result = self._pull_entries(entries, progress)
            result["mode"] = mode
            result["listing"] = listing
            result["capacity"] = capacity
            result["entries"] = entries
            return result

        self._start_log_job(job, "Pulling logs from flight computer…")

    def _delete_selected_device_files(self):
        entries = self._selected_device_entries()
        if not entries:
            self._set_log_decode_text(
                "No device files selected — Refresh the Flight Computer list "
                "and select rows first.")
            return

        local_index = self._local_log_index()
        missing_local = [
            entry for entry in entries
            if self._find_local_copy_for_entry(entry, local_index) is None
        ]
        extra = ""
        if missing_local:
            names = "\n".join(f"  {e['name']} ({self._fmt_size(e.get('size'))})"
                              for e in missing_local)
            extra = (
                "WARNING: HORIZON cannot find local archive copies for these "
                "selected device files. Pull them to the laptop before deleting "
                "unless you are absolutely sure another copy exists:\n"
                f"{names}")

        if not self._confirm_dangerous_delete(
            "Delete logs from flight computer",
            f"Delete {len(entries)} selected APXLOG file(s) from the flight computer?",
            extra):
            self._set_log_decode_text("Device delete canceled.")
            return

        snapshot = [dict(e) for e in entries]

        def job(progress):
            return self._delete_device_entries(snapshot, progress)

        self._start_log_job(job, "Deleting logs from flight computer…")

    def _delete_selected_local_logs(self):
        paths = self._selected_local_log_paths()
        if not paths:
            self._set_log_decode_text("No local archive files selected.")
            return
        if not self._device_entries:
            self._set_log_decode_text(
                "Refresh the Flight Computer file list before deleting local logs. "
                "HORIZON needs the current device list to check whether another "
                "copy is still onboard.")
            return

        missing_on_device = [
            path for path in paths
            if not any(self._device_entry_matches_local_path(entry, path)
                       for entry in self._device_entries)
        ]
        extra = ""
        if missing_on_device:
            names = "\n".join(f"  {path.name}" for path in missing_on_device)
            extra = (
                "WARNING: These local files are not currently listed on the "
                "flight computer. Moving them may leave no onboard copy:\n"
                f"{names}")

        if not self._confirm_dangerous_delete(
            "Delete local archive logs",
            f"Move {len(paths)} selected local APXLOG file(s) to the recently deleted folder?",
            extra):
            self._set_log_decode_text("Local delete canceled.")
            return

        snapshot = [Path(p) for p in paths]

        def job(progress):
            return self._trash_local_paths(snapshot, progress)

        self._start_log_job(job, "Moving local logs to recently deleted…")

    def _export_logs(self, summary_only: bool = False, all_files: bool = False):
        # Gather widget state on the UI thread; the job itself must not
        # touch Qt objects.
        used_default = False
        if all_files:
            raw_inputs: list[Path] = self._local_log_paths()
        else:
            raw_inputs = self._selected_local_log_paths()
            if not raw_inputs:
                raw_inputs = self._local_log_paths()
                used_default = True
        if not raw_inputs:
            self._set_log_decode_text(
                "The laptop archive is empty.\n"
                "Pull from the Flight Computer first, or Add external… files.")
            return
        out_dir = Path(self.log_output_field.text().strip()
                       or (paths._SIM_ROOT / "output" / "log_exports"))

        def job(progress):
            from apex_sim.logs.decoder import (
                decode_files, export_logs, iter_log_paths)
            paths = list(iter_log_paths(raw_inputs))
            if not paths:
                raise RuntimeError("No .APXLOG files found in the selected input.")
            progress(f"Decoding {len(paths)} file(s)…")
            records, stats = decode_files(paths)
            rows = flight_summaries(records)
            written = []
            if not summary_only:
                progress("Writing CSV export…")
                written = export_logs(paths, out_dir, include_ground=True)
            return {"kind": "export", "stats": stats, "rows": rows,
                    "written": written, "out_dir": out_dir,
                    "n_paths": len(paths), "summary_only": summary_only,
                    "used_default": used_default,
                    "boots": sorted({r.boot_id for r in records}),
                    "flights": sorted({r.flight_id for r in records
                                       if r.flight_id})}

        self._start_log_job(
            job, "Decoding logs…" if summary_only else "Decoding + exporting…")

    def _on_log_job_failed(self, msg: str):
        self._set_log_ops_busy(False)
        self._log_export_after_pull = False
        self._append_log_decode_text(f"\nFAILED: {msg}")
        self._log(f"[horizon] Log operation failed: {msg}")

    def _on_log_job_done(self, result: dict):
        self._set_log_ops_busy(False)
        if result["kind"] == "device_list":
            entries = result["entries"]
            self._populate_device_table(entries)
            self._set_device_capacity(result.get("capacity"))
            missing = [e for e in self._device_entries if e.get("local_status") != "archived"]
            qspi_count = sum(e.get("source") == "APEX-FLASH (QSPI)"
                             for e in self._device_entries)
            sd_count = sum(e.get("source") == "APEX-SD"
                           for e in self._device_entries)
            lines = [f"Found {len(entries)} .APXLOG file(s) on the device "
                     f"via {result['mode']}.",
                     f"Device storage: QSPI={qspi_count}, SD={sd_count}",
                     f"Local archive: {paths._RAW_LOG_ARCHIVE}",
                     f"Already archived: {len(entries) - len(missing)}; missing: {len(missing)}"]
            if missing:
                lines.append("Missing on laptop:")
                lines.extend(f"  {e['name']} ({self._fmt_size(e.get('size'))})"
                             for e in missing[:20])
                if len(missing) > 20:
                    lines.append(f"  … {len(missing) - 20} more")
            if not entries and result["mode"] == "libmtp":
                lines += ["", "libmtp connected but listed no .APXLOG files.",
                          "Raw mtp-files output:", result["listing"].strip()]
            self._append_log_decode_text("\n".join(lines))
            self._log(f"[horizon] Device listing: {len(entries)} log file(s), "
                      f"{len(missing)} missing locally via {result['mode']}")
            return

        if result["kind"] == "pull":
            copied = result["copied"]
            if "capacity" in result:
                self._set_device_capacity(result.get("capacity"))
            if result.get("entries"):
                self._populate_device_table(result["entries"])
            else:
                self._populate_device_table(self._device_entries)
            lines = [
                f"Pulled logs via {result['mode']}: "
                f"copied {len(copied)} missing file(s), "
                f"skipped {len(result['skipped'])} already archived",
                f"Requested: {result.get('n_requested', 0)}; missing before pull: {result.get('n_missing', len(copied))}",
                f"Laptop archive: {paths._RAW_LOG_ARCHIVE}",
            ]
            if copied:
                lines.append("Copied:")
                lines.extend(f"  {p}" for p in copied)
            elif result["mode"] == "libmtp" and not result["skipped"]:
                lines += ["", "libmtp connected but listed no .APXLOG files.",
                          "Raw mtp-files output:", result["listing"].strip()]
            else:
                lines.append("No transfers needed; every requested FC log already has a local copy.")
            self._set_log_decode_text("\n".join(lines))
            self._log(f"[horizon] Pulled {len(copied)} missing log file(s), "
                      f"skipped {len(result['skipped'])}")
            focus_paths = copied or [Path(p) for p in result["skipped"]]
            self._refresh_local_log_choices(focus_paths=focus_paths or None)
            if self._log_export_after_pull:
                self._log_export_after_pull = False
                self._export_logs(summary_only=False, all_files=False)
            return

        if result["kind"] == "delete_local":
            moved = result["moved"]
            lines = [
                f"Moved {len(moved)} local APXLOG file(s) to recently deleted.",
                f"Recently deleted folder: {result['trash_root']}",
            ]
            if moved:
                lines.append("Moved:")
                lines.extend(f"  {src}  ->  {dst}" for src, dst in moved)
            self._set_log_decode_text("\n".join(lines))
            self._refresh_local_log_choices(select_all=False)
            self._log(f"[horizon] Moved {len(moved)} local log file(s) to recently deleted")
            return

        if result["kind"] == "delete_device":
            deleted = result["deleted"]
            deleted_keys = set(tuple(key) for key in result.get("deleted_keys", []))
            self._device_entries = [
                entry for entry in self._device_entries
                if self._device_entry_key(entry) not in deleted_keys
            ]
            self._populate_device_table(self._device_entries)
            lines = [f"Deleted {len(deleted)} APXLOG file(s) from the flight computer."]
            if result["rescued"]:
                lines.extend([
                    "",
                    "Mounted-volume files were moved into a local rescue folder:",
                    *(f"  {p}" for p in result["rescued"]),
                ])
            if deleted:
                lines.append("")
                lines.append("Deleted:")
                lines.extend(f"  {name}" for name in deleted)
            self._set_log_decode_text("\n".join(lines))
            self._log(f"[horizon] Deleted {len(deleted)} device log file(s)")
            return

        # Export / summary result
        stats = result["stats"]
        self._populate_flights_table(result["rows"])
        lines = [
            f"Decoded {stats.records} records from {result['n_paths']} file(s)"
            + ("  [all local archive logs]" if result["used_default"] else ""),
            f"boot_ids={result['boots']}  flight_ids={result['flights']}",
            f"bad_crc={stats.bad_crc}  truncated={stats.truncated}  "
            f"resync_bytes={stats.resync_bytes}",
        ]
        if not result["summary_only"]:
            self._last_export_dir = result["out_dir"]
            written = result["written"]
            lines.append("")
            if written:
                lines.append(f"Wrote {len(written)} file(s) to {result['out_dir']}:")
                lines.extend(f"  {p}" for p in written)
            else:
                lines.append("No CSV files were written.")
            self._log(f"[horizon] Exported {len(written)} log CSV file(s)")
        else:
            self._log(f"[horizon] Log summary: {stats.records} records, "
                      f"flights={result['flights']}")
        self._set_log_decode_text("\n".join(lines))

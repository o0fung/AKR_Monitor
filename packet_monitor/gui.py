# Copied from previous package; minor docstring tweaks only
"""
Packet Monitor GUI (RR_AKR)
=================================

Purpose
- Real-time visualization of device packets over a serial port (pyserial).
- Fast, low-latency plotting using PyQt6 + pyqtgraph.
- Control tab to send parameter packets back to the device.

Quick start (macOS)
- Install deps: pip install -r requirements.txt
- Run: python -m packet_monitor.gui  (or: python packet_monitor/gui.py)
- Select the serial port (e.g., /dev/tty.usbmodem*, /dev/tty.usbserial*) and baud.
- Press Connect (or hit Enter). Space toggles recording. Backspace pauses chart. Esc exits.

What this shows/records
- Metrics in a grid plus a combined chart for angles (×1), acc (×100), and gyro (×1) stacked with offsets.
- Vertical dashed markers indicate servo state transitions (0→1 blue, 1→0 orange).
- CSV logs are written to logs/data_YYYYMMDD_HHMMSS.csv with core fields (see start_recording()).

Wire protocol at a glance
- Incoming: stream is scanned for [0xFF, 0xFF, PKT_DATA_SIZE]; then PKT_DATA_SIZE bytes are parsed via parse_packet() from cli.py.
- Outgoing (Control tab): a compact parameters frame is built in SerialWorker.send_param_packet() using bit-packed fields (WALK_MODE, flags,
  CPM timings, SERVO DF/PF, ranges, duration). Keep PKT_DATA_SIZE and parse_packet() in sync with firmware.

Extending/maintaining
- To add a plotted/recorded field: add its key to essential_keys, create a label (if needed), and update start_recording() fieldnames.
- Adjust chart scaling/offsets in fixed_scales/group_offsets. Axis tick labels are regenerated on each packet.
- If packets/fields change, update cli.Packet/parse_packet first; this file reads attributes from that object defensively.

Troubleshooting
- No data: check the selected serial port and that baud matches firmware (DEFAULT_BAUDRATE from cli.py). Ensure PKT_DATA_SIZE is correct.
- UI freezes: avoid heavy work in the GUI thread; all serial I/O runs in SerialWorker on a QThread.
- CSV not created: logs/ folder is created next to this file; verify file permissions.

Dependencies
- PyQt6, pyqtgraph, pyserial, numpy (see requirements.txt).

Last updated: 2025-10-15 (renamed package to 'packet_monitor')
"""

import sys
import time
import math
import collections
import struct
import os
import csv
import queue
from typing import Optional, Dict, List, Tuple

try:
    # Prefer same-folder import when running as a script
    from cli import Packet, parse_packet, SERIAL_PORT as SERIAL_PORT_DEFAULT, BAUDRATE as DEFAULT_BAUDRATE, DATA_SIZE as PKT_DATA_SIZE
except Exception:  # pragma: no cover - fallback if executed as a module
    # Optional relative import if the folder is a package
    from .cli import Packet, parse_packet, SERIAL_PORT as SERIAL_PORT_DEFAULT, BAUDRATE as DEFAULT_BAUDRATE, DATA_SIZE as PKT_DATA_SIZE

import serial
from serial.tools import list_ports

from PyQt6.QtCore import QObject, QThread, pyqtSignal, QTimer, Qt, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QTabWidget,
    QGroupBox,
    QFormLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QComboBox,
    QSpinBox,
    QStatusBar,
    QMessageBox,
    QCheckBox,
)
from PyQt6.QtGui import QShortcut, QKeySequence

import pyqtgraph

# The rest of the code is unchanged from the previous version; copied verbatim.
# ...existing code...

# ---------------- Serial worker -----------------
class SerialWorker(QObject):
    packet_received = pyqtSignal(object)  # Packet
    status = pyqtSignal(str)
    connection_lost = pyqtSignal(str)
    stopped = pyqtSignal()

    def __init__(self, port: str, baud: int) -> None:
        super().__init__()
        self._port = port
        self._baud = baud
        self._running = False
        self._ser: Optional[serial.Serial] = None
        # Thread-safe queue for outbound parameter frames
        self._outbox = queue.Queue()

    def stop(self):
        self._running = False

    def start(self):
        self._running = True
        header_window = [0, 0, 0]
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.2)
            self.status.emit(f"Connected to {self._ser.port} @ {self._ser.baudrate} bps")
        except Exception as e:
            self.connection_lost.emit(f"Open port failed: {e}")
            self.stopped.emit()
            return

        try:
            while self._running and self._ser and self._ser.is_open:
                # Send any queued outbound frames first
                try:
                    while not self._outbox.empty():
                        out, packet_dbg = self._outbox.get_nowait()
                        try:
                            if self._ser and self._ser.is_open:
                                self._ser.write(out)
                                self._ser.flush()
                                self.status.emit(f">>> Sent {out!r} {packet_dbg}")
                            else:
                                self.status.emit("Port not open; cannot send.")
                        except Exception as e:
                            self.connection_lost.emit(f"Send failed: {e}")
                except Exception:
                    pass
                if not self._ser.in_waiting:
                    self._ser.timeout = 0.05
                    _ = self._ser.read(1)
                    continue

                b = self._ser.read(1)
                if not b:
                    continue

                header_window[2] = header_window[1]
                header_window[1] = header_window[0]
                header_window[0] = b[0]

                if header_window == [PKT_DATA_SIZE, 255, 255]:
                    packet_bytes = self._ser.read(PKT_DATA_SIZE)
                    if len(packet_bytes) != PKT_DATA_SIZE:
                        continue
                    try:
                        pkt = parse_packet(packet_bytes)
                    except Exception:
                        pkt = None
                    if pkt is None:
                        continue
                    self.packet_received.emit(pkt)

        except Exception as e:
            self.connection_lost.emit(f"Serial error: {e}")
        finally:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
            self.stopped.emit()

    @pyqtSlot(dict)
    def send_param_packet(self, param: Dict[str, int]):
        """Build and enqueue a parameter packet to be sent by the worker thread."""
        try:
            # Bit fields per reference
            temp1 = (
                (param.get('WALK_MODE', 0) & 3)
                | ((param.get('FLAG_EARLY_SWING', 0) & 1) << 2)
                | ((param.get('FLAG_ENABLE_MOTOR', 0) & 1) << 3)
                | ((param.get('FLAG_ENABLE_BUZZER', 0) & 1) << 4)
                | ((param.get('CPM_MODE', 0) & 1) << 5)
                | ((param.get('IS_LEFT', 0) & 1) << 6)
            ) & 0xFF
            temp2 = ((param.get('CPM_DF_DT', 0) & 15) | ((param.get('CPM_DF_WAIT', 0) & 15) << 4)) & 0xFF
            temp3 = ((param.get('CPM_PF_DT', 0) & 15) | ((param.get('CPM_PF_WAIT', 0) & 15) << 4)) & 0xFF
            temp4 = (
                (param.get('RF_PAIR_BTN', 0) & 1)
                | ((param.get('RF_PAIRING', 0) & 1) << 1)
                | ((param.get('CALIBRATE', 0) & 1) << 2)
                | ((param.get('CAL_PWM', 0) & 1) << 3)
                | ((param.get('TRIGGER_DF', 0) & 1) << 4)
                | ((param.get('TRIGGER_PF', 0) & 1) << 5)
                | ((param.get('SHOW_SYSINFO', 0) & 1) << 6)
                | ((param.get('FACTORY_RESET', 0) & 1) << 7)
            ) & 0xFF

            SERVO_DF = param.get('SERVO_DF', 0) & 0xFF
            SERVO_PF = param.get('SERVO_PF', 0) & 0xFF
            CPM_RANGE_DF = param.get('CPM_RANGE_DF', 0) & 0xFF
            CPM_RANGE_PF = param.get('CPM_RANGE_PF', 0) & 0xFF
            CPM_DURATION = param.get('CPM_DURATION', 0) & 0xFF

            checksum = (~(temp1 + temp2 + temp3 + temp4 + SERVO_DF + SERVO_PF + CPM_RANGE_DF + CPM_RANGE_PF + CPM_DURATION)) & 0xFF
            PARAM_SIZE = 10  # number of bytes after this length byte (incl. checksum)

            # Assemble packet: 0xFF 0xFF, PARAM_SIZE, [payload...], checksum
            payload = [temp1, temp2, temp3, SERVO_DF, SERVO_PF, CPM_RANGE_DF, CPM_RANGE_PF, CPM_DURATION, temp4, checksum]
            packet = [255, 255, PARAM_SIZE] + payload

            out = b''.join(struct.pack('B', v & 0xFF) for v in packet)
            # Enqueue for sending in worker loop
            self._outbox.put((out, packet))
        except Exception as e:
            self.connection_lost.emit(f"Send build failed: {e}")


# ---------------- Main window -----------------
class MonitorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Real-Time Device Monitor")

        # Runtime counters
        self.worker_thread: Optional[QThread] = None
        self.worker: Optional[SerialWorker] = None
        self.last_count: Optional[float] = None
        self.packet_counter = 0
        self.pps = 0.0
        # Cache the most recent packet for control tab syncing
        self._latest_pkt: Optional[Packet] = None

        # Transition markers state (servo transitions only)
        self.last_state_gait: Optional[int] = None  # kept for compatibility but unused
        # self.transition_events removed; only using servo events now
        self.last_state_servo: Optional[int] = None
        self.transition_events_servo: collections.deque[Tuple[float, str]] = collections.deque(maxlen=10000)

        # --- Essential status buffers for chart ---
        self.essential_keys: List[str] = [
            "pps", "position", "current", "state_gait",
            "angle_forward", "acc_x", "acc_y", "acc_z",
            "angle_abduction", "gyr_x", "gyr_y", "gyr_z",
        ]
        self.essential_buffers: Dict[str, collections.deque] = {
            k: collections.deque(maxlen=600) for k in self.essential_keys
        }  # ~12s at 50Hz
        self.essential_time_buffer: collections.deque = collections.deque(maxlen=600)
        # Accumulated time in seconds based on pkt.dt (microseconds)
        self._time_accum_s: float = 0.0

        # Central widget/layout
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # --- Connection controls ---
        conn_box = QGroupBox("Connection")
        conn_layout = QHBoxLayout(conn_box)

        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.refresh_ports()

        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(1200, 10_000_000)
        self.baud_spin.setValue(int(DEFAULT_BAUDRATE))

        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.clicked.connect(self.start_connection)
        self.disconnect_btn.clicked.connect(self.stop_connection)

        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo, 1)
        conn_layout.addWidget(self.refresh_btn)
        conn_layout.addWidget(QLabel("Baud:"))
        conn_layout.addWidget(self.baud_spin)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        layout.addWidget(conn_box)

        # Avoid auto-focusing inputs that would consume Spacebar; require click to focus
        self.port_combo.setFocusPolicy(Qt.FocusPolicy.ClickFocus)
        self.baud_spin.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

        # --- Tabs ---
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)
        # Give a neutral initial focus so Spacebar goes to our shortcut
        self.tabs.setFocus()

        # Metrics Grid tab
        metrics_container = QWidget()
        grid = QGridLayout()
        metrics_container.setLayout(grid)
        self.tabs.addTab(metrics_container, "Metrics Grid")

        # Chart & Status tab container and layout
        chart_container = QWidget()
        chart_layout = QVBoxLayout(chart_container)

        plot_grid = QGridLayout()
        plot_box = QGroupBox("Essential Status")
        plot_box.setLayout(plot_grid)
        chart_layout.addWidget(plot_box, 1)

        # Plots dicts and keys
        self.status_plots = {}
        self.essential_plot_curves = {}
        self.combined_keys = [k for k in self.essential_keys if k.startswith(("angle_", "acc_", "gyr_", "state_gait"))]
        self.noncombined_keys = [k for k in self.essential_keys if k not in self.combined_keys]

        # Recording toggle for CSV dump of chart data
        self.recording = False
        self._record_fp = None
        self._record_writer = None
        self._record_start_time = None
        self._record_start_t_accum = None
        # Recording button
        self.btn_record = QPushButton("Start Recording")
        self.btn_record.setCheckable(True)
        self.btn_record.clicked.connect(self.toggle_recording)

        # Pause chart updates (does not stop data streaming/recording)
        self.chart_paused = False
        self.btn_pause_chart = QPushButton("Pause Chart")
        self.btn_pause_chart.setCheckable(True)
        self.btn_pause_chart.clicked.connect(self.toggle_chart_pause)

        record_row = QHBoxLayout()
        record_row.addWidget(self.btn_record)
        record_row.addWidget(self.btn_pause_chart)
        record_row_w = QWidget()
        record_row_w.setLayout(record_row)
        chart_layout.addWidget(record_row_w)

        # Spacebar shortcut to toggle recording regardless of focus
        self.shortcut_record = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        self.shortcut_record.setContext(Qt.ShortcutContext.ApplicationShortcut)
        self.shortcut_record.setAutoRepeat(False)
        self.shortcut_record.activated.connect(self.toggle_recording)

        cols = 4

        # Combined plot for angle/acc/gyr
        self.combined_plot = pyqtgraph.PlotWidget()
        self.combined_plot.setTitle("Angle/Acc/Gyr/State (stacked; angle×1, acc×100, gyr×1, statex10; ±100 range)")
        self.combined_plot.showGrid(x=True, y=True)
        self.combined_plot.addLegend(offset=(10, 10))
        plot_grid.addWidget(self.combined_plot, 0, 0, 1, cols)

        # Create curves for each key, but offsets will be applied per group (angle/acc/gyr)
        self.combined_curves: Dict[str, pyqtgraph.PlotDataItem] = {}
        for i, key in enumerate(self.combined_keys):
            pen = pyqtgraph.mkPen(color=pyqtgraph.intColor(i, hues=len(self.combined_keys)), width=2)
            self.combined_curves[key] = self.combined_plot.plot([], [], pen=pen, name=key)

        # Group-based offsets and scales
        self.group_prefixes: List[str] = ["angle_", "acc_", "gyr_"]
        self.group_offsets: Dict[str, float] = {}
        offset_step = 300.0  # keep groups separated (signals ~±200)
        for gi, pref in enumerate(self.group_prefixes):
            self.group_offsets[pref] = gi * offset_step
        # Fixed scales within a ±100 visual band
        # angle_: ×1 (±100 deg), acc_: ×100 (±1g → ±100), gyr_: ×1 (±100 deg/s)
        self.fixed_scales: Dict[str, float] = {"angle_": 1.0, "acc_": 100.0, "gyr_": 1.0, "state_": 10.0}

        # Add horizontal baseline (solid) and ±100 (dotted) lines for all groups
        self.zero_lines: Dict[str, pyqtgraph.InfiniteLine] = {}
        self.boundary_lines: Dict[str, Tuple[pyqtgraph.InfiniteLine, pyqtgraph.InfiniteLine]] = {}
        for pref in self.group_prefixes:  # ["angle_", "acc_", "gyr_"]
            y0 = self.group_offsets[pref]
            # Zero baseline (solid)
            zero_pen = pyqtgraph.mkPen(color='#666', width=1, style=Qt.PenStyle.SolidLine)
            zero_line = pyqtgraph.InfiniteLine(angle=0, pos=y0, pen=zero_pen, movable=False)
            self.combined_plot.addItem(zero_line)
            self.zero_lines[pref] = zero_line

            # ±100 boundaries (dotted)
            b_pen = pyqtgraph.mkPen(color='#999', width=1, style=Qt.PenStyle.DotLine)
            neg_line = pyqtgraph.InfiniteLine(angle=0, pos=y0 - 100.0, pen=b_pen, movable=False)
            pos_line = pyqtgraph.InfiniteLine(angle=0, pos=y0 + 100.0, pen=b_pen, movable=False)
            self.combined_plot.addItem(neg_line)
            self.combined_plot.addItem(pos_line)
            self.boundary_lines[pref] = (neg_line, pos_line)

        # Fix Y-axis range (no auto-scale) and cache axis for custom ticks
        self.combined_plot.enableAutoRange(axis='y', enable=False)
        self.left_axis = self.combined_plot.getPlotItem().getAxis('left')
        margin = 250.0
        # Angle band is around angle_ offset; dynamic scaling aims to fill ±90 within this band
        minY = self.group_offsets['angle_'] - margin
        maxY = self.group_offsets['gyr_'] + margin
        self.combined_plot.setYRange(minY, maxY, padding=0)

        # Individual plots for noncombined keys
        for idx, key in enumerate(self.noncombined_keys):
            plot = pyqtgraph.PlotWidget()
            plot.setTitle(key.replace("_", " ").title())
            self.status_plots[key] = plot
            self.essential_plot_curves[key] = plot.plot([], [], pen='y')
            r, c = divmod(idx, cols)
            plot_grid.addWidget(plot, r + 1, c)

        # Make the combined plot row taller than the noncombined rows
        nrows_noncombined = (len(self.noncombined_keys) + cols - 1) // cols
        plot_grid.setRowStretch(0, 3)  # combined plot gets more height
        for ri in range(1, nrows_noncombined + 1):
            plot_grid.setRowStretch(ri, 1)

        self.tabs.addTab(chart_container, "Chart & Status")

        # Vertical marker storage on combined plot
        self.marker_lines_combined: List[Tuple[pyqtgraph.InfiniteLine, float, str]] = []

        # Group builders
        self.labels: Dict[str, QLabel] = {}

        def build_group(title: str, rows: List[Tuple[str, str]]):
            gb = QGroupBox(title)
            form = QFormLayout(gb)
            for key, text in rows:
                lab = QLabel("--")
                lab.setObjectName(key)
                lab.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
                form.addRow(QLabel(text), lab)
                self.labels[key] = lab
            return gb

        g_timing = build_group("Timing / Count", [
            ("count", "Count"),
            ("dt", "Δt (raw)"),
            ("delta_count", "Δcount"),
            ("pps", "Packets/s"),
        ])

        g_ang = build_group("Leg Tilting Angle (deg)", [
            ("angle_forward", "Forward Tilting"),
            ("angle_abduction", "Leg Abduction"),
        ])

        g_acc = build_group("Accelerometer (g)", [
            ("acc_x", "Acc X"),
            ("acc_y", "Acc Y"),
            ("acc_z", "Acc Z"),
        ])

        g_gyr = build_group("Gyroscope (deg/s)", [
            ("gyr_x", "Gyr X"),
            ("gyr_y", "Gyr Y"),
            ("gyr_z", "Gyr Z"),
        ])

        g_power = QGroupBox("Power / Battery")
        power_form = QFormLayout(g_power)
        self.labels["current"] = QLabel("--")
        self.labels["battery_state"] = QLabel("--")
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.labels["battery_pct"] = QLabel("--")
        power_form.addRow(QLabel("Current (A)"), self.labels["current"])
        power_form.addRow(QLabel("Batt State"), self.labels["battery_state"])
        power_form.addRow(QLabel("Battery %"), self.battery_bar)
        power_form.addRow(QLabel("Battery Text"), self.labels["battery_pct"])

        g_state = QGroupBox("States / Flags")
        state_layout = QFormLayout(g_state)
        for k in ["state_gait", "state_servo", "state_battery", "walk_mode"]:
            lab = QLabel("--")
            self.labels[k] = lab
            state_layout.addRow(QLabel(k.replace("_", " ").title()), lab)
        # Checkboxes for flags
        self.chk_cal_error = QCheckBox("Calibrate Error")
        self.chk_early_swing = QCheckBox("Early Swing")
        self.chk_motor = QCheckBox("Motor Enabled")
        self.chk_buzzer = QCheckBox("Buzzer Enabled")
        self.chk_cpm_mode = QCheckBox("CPM Mode")
        self.chk_is_left = QCheckBox("Is Left")
        for cb in [self.chk_cal_error, self.chk_early_swing, self.chk_motor,
                   self.chk_buzzer, self.chk_cpm_mode, self.chk_is_left]:
            cb.setEnabled(False)
            state_layout.addRow(cb)

        g_servo = build_group("Servo / Position", [
            ("position", "Position (deg)"),
            ("servo_lo", "Servo Lo"),
            ("servo_hi", "Servo Hi"),
            ("cpm_range_df", "Range DF (%)"),
            ("cpm_range_pf", "Range PF (%)"),
        ])

        g_cpm = build_group("CPM Parameters", [
            ("cpm_count", "CPM Count"),
            ("cpm_duration", "CPM Duration (s)"),
            ("cpm_df_dt", "DF dt"),
            ("cpm_df_wait", "DF wait"),
            ("cpm_pf_dt", "PF dt"),
            ("cpm_pf_wait", "PF wait"),
        ])

        # System Info (last captured when count==0)
        g_sys = build_group("System Info (last)", [
            ("sys_ver", "Version"),
            ("sys_side", "Side"),
            ("sys_df", "DF Range"),
            ("sys_date", "FW Date"),
        ])

        # Place groups in grid
        groups = [g_timing, g_ang, g_acc, g_gyr, g_power, g_state, g_servo, g_cpm, g_sys]
        for gb in groups:
            gb.setMinimumWidth(200)

        r, c = 0, 0
        cols = 4
        for gb in groups:
            grid.addWidget(gb, r, c)
            c += 1
            if c >= cols:
                c = 0
                r += 1

        for i in range(cols):
            grid.setColumnStretch(i, 1)
        row_count = (len(groups) + cols - 1) // cols
        for i in range(row_count):
            grid.setRowStretch(i, 1)

        # --- Control tab (send parameters) ---
        ctrl_container = QWidget()
        form = QFormLayout()
        ctrl_container.setLayout(form)
        tip = QLabel("Tip: Press 'Load From Packet' to populate controls. 'Send Packet' enables after loading.")
        tip.setStyleSheet("color: #666; font-style: italic;")
        form.addRow(tip)

        def make_spin(minv, maxv, val):
            sb = QSpinBox(); sb.setRange(minv, maxv); sb.setValue(val); return sb

        self.in_walk_mode = make_spin(0, 3, 0)
        self.in_early_swing = QCheckBox(); self.in_early_swing.setChecked(False)
        self.in_enable_motor = QCheckBox(); self.in_enable_motor.setChecked(False)
        self.in_enable_buzzer = QCheckBox(); self.in_enable_buzzer.setChecked(False)
        self.in_cpm_mode = QCheckBox(); self.in_cpm_mode.setChecked(False)
        self.in_is_left = QCheckBox(); self.in_is_left.setChecked(False)

        self.in_cpm_df_dt = make_spin(0, 15, 0)
        self.in_cpm_df_wait = make_spin(0, 15, 0)
        self.in_cpm_pf_dt = make_spin(0, 15, 0)
        self.in_cpm_pf_wait = make_spin(0, 15, 0)

        self.in_servo_df = make_spin(0, 255, 0)
        self.in_servo_pf = make_spin(0, 255, 0)
        self.in_cpm_range_df = make_spin(0, 100, 50)
        self.in_cpm_range_pf = make_spin(0, 100, 50)
        self.in_cpm_duration = make_spin(0, 255, 0)

        self.in_rf_pair_btn = QCheckBox()
        self.in_rf_pairing = QCheckBox()
        self.in_calibrate = QCheckBox()
        self.in_cal_pwm = QCheckBox()
        self.in_trigger_df = QCheckBox()
        self.in_trigger_pf = QCheckBox()
        self.in_factory_reset = QCheckBox()
        self.in_show_sysinfo = QCheckBox()

        form.addRow("Walk Mode (0-3)", self.in_walk_mode)
        form.addRow("Early Swing", self.in_early_swing)
        form.addRow("Enable Motor", self.in_enable_motor)
        form.addRow("Enable Buzzer", self.in_enable_buzzer)
        form.addRow("CPM Mode", self.in_cpm_mode)
        form.addRow("Is Left", self.in_is_left)

        form.addRow("CPM DF dt (0-15)", self.in_cpm_df_dt)
        form.addRow("CPM DF wait (0-15)", self.in_cpm_df_wait)
        form.addRow("CPM PF dt (0-15)", self.in_cpm_pf_dt)
        form.addRow("CPM PF wait (0-15)", self.in_cpm_pf_wait)

        form.addRow("SERVO DF (0-255)", self.in_servo_df)
        form.addRow("SERVO PF (0-255)", self.in_servo_pf)
        form.addRow("CPM Range DF %", self.in_cpm_range_df)
        form.addRow("CPM Range PF %", self.in_cpm_range_pf)
        form.addRow("CPM Duration (0-255)", self.in_cpm_duration)

        form.addRow("RF Pair Btn", self.in_rf_pair_btn)
        form.addRow("RF Pairing", self.in_rf_pairing)
        form.addRow("Calibrate", self.in_calibrate)
        form.addRow("Cal PWM", self.in_cal_pwm)
        form.addRow("Trigger DF", self.in_trigger_df)
        form.addRow("Trigger PF", self.in_trigger_pf)
        form.addRow("Factory Reset", self.in_factory_reset)
        form.addRow("Show System Info (once)", self.in_show_sysinfo)

        self.btn_send = QPushButton("Send Packet")
        self.btn_send.setEnabled(False)
        self.btn_load_from_packet = QPushButton("Load From Packet")
        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_load_from_packet)
        btn_row.addWidget(self.btn_send)
        form.addRow(btn_row)

        def gather_params() -> Dict[str, int]:
            return {
                'WALK_MODE': self.in_walk_mode.value(),
                'FLAG_EARLY_SWING': int(self.in_early_swing.isChecked()),
                'FLAG_ENABLE_MOTOR': int(self.in_enable_motor.isChecked()),
                'FLAG_ENABLE_BUZZER': int(self.in_enable_buzzer.isChecked()),
                'CPM_MODE': int(self.in_cpm_mode.isChecked()),
                'IS_LEFT': int(self.in_is_left.isChecked()),
                'CPM_DF_DT': self.in_cpm_df_dt.value(),
                'CPM_DF_WAIT': self.in_cpm_df_wait.value(),
                'CPM_PF_DT': self.in_cpm_pf_dt.value(),
                'CPM_PF_WAIT': self.in_cpm_pf_wait.value(),
                'SERVO_DF': self.in_servo_df.value(),
                'SERVO_PF': self.in_servo_pf.value(),
                'CPM_RANGE_DF': self.in_cpm_range_df.value(),
                'CPM_RANGE_PF': self.in_cpm_range_pf.value(),
                'CPM_DURATION': self.in_cpm_duration.value(),
                'RF_PAIR_BTN': int(self.in_rf_pair_btn.isChecked()),
                'RF_PAIRING': int(self.in_rf_pairing.isChecked()),
                'CALIBRATE': int(self.in_calibrate.isChecked()),
                'CAL_PWM': int(self.in_cal_pwm.isChecked()),
                'TRIGGER_DF': int(self.in_trigger_df.isChecked()),
                'TRIGGER_PF': int(self.in_trigger_pf.isChecked()),
                'SHOW_SYSINFO': int(self.in_show_sysinfo.isChecked()),
                'FACTORY_RESET': int(self.in_factory_reset.isChecked()),
            }

        def on_send():
            if not self.worker:
                QMessageBox.information(self, "Connection", "Not connected.")
                return
            params = gather_params()
            # Direct call enqueues to worker's outbox; serial write happens on worker thread
            self.worker.send_param_packet(params)
            # Auto-reset one-shot sysinfo toggle to avoid repeated triggers
            if self.in_show_sysinfo.isChecked():
                self.in_show_sysinfo.setChecked(False)
            self.set_status("Send attempted")

        self.btn_send.clicked.connect(on_send)

        def apply_from_pkt(pkt: object):
            if pkt is None:
                return
            try:
                self.in_walk_mode.setValue(int(getattr(pkt, 'walk_mode', self.in_walk_mode.value())))
                self.in_early_swing.setChecked(bool(getattr(pkt, 'flag_early_swing', self.in_early_swing.isChecked())))
                self.in_enable_motor.setChecked(bool(getattr(pkt, 'flag_enable_motor', self.in_enable_motor.isChecked())))
                self.in_enable_buzzer.setChecked(bool(getattr(pkt, 'flag_enable_buzzer', self.in_enable_buzzer.isChecked())))
                self.in_cpm_mode.setChecked(bool(getattr(pkt, 'cpm_mode', self.in_cpm_mode.isChecked())))
                self.in_is_left.setChecked(bool(getattr(pkt, 'is_left', self.in_is_left.isChecked())))

                self.in_cpm_df_dt.setValue(int(getattr(pkt, 'cpm_df_dt', self.in_cpm_df_dt.value())))
                self.in_cpm_df_wait.setValue(int(getattr(pkt, 'cpm_df_wait', self.in_cpm_df_wait.value())))
                self.in_cpm_pf_dt.setValue(int(getattr(pkt, 'cpm_pf_dt', self.in_cpm_pf_dt.value())))
                self.in_cpm_pf_wait.setValue(int(getattr(pkt, 'cpm_pf_wait', self.in_cpm_pf_wait.value())))

                self.in_servo_df.setValue(int(getattr(pkt, 'servo_lo', getattr(pkt, 'servo_df', self.in_servo_df.value()))))
                self.in_servo_pf.setValue(int(getattr(pkt, 'servo_hi', getattr(pkt, 'servo_pf', self.in_servo_pf.value()))))

                self.in_cpm_range_df.setValue(int(getattr(pkt, 'cpm_range_df', self.in_cpm_range_df.value())))
                self.in_cpm_range_pf.setValue(int(getattr(pkt, 'cpm_range_pf', self.in_cpm_range_pf.value())))
                self.in_cpm_duration.setValue(int(getattr(pkt, 'cpm_duration', self.in_cpm_duration.value())))

                self.in_rf_pair_btn.setChecked(bool(getattr(pkt, 'rf_pair_btn', self.in_rf_pair_btn.isChecked())))
                self.in_rf_pairing.setChecked(bool(getattr(pkt, 'rf_pairing', self.in_rf_pairing.isChecked())))
                self.in_calibrate.setChecked(bool(getattr(pkt, 'calibrate', self.in_calibrate.isChecked())))
                self.in_cal_pwm.setChecked(bool(getattr(pkt, 'cal_pwm', self.in_cal_pwm.isChecked())))
                self.in_trigger_df.setChecked(bool(getattr(pkt, 'trigger_df', self.in_trigger_df.isChecked())))
                self.in_trigger_pf.setChecked(bool(getattr(pkt, 'trigger_pf', self.in_trigger_pf.isChecked())))
                self.in_factory_reset.setChecked(bool(getattr(pkt, 'factory_reset', self.in_factory_reset.isChecked())))
                self.in_show_sysinfo.setChecked(False)
            except Exception:
                pass

        def on_load_from_packet():
            pkt = getattr(self, '_latest_pkt', None)
            if not pkt:
                QMessageBox.information(self, "Control Sync", "No packet received yet.")
                return
            apply_from_pkt(pkt)
            self.set_status("Loaded control values from last packet")
            self.btn_send.setEnabled(True)

        self.btn_load_from_packet.clicked.connect(on_load_from_packet)

        self.ctrl_tab_index = self.tabs.addTab(ctrl_container, "Control (Send)")

        def on_tab_changed(idx: int):
            if idx == getattr(self, 'ctrl_tab_index', -1):
                self.btn_send.setEnabled(False)
                self.set_status("Load from packet before sending")

        self.tabs.currentChanged.connect(on_tab_changed)

    # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # Timer for PPS update
        self.pps_timer = QTimer(self)
        self.pps_timer.setInterval(1000)
        self.pps_timer.timeout.connect(self.update_pps)
        self.pps_timer.start()

        self.resize(1200, 750)

    # ------------- Connection handling -------------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        current = self.port_combo.currentText() if getattr(self, 'port_combo', None) and self.port_combo.count() else None
        self.port_combo.clear()
        self.port_combo.addItems(ports or [SERIAL_PORT_DEFAULT])
        if SERIAL_PORT_DEFAULT in ports:
            self.port_combo.setCurrentText(SERIAL_PORT_DEFAULT)
        elif current in ports:
            self.port_combo.setCurrentText(current)

    def start_connection(self):
        if self.worker_thread:
            return
        port = self.port_combo.currentText().strip()
        baud = self.baud_spin.value()

        self.worker_thread = QThread()
        self.worker = SerialWorker(port, baud)
        self.worker.moveToThread(self.worker_thread)

        self.worker_thread.started.connect(self.worker.start)
        self.worker.packet_received.connect(self.on_packet)
        self.worker.connection_lost.connect(self.on_connection_lost)
        self.worker.status.connect(self.set_status)
        self.worker.stopped.connect(self.cleanup_thread)
        self.worker_thread.start()

        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.set_status("Connecting...")

    def stop_connection(self):
        if self.worker:
            self.worker.stop()

    def cleanup_thread(self):
        if self.worker_thread:
            self.worker_thread.quit()
            self.worker_thread.wait(1500)
        self.worker_thread = None
        self.worker = None
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Return:
            self.start_connection()
        elif event.key() == Qt.Key.Key_Backspace:
            self.toggle_chart_pause()
        # elif event.key() == Qt.Key.Key_Space:
            # self.toggle_recording()
        elif event.key() == Qt.Key.Key_Escape:
            self.close()

    def closeEvent(self, event):
        self.stop_connection()
        # Ensure recording file is closed cleanly
        try:
            if getattr(self, 'recording', False):
                self.stop_recording()
        except Exception:
            pass
        if self.worker_thread:
            self.worker_thread.quit()
            self.worker_thread.wait(1500)
        event.accept()

    # ------------- Packet handling -------------
    def on_packet(self, pkt: Packet):
        # Keep the latest packet for control tab sync
        self._latest_pkt = pkt

        # If this is a system-info packet, display a banner in status bar and do a light toast
        try:
            if getattr(pkt, 'is_sysinfo', False):
                side = 'LEFT' if getattr(pkt, 'sys_is_left', 1) else 'RIGHT'
                ver = getattr(pkt, 'sys_ver', None)
                date = getattr(pkt, 'sys_fw_date', '')
                rng = getattr(pkt, 'sys_df_range', None)
                msg = f"System Info: v{ver:.0f} {side} DF{int(rng) if rng else ''} {date}"
                self.set_status(msg)
                # Update System Info grid labels
                try:
                    self.set_label("sys_ver", f"v{int(ver) if ver is not None else 0}")
                except Exception:
                    self.set_label("sys_ver", "--")
                try:
                    self.set_label("sys_side", side)
                except Exception:
                    self.set_label("sys_side", "--")
                try:
                    self.set_label("sys_df", f"DF{int(rng) if rng else ''}")
                except Exception:
                    self.set_label("sys_df", "--")
                try:
                    self.set_label("sys_date", date or "--")
                except Exception:
                    self.set_label("sys_date", "--")
                # Do not feed this packet into plotting/recording
                return
        except Exception:
            pass

        # Accumulate time using pkt.dt in microseconds (e.g., 20000 -> 20ms)
        # Fallback conservatively if dt is missing/invalid.
        try:
            dt_us_val = getattr(pkt, 'dt', None)
            dt_s = float(dt_us_val) / 1_000_000.0 if dt_us_val is not None else None
        except Exception:
            dt_s = None

        # Robust fallback if dt is missing/invalid
        if dt_s is None or not isinstance(dt_s, (int, float)) or dt_s < 0:
            est_hz = self.pps if (self.pps and self.pps > 0) else 50.0
            dt_s = 1.0 / float(est_hz)
        # Clamp extreme spikes
        if dt_s > 0.5:
            dt_s = 0.5

        self._time_accum_s += dt_s
        t_accum = self._time_accum_s
        self.essential_time_buffer.append(t_accum)

        # Gather values (robust to missing attributes like angle_forward)
        def get_val(name: str, default_nan: bool = True):
            if name == 'pps':
                return float(self.pps)
            try:
                return float(getattr(pkt, name))
            except Exception:
                return math.nan if default_nan else 0.0

        values = {k: get_val(k) for k in self.essential_keys}
        for k in self.essential_keys:
            self.essential_buffers[k].append(values[k])

        # Detect state_servo transitions (0->1 blue, 1->0 orange)
        ss = int(getattr(pkt, 'state_servo', 0))
        if self.last_state_servo is None:
            self.last_state_servo = ss
        else:
            if self.last_state_servo == 0 and ss == 1:
                self.transition_events_servo.append((t_accum, 'blue'))
            elif self.last_state_servo == 1 and ss == 0:
                self.transition_events_servo.append((t_accum, 'orange'))
            self.last_state_servo = ss

        # Rolling window
        window_sec = 10.0
        t0 = t_accum - window_sec
        times_all = list(self.essential_time_buffer)
        times_window = [t for t in times_all if t >= t0]
        window_start = times_window[0] if times_window else None

        # Update combined plot (skip when paused)
        if window_start is not None and not getattr(self, 'chart_paused', False):
            t_filtered = [t - window_start for t in times_window]
            total_len = len(times_all)
            idx_start = total_len - len(times_window)

            for key in self.combined_keys:
                vals_full = list(self.essential_buffers[key])
                vals = vals_full[idx_start:]
                # scaling per group
                if key.startswith("gyr_"):
                    scale = self.fixed_scales["gyr_"]
                    offset = self.group_offsets["gyr_"]
                elif key.startswith("acc_"):
                    scale = self.fixed_scales["acc_"]
                    offset = self.group_offsets["acc_"]
                elif key.startswith("angle_"):
                    scale = self.fixed_scales["angle_"]
                    offset = self.group_offsets["angle_"]
                elif key.startswith("state_"):
                    scale = self.fixed_scales["state_"]
                    offset = self.group_offsets["angle_"] - 240

                y = [((v * scale) + offset) if (not math.isnan(v)) else math.nan for v in vals]
                self.combined_curves[key].setData(t_filtered, y)

            # Update noncombined plots
            for key in self.noncombined_keys:
                plot_curve = self.essential_plot_curves.get(key)
                if not plot_curve:
                    continue
                vals_full = list(self.essential_buffers[key])
                vals = vals_full[idx_start:]
                plot_curve.setData(t_filtered, vals)

            # Update vertical markers on combined plot
            # Move existing and drop out-of-window
            alive: List[Tuple[pyqtgraph.InfiniteLine, float, str]] = []
            for line_item, t_event, color in self.marker_lines_combined:
                if t_event >= t0:
                    x = t_event - window_start
                    try:
                        line_item.setValue(x)
                    except Exception:
                        pass
                    alive.append((line_item, t_event, color))
                else:
                    try:
                        self.combined_plot.removeItem(line_item)
                    except Exception:
                        pass
            self.marker_lines_combined = alive

            # Add new markers for events within current window
            existing_times = {t for _, t, _ in alive}

            # Add servo transition markers (dashed)
            for t_event, color in self.transition_events_servo:
                if t_event < t0 or t_event in existing_times:
                    continue
                pen = pyqtgraph.mkPen(color=color, width=2, style=Qt.PenStyle.DashLine)
                line = pyqtgraph.InfiniteLine(pos=t_event - window_start, angle=90, pen=pen, movable=False)
                self.combined_plot.addItem(line)
                self.marker_lines_combined.append((line, t_event, color))

            # Ensure axis ticks remain as defined
            if hasattr(self, 'left_axis') and hasattr(self, '_fixed_ticks'):
                self.left_axis.setTicks(self._fixed_ticks)

        # Received incoming data packet
        self.packet_counter += 1

        # Timing
        self.set_label("count", f"{getattr(pkt, 'count', 0.0):.0f}")
        self.set_label("dt", f"{getattr(pkt, 'dt', 0.0):.0f}")
        if self.last_count is None:
            delta = 0
        else:
            try:
                delta = getattr(pkt, 'count', 0.0) - self.last_count
            except Exception:
                delta = 0
        self.set_label("delta_count", f"{delta:.0f}")
        try:
            self.last_count = getattr(pkt, 'count', None)
        except Exception:
            self.last_count = None

        # Angles
        self.set_label("angle_forward", f"{getattr(pkt, 'angle_forward', math.nan):5.1f}")
        self.set_label("angle_abduction", f"{getattr(pkt, 'angle_abduction', math.nan):5.1f}")

        # Accel
        self.set_label("acc_x", f"{getattr(pkt, 'acc_x', math.nan):6.2f}")
        self.set_label("acc_y", f"{getattr(pkt, 'acc_y', math.nan):6.2f}")
        self.set_label("acc_z", f"{getattr(pkt, 'acc_z', math.nan):6.2f}")

        # Gyro
        self.set_label("gyr_x", f"{getattr(pkt, 'gyr_x', math.nan):6.2f}")
        self.set_label("gyr_y", f"{getattr(pkt, 'gyr_y', math.nan):6.2f}")
        self.set_label("gyr_z", f"{getattr(pkt, 'gyr_z', math.nan):6.2f}")

        # Power
        self.set_label("current", f"{getattr(pkt, 'current', math.nan):4.2f}")
        self.set_label("battery_state", f"{getattr(pkt, 'state_battery', 0)}")
        percent = int(getattr(pkt, 'percentage_battery', 0))
        self.battery_bar.setValue(max(0, min(100, percent)))
        self.set_label("battery_pct", f"{percent} %")
        self.colorize_battery(percent)

        # States
        self.set_label("state_gait", str(getattr(pkt, 'state_gait', 0)))
        self.set_label("state_servo", str(getattr(pkt, 'state_servo', 0)))
        self.set_label("state_battery", str(getattr(pkt, 'state_battery', 0)))
        self.set_label("walk_mode", str(getattr(pkt, 'walk_mode', 0)))
        try:
            self.chk_cal_error.setChecked(bool(getattr(pkt, 'calibrate_error', 0)))
            self.chk_early_swing.setChecked(bool(getattr(pkt, 'flag_early_swing', 0)))
            self.chk_motor.setChecked(bool(getattr(pkt, 'flag_enable_motor', 0)))
            self.chk_buzzer.setChecked(bool(getattr(pkt, 'flag_enable_buzzer', 0)))
            self.chk_cpm_mode.setChecked(bool(getattr(pkt, 'cpm_mode', 0)))
            self.chk_is_left.setChecked(bool(getattr(pkt, 'is_left', 0)))
        except Exception:
            pass

        # Calibrate error highlighting
        if bool(getattr(pkt, 'calibrate_error', 0)):
            self.chk_cal_error.setStyleSheet("background: #ff5555;")
        else:
            self.chk_cal_error.setStyleSheet("")

        # Servo / position setting and status
        self.set_label("position", f"{getattr(pkt, 'position', math.nan):6.2f}")
        self.set_label("servo_lo", f"{getattr(pkt, 'servo_lo', 0)}")
        self.set_label("servo_hi", f"{getattr(pkt, 'servo_hi', 0)}")
        self.set_label("cpm_range_df", f"{getattr(pkt, 'cpm_range_df', 0)}")
        self.set_label("cpm_range_pf", f"{getattr(pkt, 'cpm_range_pf', 0)}")

        # CPM settings
        self.set_label("cpm_count", f"{getattr(pkt, 'cpm_count', math.nan):.0f}")
        self.set_label("cpm_duration", f"{getattr(pkt, 'cpm_duration', math.nan):5.2f}")
        self.set_label("cpm_df_dt", f"{getattr(pkt, 'cpm_df_dt', 0)}")
        self.set_label("cpm_df_wait", f"{getattr(pkt, 'cpm_df_wait', 0)}")
        self.set_label("cpm_pf_dt", f"{getattr(pkt, 'cpm_pf_dt', 0)}")
        self.set_label("cpm_pf_wait", f"{getattr(pkt, 'cpm_pf_wait', 0)}")

        # Custom Y-axis ticks that reflect original units at each group baseline
        # Angle: show 0, ±50°, ±100° around angle baseline (×1 ⇒ ±100)
        # Accel: show 0g, ±0.5g, ±1g around acc baseline (×100 ⇒ ±100)
        # Gyro: show 0, ±50, ±100 deg/s around gyro baseline (×1 ⇒ ±100)
        def make_ticks():
            ticks: List[Tuple[float, str]] = []
            # angle baseline with ±50, ±100 degrees
            a0 = self.group_offsets['angle_']
            ticks.extend([
                (a0 - 100, '-100°'),
                (a0 - 50, '-50°'),
                (a0, '0°'),
                (a0 + 50, '+50°'),
                (a0 + 100, '+100°'),
            ])
            # accel baseline ±1g -> ±100 after scale
            acc0 = self.group_offsets['acc_']
            ticks.extend([
                (acc0 - 100, '-1g'),
                (acc0 - 50, '-0.5g'),
                (acc0, '0g'),
                (acc0 + 50, '+0.5g'),
                (acc0 + 100, '+1g'),
            ])
            # gyro baseline ±100 deg/s -> ±100 after scale
            g0 = self.group_offsets['gyr_']
            ticks.extend([
                (g0 - 100, '-100'),
                (g0 - 50, '-50'),
                (g0, '0'),
                (g0 + 50, '+50'),
                (g0 + 100, '+100'),
            ])
            # AxisItem.setTicks expects a list of levels; we provide one level with our ticks
            return [ticks]

        self._fixed_ticks = make_ticks()
        self.left_axis.setTicks(self._fixed_ticks)

        # ---- CSV recording (non-blocking, buffered) ----
        if getattr(self, 'recording', False) and self._record_writer is not None:
            try:
                # Build row of requested fields
                # Required: pps, position, current, state_gait, state_servo,
                #           angle_*, acc_*, gyr_*
                row = {
                    # Keep real epoch time for logs; t_rec uses accumulated packet time
                    't_rec': (self._time_accum_s - self._record_start_t_accum) if self._record_start_t_accum is not None else 0.0,
                    'ts_epoch': time.time(),
                    'count': getattr(pkt, 'count', 0),
                    'dt': getattr(pkt, 'dt', 0.0),
                    'pps': float(self.pps),
                    'position': getattr(pkt, 'position', math.nan),
                    'current': getattr(pkt, 'current', math.nan),
                    'is_left': pkt.is_left,
                    'angle_forward': getattr(pkt, 'angle_forward', math.nan),
                    'angle_abduction': getattr(pkt, 'angle_abduction', math.nan),
                    'acc_x': getattr(pkt, 'acc_x', math.nan),
                    'acc_y': getattr(pkt, 'acc_y', math.nan),
                    'acc_z': getattr(pkt, 'acc_z', math.nan),
                    'gyr_x': getattr(pkt, 'gyr_x', math.nan),
                    'gyr_y': getattr(pkt, 'gyr_y', math.nan),
                    'gyr_z': getattr(pkt, 'gyr_z', math.nan),
                    'state_servo': getattr(pkt, 'state_servo', 0),
                    'state_gait': getattr(pkt, 'state_gait', 0),
                }
                self._record_writer.writerow(row)
                # Flush lightly to keep data safe without too much overhead
                if self._record_fp:
                    self._record_fp.flush()
            except Exception as e:
                # Do not break UI on logging errors
                print(e)

    def update_pps(self):
        self.pps = self.packet_counter
        self.set_label("pps", f"{self.pps:4.1f}")
        self.packet_counter = 0

    # ------------- Helpers -------------
    def set_label(self, key: str, text: str):
        lab = self.labels.get(key)
        if lab:
            lab.setText(text)

    def colorize_battery(self, pct: int):
        if pct >= 70:
            color = "#4CAF50"
        elif pct >= 40:
            color = "#FFC107"
        else:
            color = "#F44336"
        self.battery_bar.setStyleSheet(
            f"QProgressBar::chunk {{background-color: {color};}}"
        )

    def on_connection_lost(self, msg: str):
        QMessageBox.warning(self, "Connection", msg)
        self.stop_connection()

    def set_status(self, txt: str):
        self.status_bar.showMessage(txt)

    # ------------- Recording controls -------------
    def toggle_recording(self):
        if not self.recording:
            self.start_recording()
        else:
            self.stop_recording()

    def toggle_chart_pause(self):
        """Pause/resume chart updates while keeping data capture & recording running."""
        self.chart_paused = not self.chart_paused
        if self.chart_paused:
            try:
                self.btn_pause_chart.setText('Resume Chart')
                self.btn_pause_chart.setStyleSheet('background:#1976d2; color:white;')
            except Exception:
                pass
            self.set_status('Chart paused (data still streaming)')
        else:
            try:
                self.btn_pause_chart.setText('Pause Chart')
                self.btn_pause_chart.setStyleSheet('')
            except Exception:
                pass
            self.set_status('Chart resumed')

    def start_recording(self):
        try:
            logs_dir = os.path.join(os.path.dirname(__file__), 'logs')
            os.makedirs(logs_dir, exist_ok=True)
            ts_name = time.strftime('%Y%m%d_%H%M%S')
            fname = os.path.join(logs_dir, f'data_{ts_name}.csv')
            fp = open(fname, 'a', newline='')
            fieldnames = [
                't_rec', 'ts_epoch', 'count', 'dt',
                'pps', 'position', 'current', 'is_left',
                'angle_forward', 'angle_abduction',
                'acc_x', 'acc_y', 'acc_z',
                'gyr_x', 'gyr_y', 'gyr_z',
                'state_servo',
                'state_gait',
            ]
            writer = csv.DictWriter(fp, fieldnames=fieldnames)
            # Write header if file just created (size 0)
            if fp.tell() == 0:
                writer.writeheader()
            self._record_fp = fp
            self._record_writer = writer
            self._record_start_time = time.time()
            # Anchor accumulated time so CSV t_rec starts at ~0 when recording begins
            self._record_start_t_accum = self._time_accum_s
            self.recording = True
            # Update UI
            self.btn_record.setChecked(True)
            self.btn_record.setText('Stop Recording')
            self.btn_record.setStyleSheet('background:#c62828; color:white;')
            self.set_status(f"Recording to {os.path.basename(fname)}")
        except Exception as e:
            self.set_status(f"Record start failed: {e}")
            self.recording = False
            self._record_fp = None
            self._record_writer = None
            self._record_start_time = None
            try:
                if 'fp' in locals():
                    fp.close()
            except Exception:
                pass

    def stop_recording(self):
        try:
            if self._record_fp:
                try:
                    self._record_fp.flush()
                except Exception:
                    pass
                self._record_fp.close()
        except Exception:
            pass
        finally:
            self._record_fp = None
            self._record_writer = None
            self._record_start_time = None
            self._record_start_t_accum = None
            self.recording = False
            # Update UI
            try:
                self.btn_record.setChecked(False)
                self.btn_record.setText('Start Recording')
                self.btn_record.setStyleSheet('')
            except Exception:
                pass
            self.set_status('Recording stopped')


# ------------- Entry point -------------
def main():
    app = QApplication(sys.argv)
    win = MonitorWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

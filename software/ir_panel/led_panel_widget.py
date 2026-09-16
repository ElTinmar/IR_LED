"""
qtpy widget for the Arduino LED Panel Controller.

All serial I/O happens on a background QThread (PanelWorker); discovery
also runs on its own QThread (PortScanner) since probing several ports
with a boot-wait can take a couple of seconds.
"""

from __future__ import annotations

import queue
import sys
import time
from typing import List, Optional

from qtpy.QtCore import QThread, Signal, Qt
from qtpy.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from arduino_led_panel import (
    LEDPanelError,
    LEDPanelInfo,
    PIDState,
    SerialLEDPanel,
)


# ============================================================== #
# Background worker: scans ports using the ID handshake
# ============================================================== #
class PortScanner(QThread):
    finished_scan = Signal(list)  # List[LEDPanelInfo]

    def run(self):
        try:
            panels = SerialLEDPanel.list_available_panels()
        except Exception:
            panels = []
        self.finished_scan.emit(panels)


# ============================================================== #
# Background worker: owns the serial connection to one board
# ============================================================== #
class PanelWorker(QThread):
    connection_changed = Signal(bool)
    error_occurred = Signal(str)

    dac_value_changed = Signal(int, int)      # channel, value
    dac_values_loaded = Signal(list)          # [v0, v1, v2]
    fan_changed = Signal(int)
    temperature_changed = Signal(float)
    telemetry_received = Signal(dict)

    pid_mode_changed = Signal(bool)
    pid_setpoint_changed = Signal(float)
    pid_gains_changed = Signal(float, float, float)
    pid_state_loaded = Signal(object)         # PIDState
    pid_telemetry_received = Signal(dict)

    log_message = Signal(str)

    def __init__(self, panel: SerialLEDPanel, poll_interval: float = 2.0):
        super().__init__()
        self.panel = panel
        self.poll_interval = poll_interval
        self._jobs: "queue.Queue" = queue.Queue()
        self._running = False

    # -- lifecycle ------------------------------------------------- #
    def run(self):
        self._running = True
        self.panel.set_telemetry_callback(lambda d: self.telemetry_received.emit(d))
        self.panel.set_pid_telemetry_callback(lambda d: self.pid_telemetry_received.emit(d))
        self.panel.set_log_callback(lambda s: self.log_message.emit(s))

        try:
            self.panel.connect()
        except LEDPanelError as e:
            self.error_occurred.emit(str(e))
            self.connection_changed.emit(False)
            return

        self.connection_changed.emit(True)
        last_poll = 0.0

        while self._running:
            try:
                func = self._jobs.get(timeout=0.1)
                try:
                    func()
                except LEDPanelError as e:
                    self.error_occurred.emit(str(e))
            except queue.Empty:
                pass

            now = time.monotonic()
            if now - last_poll >= self.poll_interval:
                last_poll = now
                try:
                    temp = self.panel.get_temperature_degC()
                    self.temperature_changed.emit(temp)
                except LEDPanelError as e:
                    self.error_occurred.emit(str(e))

        self.panel.disconnect()
        self.connection_changed.emit(False)

    def stop(self):
        self._running = False
        self.wait(3000)

    # -- job submission API (call from the GUI thread) ------------- #
    def _submit(self, func):
        self._jobs.put(func)

    def request_set_dac(self, channel: int, value: int):
        def _job():
            val = self.panel.set_dac_raw(channel, value)
            self.dac_value_changed.emit(channel, val)
        self._submit(_job)

    def request_set_fan(self, percent: int):
        def _job():
            val = self.panel.set_fan_percent(percent)
            self.fan_changed.emit(val)
        self._submit(_job)

    def request_get_dac_values(self):
        def _job():
            vals = self.panel.get_dac_values()
            self.dac_values_loaded.emit(vals)
        self._submit(_job)

    # -- PID job submission ----------------------------------------- #
    def request_set_pid_mode(self, enabled: bool):
        def _job():
            val = self.panel.set_pid_mode(enabled)
            self.pid_mode_changed.emit(val)
        self._submit(_job)

    def request_set_pid_setpoint(self, setpoint_degC: float):
        def _job():
            val = self.panel.set_pid_setpoint(setpoint_degC)
            self.pid_setpoint_changed.emit(val)
        self._submit(_job)

    def request_set_pid_gains(self, kp: float, ki: float, kd: float):
        def _job():
            vals = self.panel.set_pid_gains(kp, ki, kd)
            self.pid_gains_changed.emit(*vals)
        self._submit(_job)

    def request_get_pid_state(self):
        def _job():
            state = self.panel.get_pid_state()
            self.pid_state_loaded.emit(state)
        self._submit(_job)


# ============================================================== #
# One DAC channel's controls
# ============================================================== #
class DacChannelWidget(QGroupBox):
    value_changed = Signal(int, int)  # channel, value

    def __init__(self, channel: int, title: str, parent=None):
        super().__init__(title, parent)
        self.channel = channel

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 4095)

        self.spinbox = QSpinBox()
        self.spinbox.setRange(0, 4095)
        self.spinbox.setKeyboardTracking(False)  # commit on Enter / focus-out / arrows only

        self.sat_label = QLabel("")
        self.sat_label.setStyleSheet("color: red; font-weight: bold;")

        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spinbox.valueChanged.connect(self._on_spinbox_changed)

        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        row.addWidget(self.slider)
        row.addWidget(self.spinbox)
        layout.addLayout(row)
        layout.addWidget(self.sat_label)

        self._updating = False

    def _on_slider_changed(self, value: int):
        if self._updating:
            return
        self._updating = True
        self.spinbox.setValue(value)
        self._updating = False
        self.value_changed.emit(self.channel, value)

    def _on_spinbox_changed(self, value: int):
        if self._updating:
            return
        self._updating = True
        self.slider.setValue(value)
        self._updating = False
        self.value_changed.emit(self.channel, value)

    def set_value_silent(self, value: int):
        self._updating = True
        self.slider.setValue(value)
        self.spinbox.setValue(value)
        self._updating = False
        self.update_saturation(value)

    def update_saturation(self, value: int):
        self.sat_label.setText("SATURATED" if value in (0, 4095) else "")


# ============================================================== #
# Thermal control: manual fan + PID controller
# ============================================================== #
class ThermalControlWidget(QGroupBox):
    fan_manual_changed = Signal(int)
    pid_enable_toggled = Signal(bool)
    pid_setpoint_changed = Signal(float)
    pid_gains_applied = Signal(float, float, float)

    def __init__(self, parent=None):
        super().__init__("Thermal Control", parent)

        # -- manual fan ---------------------------------------------- #
        self.fan_group = QGroupBox("Manual Fan")
        self.fan_slider = QSlider(Qt.Horizontal)
        self.fan_slider.setRange(0, 100)
        self.fan_spin = QSpinBox()
        self.fan_spin.setRange(0, 100)
        self.fan_spin.setSuffix(" %")
        self.fan_spin.setKeyboardTracking(False)
        self.fan_slider.valueChanged.connect(self.fan_spin.setValue)
        self.fan_spin.valueChanged.connect(self.fan_slider.setValue)
        self.fan_spin.valueChanged.connect(self.fan_manual_changed.emit)
        fan_layout = QHBoxLayout(self.fan_group)
        fan_layout.addWidget(self.fan_slider)
        fan_layout.addWidget(self.fan_spin)

        # -- PID controller -------------------------------------------#
        self.pid_group = QGroupBox("PID Controller")
        self.pid_enable_checkbox = QCheckBox("Enable")
        self.pid_enable_checkbox.toggled.connect(self._on_enable_toggled)

        self.setpoint_spin = QDoubleSpinBox()
        self.setpoint_spin.setRange(-40.0, 150.0)
        self.setpoint_spin.setDecimals(1)
        self.setpoint_spin.setSingleStep(0.5)
        self.setpoint_spin.setSuffix(" \u00b0C")
        self.setpoint_spin.setKeyboardTracking(False)
        self.setpoint_spin.valueChanged.connect(self.pid_setpoint_changed.emit)

        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0.0, 1000.0)
        self.kp_spin.setDecimals(3)
        self.kp_spin.setSingleStep(0.1)
        self.kp_spin.setKeyboardTracking(False)

        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0.0, 1000.0)
        self.ki_spin.setDecimals(3)
        self.ki_spin.setSingleStep(0.05)
        self.ki_spin.setKeyboardTracking(False)

        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0.0, 1000.0)
        self.kd_spin.setDecimals(3)
        self.kd_spin.setSingleStep(0.1)
        self.kd_spin.setKeyboardTracking(False)

        self.apply_gains_btn = QPushButton("Apply Gains")
        self.apply_gains_btn.clicked.connect(self._on_apply_gains)

        form = QFormLayout()
        form.addRow(self.pid_enable_checkbox)
        form.addRow("Setpoint:", self.setpoint_spin)
        form.addRow("Kp:", self.kp_spin)
        form.addRow("Ki:", self.ki_spin)
        form.addRow("Kd:", self.kd_spin)
        form.addRow(self.apply_gains_btn)
        self.pid_group.setLayout(form)

        # -- live readout ---------------------------------------------#
        self.temp_readout = QLabel("-- \u00b0C")
        self.fan_readout = QLabel("-- %")
        readout_form = QFormLayout()
        readout_form.addRow("Temperature:", self.temp_readout)
        readout_form.addRow("Fan duty:", self.fan_readout)

        layout = QVBoxLayout(self)
        layout.addWidget(self.fan_group)
        layout.addWidget(self.pid_group)
        layout.addLayout(readout_form)

        self.setEnabled(False)

    # ------------------------------------------------------------ #
    def _on_enable_toggled(self, checked: bool):
        self.fan_group.setEnabled(not checked)
        self.pid_enable_toggled.emit(checked)

    def _on_apply_gains(self):
        self.pid_gains_applied.emit(
            self.kp_spin.value(), self.ki_spin.value(), self.kd_spin.value()
        )

    # -- external state updates (must not re-trigger signals) ----- #
    def set_manual_fan_silent(self, value: int):
        self.fan_slider.blockSignals(True)
        self.fan_spin.blockSignals(True)
        self.fan_slider.setValue(value)
        self.fan_spin.setValue(value)
        self.fan_slider.blockSignals(False)
        self.fan_spin.blockSignals(False)

    def set_pid_enabled_silent(self, enabled: bool):
        self.pid_enable_checkbox.blockSignals(True)
        self.pid_enable_checkbox.setChecked(enabled)
        self.pid_enable_checkbox.blockSignals(False)
        self.fan_group.setEnabled(not enabled)

    def set_pid_setpoint_silent(self, value: float):
        self.setpoint_spin.blockSignals(True)
        self.setpoint_spin.setValue(value)
        self.setpoint_spin.blockSignals(False)

    def set_pid_gains_silent(self, kp: float, ki: float, kd: float):
        for spin, value in ((self.kp_spin, kp), (self.ki_spin, ki), (self.kd_spin, kd)):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)

    def apply_pid_state(self, state: PIDState):
        self.set_pid_enabled_silent(state.enabled)
        self.set_pid_setpoint_silent(state.setpoint)
        self.set_pid_gains_silent(state.kp, state.ki, state.kd)
        self.update_readout(state.temperature, state.fan_duty)
        if not state.enabled:
            self.set_manual_fan_silent(state.fan_duty)

    def update_readout(self, temperature: float, fan_duty: int):
        self.temp_readout.setText(f"{temperature:.2f} \u00b0C")
        self.fan_readout.setText(f"{fan_duty} %")


# ============================================================== #
# Main widget
# ============================================================== #
class LEDPanelWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker: Optional[PanelWorker] = None
        self.scanner: Optional[PortScanner] = None

        # -- board selection ----------------------------------------- #
        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh")
        self.connect_btn = QPushButton("Connect")
        self.status_label = QLabel("Disconnected")

        top = QHBoxLayout()
        top.addWidget(QLabel("Board:"))
        top.addWidget(self.port_combo, 1)
        top.addWidget(self.refresh_btn)
        top.addWidget(self.connect_btn)

        # -- DAC channels --------------------------------------------#
        self.channels = [
            DacChannelWidget(0, "Channel 1"),
            DacChannelWidget(1, "Channel 2"),
            DacChannelWidget(2, "Channel 3"),
        ]
        for ch in self.channels:
            ch.setEnabled(False)
            ch.value_changed.connect(self._on_channel_changed)

        # -- Thermal control (manual fan + PID) -----------------------#
        self.thermal = ThermalControlWidget()
        self.thermal.fan_manual_changed.connect(self._on_fan_changed)
        self.thermal.pid_enable_toggled.connect(self._on_pid_enable_toggled)
        self.thermal.pid_setpoint_changed.connect(self._on_pid_setpoint_changed)
        self.thermal.pid_gains_applied.connect(self._on_pid_gains_applied)

        # -- log / status --------------------------------------------- #
        self.log_label = QLabel("")
        self.log_label.setWordWrap(True)

        # -- assemble --------------------------------------------------#
        layout = QVBoxLayout(self)
        layout.addLayout(top)
        for ch in self.channels:
            layout.addWidget(ch)
        layout.addWidget(self.thermal)
        layout.addWidget(self.status_label)
        layout.addWidget(self.log_label)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)

        self.refresh_ports()

    # ------------------------------------------------------------ #
    # Discovery
    # ------------------------------------------------------------ #
    def refresh_ports(self):
        self.refresh_btn.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.port_combo.clear()
        self.port_combo.addItem("Scanning\u2026", None)
        self.status_label.setText("Scanning for LED panel boards\u2026")

        self.scanner = PortScanner()
        self.scanner.finished_scan.connect(self._on_scan_finished)
        self.scanner.start()

    def _on_scan_finished(self, panels: List[LEDPanelInfo]):
        self.port_combo.clear()
        if not panels:
            self.port_combo.addItem("No boards found", None)
            self.status_label.setText("No LED panel boards found")
        else:
            for info in panels:
                self.port_combo.addItem(info.name, info)
            self.status_label.setText(f"Found {len(panels)} board(s)")
        self.refresh_btn.setEnabled(True)
        self.connect_btn.setEnabled(True)

    # ------------------------------------------------------------ #
    # Connection lifecycle
    # ------------------------------------------------------------ #
    def toggle_connection(self):
        if self.worker is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        info: Optional[LEDPanelInfo] = self.port_combo.currentData()
        if info is None:
            QMessageBox.warning(
                self, "No board", "Click Refresh and select a board first."
            )
            return

        panel = info.instantiate()
        self.worker = PanelWorker(panel)
        self.worker.connection_changed.connect(self._on_connection_changed)
        self.worker.error_occurred.connect(self._on_error)
        self.worker.dac_value_changed.connect(self._on_dac_value_from_worker)
        self.worker.dac_values_loaded.connect(self._on_dac_values_loaded)
        self.worker.fan_changed.connect(self._on_fan_from_worker)
        self.worker.temperature_changed.connect(self._on_temperature)
        self.worker.telemetry_received.connect(self._on_telemetry)

        self.worker.pid_mode_changed.connect(self._on_pid_mode_from_worker)
        self.worker.pid_setpoint_changed.connect(self._on_pid_setpoint_from_worker)
        self.worker.pid_gains_changed.connect(self._on_pid_gains_from_worker)
        self.worker.pid_state_loaded.connect(self._on_pid_state_loaded)
        self.worker.pid_telemetry_received.connect(self._on_pid_telemetry)

        self.worker.log_message.connect(self.log_label.setText)

        self.connect_btn.setEnabled(False)
        self.status_label.setText("Connecting\u2026")
        self.worker.start()

    def _disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.connect_btn.setText("Connect")
        self.connect_btn.setEnabled(True)
        self.status_label.setText("Disconnected")
        for ch in self.channels:
            ch.setEnabled(False)
        self.thermal.setEnabled(False)

    # ------------------------------------------------------------ #
    # Slots reacting to worker signals
    # ------------------------------------------------------------ #
    def _on_connection_changed(self, connected: bool):
        self.connect_btn.setEnabled(True)
        if connected:
            self.connect_btn.setText("Disconnect")
            self.status_label.setText("Connected")
            for ch in self.channels:
                ch.setEnabled(True)
            self.thermal.setEnabled(True)
            self.worker.request_get_dac_values()
            self.worker.request_get_pid_state()
        else:
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            self.worker = None

    def _on_error(self, message: str):
        self.status_label.setText(f"Error: {message}")

    def _on_dac_value_from_worker(self, channel: int, value: int):
        self.channels[channel].set_value_silent(value)

    def _on_dac_values_loaded(self, values):
        for ch, val in zip(self.channels, values):
            ch.set_value_silent(val)

    def _on_fan_from_worker(self, value: int):
        self.thermal.set_manual_fan_silent(value)

    def _on_temperature(self, temp: float):
        self.thermal.temp_readout.setText(f"{temp:.2f} \u00b0C")

    def _on_telemetry(self, data: dict):
        # Encoder-driven change on the board itself: reflect it in the UI.
        ch = data["channel"] - 1
        if 0 <= ch < 3:
            self.channels[ch].set_value_silent(data["value"])

    def _on_pid_mode_from_worker(self, enabled: bool):
        self.thermal.set_pid_enabled_silent(enabled)

    def _on_pid_setpoint_from_worker(self, value: float):
        self.thermal.set_pid_setpoint_silent(value)

    def _on_pid_gains_from_worker(self, kp: float, ki: float, kd: float):
        self.thermal.set_pid_gains_silent(kp, ki, kd)

    def _on_pid_state_loaded(self, state: PIDState):
        self.thermal.apply_pid_state(state)

    def _on_pid_telemetry(self, data: dict):
        self.thermal.update_readout(data["temperature"], data["fan_duty"])
        # Keep the enable checkbox in sync in case the board's PID mode
        # was toggled by something other than this UI (e.g. a reboot).
        self.thermal.set_pid_enabled_silent(data["enabled"])

    # ------------------------------------------------------------ #
    # Slots reacting to user interaction
    # ------------------------------------------------------------ #
    def _on_channel_changed(self, channel: int, value: int):
        if self.worker:
            self.worker.request_set_dac(channel, value)

    def _on_fan_changed(self, value: int):
        if self.worker:
            self.worker.request_set_fan(value)

    def _on_pid_enable_toggled(self, checked: bool):
        if self.worker:
            self.worker.request_set_pid_mode(checked)

    def _on_pid_setpoint_changed(self, value: float):
        if self.worker:
            self.worker.request_set_pid_setpoint(value)

    def _on_pid_gains_applied(self, kp: float, ki: float, kd: float):
        if self.worker:
            self.worker.request_set_pid_gains(kp, ki, kd)

    def closeEvent(self, event):
        self._disconnect()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = LEDPanelWidget()
    w.setWindowTitle("Arduino LED Panel Controller")
    w.resize(440, 700)
    w.show()
    sys.exit(app.exec_())
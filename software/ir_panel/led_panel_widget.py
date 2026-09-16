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
    QComboBox,
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

from led_panel import (
    LEDPanelError,
    LEDPanelInfo,
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


# ============================================================== #
# One channel's controls
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

        # -- Fan -------------------------------------------------------#
        self.fan_group = QGroupBox("Fan")
        self.fan_slider = QSlider(Qt.Horizontal)
        self.fan_slider.setRange(0, 100)
        self.fan_spin = QSpinBox()
        self.fan_spin.setRange(0, 100)
        self.fan_slider.valueChanged.connect(self.fan_spin.setValue)
        self.fan_spin.valueChanged.connect(self.fan_slider.setValue)
        self.fan_spin.valueChanged.connect(self._on_fan_changed)
        fan_layout = QHBoxLayout(self.fan_group)
        fan_layout.addWidget(self.fan_slider)
        fan_layout.addWidget(self.fan_spin)
        self.fan_group.setEnabled(False)

        # -- Temperature ------------------------------------------------#
        self.temp_label = QLabel("-- degC")
        temp_group = QGroupBox("TMP126")
        temp_layout = QHBoxLayout(temp_group)
        temp_layout.addWidget(self.temp_label)

        # -- log / status --------------------------------------------- #
        self.log_label = QLabel("")
        self.log_label.setWordWrap(True)

        # -- assemble --------------------------------------------------#
        layout = QVBoxLayout(self)
        layout.addLayout(top)
        for ch in self.channels:
            layout.addWidget(ch)
        layout.addWidget(self.fan_group)
        layout.addWidget(temp_group)
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
        self.fan_group.setEnabled(False)

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
            self.fan_group.setEnabled(True)
            self.worker.request_get_dac_values()
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
        self.fan_spin.blockSignals(True)
        self.fan_spin.setValue(value)
        self.fan_spin.blockSignals(False)

    def _on_temperature(self, temp: float):
        self.temp_label.setText(f"{temp:.2f} degC")

    def _on_telemetry(self, data: dict):
        # Encoder-driven change on the board itself: reflect it in the UI.
        ch = data["channel"] - 1
        if 0 <= ch < 3:
            self.channels[ch].set_value_silent(data["value"])

    # ------------------------------------------------------------ #
    # Slots reacting to user interaction
    # ------------------------------------------------------------ #
    def _on_channel_changed(self, channel: int, value: int):
        if self.worker:
            self.worker.request_set_dac(channel, value)

    def _on_fan_changed(self, value: int):
        if self.worker:
            self.worker.request_set_fan(value)

    def closeEvent(self, event):
        self._disconnect()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = LEDPanelWidget()
    w.setWindowTitle("Arduino LED Panel Controller")
    w.resize(420, 520)
    w.show()
    sys.exit(app.exec_())
"""
Python driver for the Arduino LED Panel Controller.

Protocol (newline-terminated ASCII lines over serial, 115200 baud 8N1):

    Host -> Board                Board -> Host
    -------------                -------------
    ID                       ->  RSP:ID,<device_type>,<fw_version>,<unique_id>
    SET_FAN:<0-100>          ->  RSP:FAN,<value>
    SET_DAC:<1-3>,<0-4095>   ->  RSP:DAC_SET,<channel>,<value>
    GET_DAC                 ->  RSP:DAC_VALS,<v1>,<v2>,<v3>
    GET_TEMP                ->  RSP:TMP,<float>
    (unsolicited, encoder)  ->  CH:<n>,VAL:<v>,SAT:<0|1>,MOCK:<0|1>
    (boot)                  ->  SYS:BOOTING... / SYS:READY / WARN:...
    (any error)             ->  ERR:<reason>
"""

from __future__ import annotations

import queue
import re
import threading
import time
from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple, Type, Union

import serial
import serial.tools.list_ports


# ============================================================== #
# Exceptions
# ============================================================== #
class LEDPanelError(Exception):
    """Base class for all errors raised by this module."""


class LEDConnectionFailed(LEDPanelError):
    """Could not open / identify / initialize the serial connection."""


class LEDCommandError(LEDPanelError):
    """The board responded with ERR:... or an unparsable response."""


class LEDTimeoutError(LEDPanelError):
    """No response was received within the configured timeout."""


# ============================================================== #
# Discovery data structures
# ============================================================== #
@dataclass
class LEDPanelIdentity:
    """Result of a successful `ID` handshake with a board."""
    device_type: str
    firmware_version: str
    unique_id: str
    port: str


@dataclass
class LEDPanelInfo:
    """Describes an instantiable panel, as returned by discovery."""
    name: str
    ledpanel_cls: Type["LEDPanel"]
    args: Tuple[Any, ...] = field(default_factory=tuple)
    kwargs: Dict[str, Any] = field(default_factory=dict)

    def instantiate(self) -> "LEDPanel":
        return self.ledpanel_cls(*self.args, **self.kwargs)


# ============================================================== #
# Abstract interface
# ============================================================== #
class LEDPanel(ABC):

    @classmethod
    @abstractmethod
    def list_available_panels(cls, *args, **kwargs) -> List[LEDPanelInfo]:
        ...

    @abstractmethod
    def connect(self):
        ...

    @abstractmethod
    def disconnect(self):
        ...

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    @abstractmethod
    def set_brightness(self, brightness: Union[float, Iterable[float]]):
        ...

    @abstractmethod
    def get_temperature_degC(self) -> float:
        ...


# ============================================================== #
# Concrete serial implementation
# ============================================================== #
class SerialLEDPanel(LEDPanel):

    VALID_BAUD_RATES = [2400, 4800, 9600, 14400, 19200, 38400, 115200]

    #: Value the firmware reports for DEVICE_TYPE. Used to reject
    #: unrelated Arduino sketches during discovery.
    DEVICE_TYPE = "LED_PANEL_CONTROLLER"

    _RSP_ID = re.compile(r"RSP:ID,([^,]+),([^,]+),([^,\r\n]*)")
    _RSP_DAC_SET = re.compile(r"RSP:DAC_SET,(\d+),(\d+)")
    _RSP_DAC_VALS = re.compile(r"RSP:DAC_VALS,(\d+),(\d+),(\d+)")
    _RSP_FAN = re.compile(r"RSP:FAN,(\d+)")
    _RSP_TMP = re.compile(r"RSP:TMP,([\-0-9.]+)")
    _TELEMETRY = re.compile(r"CH:(\d+),VAL:(\d+),SAT:([01]),MOCK:([01])")

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        data_byte_length=serial.EIGHTBITS,
        parity_check=serial.PARITY_NONE,
        num_stop_bit: int = serial.STOPBITS_ONE,
        timeout: float = 0.2,           # readline() timeout inside reader thread
        write_timeout: Optional[float] = 1.0,
        command_timeout: float = 2.0,   # max wait for RSP:/ERR: to a command
        boot_timeout: float = 5.0,      # max wait for SYS:READY on connect
        xonxoff: bool = False,
        rtscts: bool = False,
        dsrdtr: bool = False,
        verbose: bool = False,
        verify_identity: bool = True,
        expected_unique_id: Optional[str] = None,
    ):
        if baudrate not in self.VALID_BAUD_RATES:
            raise ValueError(f"Supported baud rates are: {self.VALID_BAUD_RATES}")

        self.port = port
        self.baudrate = baudrate
        self.data_byte_length = data_byte_length
        self.parity_check = parity_check
        self.num_stop_bit = num_stop_bit
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.command_timeout = command_timeout
        self.boot_timeout = boot_timeout
        self.xonxoff = xonxoff
        self.rtscts = rtscts
        self.dsrdtr = dsrdtr
        self.verbose = verbose
        self.verify_identity = verify_identity
        self.expected_unique_id = expected_unique_id

        self.connection: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._response_queue: "queue.Queue[str]" = queue.Queue()
        self._send_lock = threading.Lock()

        self._telemetry_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        self._log_callback: Optional[Callable[[str], None]] = None

        # cache, updated whenever we successfully talk to the board
        self.dac_values: List[int] = [0, 0, 0]
        self.mock_active: bool = False
        self.unique_id: Optional[str] = None
        self.firmware_version: Optional[str] = None

    # ---------------------------------------------------------- #
    # Callbacks (used by the Qt layer, but usable standalone too)
    # ---------------------------------------------------------- #
    def set_telemetry_callback(self, cb: Optional[Callable[[Dict[str, Any]], None]]):
        self._telemetry_callback = cb

    def set_log_callback(self, cb: Optional[Callable[[str], None]]):
        self._log_callback = cb

    def _log(self, msg: str):
        if self.verbose:
            print(msg)
        if self._log_callback:
            self._log_callback(msg)

    # ---------------------------------------------------------- #
    # Connection management
    # ---------------------------------------------------------- #
    def connect(self):
        if self.connection and self.connection.is_open:
            return

        try:
            self.connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.data_byte_length,
                parity=self.parity_check,
                stopbits=self.num_stop_bit,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
                xonxoff=self.xonxoff,
                rtscts=self.rtscts,
                dsrdtr=self.dsrdtr,
            )
        except serial.SerialException as e:
            raise LEDConnectionFailed(f"Failed to connect to {self.port}") from e

        self._stop_event.clear()
        self._ready_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self._ready_event.wait(self.boot_timeout)
        if not self._ready_event.is_set():
            self._log("WARN: did not see SYS:READY, continuing anyway")

        if self.verify_identity:
            try:
                identity = self.identify()
            except LEDPanelError as e:
                self.disconnect()
                raise LEDConnectionFailed(
                    f"No valid ID handshake from {self.port}"
                ) from e

            if identity.device_type != self.DEVICE_TYPE:
                self.disconnect()
                raise LEDConnectionFailed(
                    f"{self.port} is not a {self.DEVICE_TYPE} "
                    f"(got device_type={identity.device_type!r})"
                )

            if self.expected_unique_id and identity.unique_id != self.expected_unique_id:
                self.disconnect()
                raise LEDConnectionFailed(
                    f"{self.port} has unique_id={identity.unique_id!r}, "
                    f"expected {self.expected_unique_id!r}"
                )

            self.unique_id = identity.unique_id
            self.firmware_version = identity.firmware_version

    def disconnect(self):
        self._stop_event.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=2.0)
        self._reader_thread = None
        if self.connection and self.connection.is_open:
            self.connection.close()
        self.connection = None

    @property
    def is_connected(self) -> bool:
        return bool(self.connection and self.connection.is_open)

    # ---------------------------------------------------------- #
    # Reader thread
    # ---------------------------------------------------------- #
    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                raw = self.connection.readline()
            except (serial.SerialException, OSError):
                break
            if not raw:
                continue  # readline() timeout, nothing received

            line = raw.decode(errors="replace").strip()
            if not line:
                continue

            if line.startswith("RSP:") or line.startswith("ERR:"):
                self._response_queue.put(line)
            elif line.startswith("CH:"):
                self._handle_telemetry(line)
            elif line == "SYS:READY":
                self._ready_event.set()
                self._log(line)
            else:
                self._log(line)

    def _handle_telemetry(self, line: str):
        m = self._TELEMETRY.match(line)
        if not m:
            self._log(f"Unparsed telemetry: {line}")
            return
        data = {
            "channel": int(m.group(1)),
            "value": int(m.group(2)),
            "saturated": bool(int(m.group(3))),
            "mock": bool(int(m.group(4))),
        }
        self.mock_active = data["mock"]
        if self._telemetry_callback:
            self._telemetry_callback(data)

    # ---------------------------------------------------------- #
    # Low-level command/response
    # ---------------------------------------------------------- #
    def _send_command(self, command: str) -> str:
        if not self.is_connected:
            raise LEDConnectionFailed("Not connected")

        with self._send_lock:
            while not self._response_queue.empty():
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break

            try:
                self.connection.write((command + "\n").encode())
            except serial.SerialException as e:
                raise LEDConnectionFailed(str(e)) from e

            try:
                response = self._response_queue.get(timeout=self.command_timeout)
            except queue.Empty:
                raise LEDTimeoutError(f"No response to: {command!r}")

        if response.startswith("ERR:"):
            raise LEDCommandError(f"{command!r} -> {response}")
        return response

    # ---------------------------------------------------------- #
    # Identity
    # ---------------------------------------------------------- #
    def identify(self) -> LEDPanelIdentity:
        resp = self._send_command("ID")
        m = self._RSP_ID.match(resp)
        if not m:
            raise LEDCommandError(f"Unexpected response: {resp}")
        device_type, fw_version, unique_id = m.groups()
        return LEDPanelIdentity(device_type, fw_version, unique_id, self.port)

    # ---------------------------------------------------------- #
    # High-level API
    # ---------------------------------------------------------- #
    def set_fan_percent(self, percent: int) -> int:
        percent = int(max(0, min(100, percent)))
        resp = self._send_command(f"SET_FAN:{percent}")
        m = self._RSP_FAN.match(resp)
        if not m:
            raise LEDCommandError(f"Unexpected response: {resp}")
        return int(m.group(1))

    def set_dac_raw(self, channel: int, value: int) -> int:
        if channel not in (0, 1, 2):
            raise ValueError("channel must be 0, 1 or 2")
        value = int(max(0, min(4095, value)))
        resp = self._send_command(f"SET_DAC:{channel + 1},{value}")
        m = self._RSP_DAC_SET.match(resp)
        if not m:
            raise LEDCommandError(f"Unexpected response: {resp}")
        val = int(m.group(2))
        self.dac_values[channel] = val
        return val

    def get_dac_values(self) -> List[int]:
        resp = self._send_command("GET_DAC")
        m = self._RSP_DAC_VALS.match(resp)
        if not m:
            raise LEDCommandError(f"Unexpected response: {resp}")
        vals = [int(m.group(i)) for i in (1, 2, 3)]
        self.dac_values = vals
        return vals

    def get_temperature_degC(self) -> float:
        resp = self._send_command("GET_TEMP")
        m = self._RSP_TMP.match(resp)
        if not m:
            raise LEDCommandError(f"Unexpected response: {resp}")
        return float(m.group(1))

    def set_brightness(self, brightness: Union[float, Iterable[float]]) -> List[int]:
        """Abstract API: brightness in [0, 1], scalar or per-channel iterable."""
        if isinstance(brightness, (int, float)):
            brightness = [brightness] * 3
        brightness = list(brightness)
        if len(brightness) != 3:
            raise ValueError("Expected 3 brightness values (one per channel)")

        values = []
        for ch, b in enumerate(brightness):
            raw = int(round(max(0.0, min(1.0, b)) * 4095))
            values.append(self.set_dac_raw(ch, raw))
        return values

    # ---------------------------------------------------------- #
    # Discovery: probe every serial port for the ID handshake
    # ---------------------------------------------------------- #
    @classmethod
    def _probe_port(
        cls,
        port_device: str,
        baudrate: int,
        boot_timeout: float,
        id_timeout: float,
    ) -> Optional[LEDPanelIdentity]:
        """
        Open `port_device`, wait for it to boot, ask 'ID', and return the
        parsed identity iff the board answers correctly and in time.
        Returns None for anything else (busy port, unrelated device,
        no/garbled response, ...).
        """
        try:
            with serial.Serial(port_device, baudrate=baudrate, timeout=0.2) as ser:
                # Opening the port resets Nano-Every-style boards. Wait for
                # the boot banner, but don't hard-fail if we miss it (board
                # might already have been running before we opened the port).
                deadline = time.monotonic() + boot_timeout
                while time.monotonic() < deadline:
                    line = ser.readline().decode(errors="replace").strip()
                    if line == "SYS:READY":
                        break

                ser.reset_input_buffer()
                ser.write(b"ID\n")

                deadline = time.monotonic() + id_timeout
                while time.monotonic() < deadline:
                    line = ser.readline().decode(errors="replace").strip()
                    if not line:
                        continue
                    m = cls._RSP_ID.match(line)
                    if m:
                        device_type, fw_version, unique_id = m.groups()
                        return LEDPanelIdentity(
                            device_type, fw_version, unique_id, port_device
                        )
                return None
        except (serial.SerialException, OSError, PermissionError):
            return None

    @classmethod
    def list_available_panels(
        cls,
        baudrate: int = 115200,
        device_type: Optional[str] = None,
        boot_timeout: float = 3.0,
        id_timeout: float = 1.0,
        max_workers: int = 8,
        **kwargs,
    ) -> List[LEDPanelInfo]:
        """
        Scan every visible serial port in parallel, keeping only the ones
        that answer the `ID` handshake with a matching device_type. This
        makes discovery robust even when other, unrelated Arduino boards
        (running different sketches) are plugged in at the same time.
        """
        device_type = device_type or cls.DEVICE_TYPE
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            return []

        panels: List[LEDPanelInfo] = []
        with ThreadPoolExecutor(max_workers=min(max_workers, len(ports))) as pool:
            futures = {
                pool.submit(cls._probe_port, port, baudrate, boot_timeout, id_timeout): port
                for port in ports
            }
            for future in as_completed(futures):
                identity = future.result()
                if identity is None or identity.device_type != device_type:
                    continue
                name = (
                    f"{identity.port} \u2014 {identity.device_type} "
                    f"v{identity.firmware_version} [{identity.unique_id}]"
                )
                panels.append(
                    LEDPanelInfo(
                        name=name,
                        ledpanel_cls=cls,
                        kwargs={
                            "port": identity.port,
                            "baudrate": baudrate,
                            "expected_unique_id": identity.unique_id,
                            **kwargs,
                        },
                    )
                )
        return panels

    def __repr__(self):
        return (
            f"<SerialLEDPanel port={self.port!r} connected={self.is_connected} "
            f"unique_id={self.unique_id!r}>"
        )
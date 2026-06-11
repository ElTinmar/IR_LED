from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Tuple, Type, Dict, Any, List, Iterable, Union
import serial
import time

class LEDPanelError(Exception): pass
class LEDConnectionFailed(LEDPanelError): pass

@dataclass
class LEDPanelInfo:
    name: str            
    ledpanel_cls: Type['LEDPanel'] 
    args: Tuple[Any, ...] = field(default_factory=tuple) 
    kwargs: Dict[str, Any] = field(default_factory=dict) 

    def instantiate(self) -> 'LEDPanel':
        return self.ledpanel_cls(*self.args, **self.kwargs) 

class LEDPanel(ABC):

    @classmethod
    @abstractmethod
    def list_available_panels(cls, *args, **kwargs) -> List[LEDPanelInfo]:
        pass

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
        pass

    @abstractmethod
    def get_temperature_degC(self) -> float:
        pass


class SerialLEDPanel(LEDPanel):

    VALID_BAUD_RATES = [2400,4800,9600,14400,19200,38400,115200]

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        data_byte_length = serial.EIGHTBITS,
        parity_check = serial.PARITY_NONE,
        num_stop_bit: int = serial.STOPBITS_ONE,
        timeout: float | None = 10.0,
        write_timeout: float | None = 1.0,
        xonxoff: bool = False,  # Disable Software Flow Control
        rtscts: bool = False,  # Disable Hardware Flow Control (RTS/CTS)
        dsrdtr: bool = False,  # Disable Hardware Flow Control (DSR/DTR)
        verbose: bool = False
    ):

        if baudrate not in self.VALID_BAUD_RATES:
            raise ValueError(f'Supported baud rates are: {self.VALID_BAUD_RATES}')
        
        self.port = port
        self.baudrate = baudrate
        self.data_byte_length = data_byte_length
        self.parity_check = parity_check
        self.num_stop_bit = num_stop_bit
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.xonxoff = xonxoff
        self.rtscts = rtscts
        self.dsrdtr = dsrdtr
        self.verbose = verbose
        self.connection = None

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

            # clear the line
            self.connection.write(b"\r\r\r")
            time.sleep(0.1)
            self.connection.read_all()

        except serial.SerialException as e:
            raise LEDConnectionFailed(f"Failed to connect to {self.port}") from e

    def disconnect(self):

        if self.connection and self.connection.is_open:
            self.connection.close()
        self.connection = None

    def list_available_panels(cls, *args, **kwargs) -> List[LEDPanelInfo]:
        ...

    def set_brightness(self, brightness):
        ...

    def get_temperature_degC(self) -> float:
        ...
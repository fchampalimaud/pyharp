from __future__ import annotations

import logging
import queue
from dataclasses import dataclass
from enum import Enum, IntEnum, IntFlag
from io import BufferedWriter
from pathlib import Path
from typing import Any, Callable, ClassVar, Generic, Literal, Optional, TypeVar

import serial
from harp.protocol import (
    MessageType,
    OperationCtrl,
    PayloadType,
)
from harp.protocol.exceptions import (
    HarpReadException,
    HarpTimeoutException,
    HarpWriteException,
)
from harp.protocol.messages import HarpMessage
from harp.serial.harp_serial import HarpSerial

T = TypeVar("T")


@dataclass(frozen=True)
class Spec(Generic[T]):
    addr: int
    payload_type: PayloadType
    decode: Callable[[Any], T]  # from payload (int|float|list) -> T
    encode: Callable[[T], Any]  # from T -> payload (int|float|list)


# Basic adapters
def _id(x):
    return x


def _enum(enum_cls):
    return lambda v: enum_cls(v), lambda v: int(v)


class TimeoutStrategy(Enum):
    """
    Strategy to handle timeouts when waiting for a reply from the device.

    Attributes
    ----------
    RAISE : str
        Raise HarpTimeoutException
    RETURN_NONE : str
        Return None
    LOG_AND_RAISE : str
        Log the timeout and raise HarpTimeoutException
    LOG_AND_NONE : str
        Log the timeout and return None
    """

    RAISE = "raise"  # Raise HarpTimeoutException
    RETURN_NONE = "return_none"  # Return None
    LOG_AND_RAISE = "log_and_raise"
    LOG_AND_NONE = "log_and_none"


@dataclass
class OperationControlPayload:
    # Specifies the operation mode of the device.
    OperationMode: OperationMode
    # Specifies whether the device should report the content of all registers on initialization.
    DumpRegisters: bool
    # Specifies whether the replies to all commands will be muted, i.e. not sent by the device.
    MuteReplies: bool
    # Specifies the state of all visual indicators on the device.
    VisualIndicators: LedState
    # Specifies whether the device state LED should report the operation mode of the device.
    OperationLed: LedState
    # Specifies whether the device should report the content of the seconds register each second.
    Heartbeat: bool


class ResetFlags(IntFlag):
    """
    Specifies the behavior of the non-volatile registers when resetting the device.

    Attributes
    ----------
    NONE : int
        All reset flags are cleared.
    RESTORE_DEFAULT : int
        The device will boot with all the registers reset to their default factory values.
    RESTORE_EEPROM : int
        The device will boot and restore all the registers to the values stored in non-volatile memory.
    SAVE : int
        The device will boot and save all the current register values to non-volatile memory.
    RESTORE_NAME : int
        The device will boot with the default device name.
    BOOT_FROM_DEFAULT : int
        Specifies that the device has booted from default factory values.
    BOOT_FROM_EEPROM : int
        Specifies that the device has booted from non-volatile values stored in EEPROM.
    """

    NONE = 0x0
    RESTORE_DEFAULT = 0x1
    RESTORE_EEPROM = 0x2
    SAVE = 0x4
    RESTORE_NAME = 0x8
    BOOT_FROM_DEFAULT = 0x40
    BOOT_FROM_EEPROM = 0x80


class ClockConfigurationFlags(IntFlag):
    """
    Specifies configuration flags for the device synchronization clock.

    Attributes
    ----------
    NONE : int
        All clock configuration flags are cleared.
    CLOCK_REPEATER : int
        The device will repeat the clock synchronization signal to the clock output connector, if available.
    CLOCK_GENERATOR : int
        The device resets and generates the clock synchronization signal on the clock output connector, if available.
    REPEATER_CAPABILITY : int
        Specifies the device has the capability to repeat the clock synchronization signal to the clock output connector.
    GENERATOR_CAPABILITY : int
        Specifies the device has the capability to generate the clock synchronization signal to the clock output connector.
    CLOCK_UNLOCK : int
        The device will unlock the timestamp register counter and will accept commands to set new timestamp values.
    CLOCK_LOCK : int
        The device will lock the timestamp register counter and will not accept commands to set new timestamp values.
    """

    NONE = 0x0
    CLOCK_REPEATER = 0x1
    CLOCK_GENERATOR = 0x2
    REPEATER_CAPABILITY = 0x8
    GENERATOR_CAPABILITY = 0x10
    CLOCK_UNLOCK = 0x40
    CLOCK_LOCK = 0x80


class OperationMode(IntEnum):
    """
    Specifies the operation mode of the device.

    Attributes
    ----------
    STANDBY : int
        Disable all event reporting on the device.
    ACTIVE : int
        Event detection is enabled. Only enabled events are reported by the device.
    SPEED : int
        The device enters speed mode.
    """

    STANDBY = 0
    ACTIVE = 1
    SPEED = 3


class EnableFlag(IntEnum):
    """
    Specifies whether a specific register flag is enabled or disabled.

    Attributes
    ----------
    DISABLED : int
        Specifies that the flag is disabled.
    ENABLED : int
        Specifies that the flag is enabled.
    """

    DISABLED = 0
    ENABLED = 1


class LedState(IntEnum):
    """
    Specifies the state of an LED on the device.

    Attributes
    ----------
    OFF : int
        Specifies that the LED is off.
    ON : int
        Specifies that the LED is on.
    """

    OFF = 0
    ON = 1


class WhoAmI(HarpMessage[int]):
    ADDRESS: int = 0

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U16, PayloadType.TIMESTAMPED_U16
        ] = PayloadType.U16,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class HardwareVersionHigh(HarpMessage[int]):
    ADDRESS: int = 1

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class HardwareVersionLow(HarpMessage[int]):
    ADDRESS: int = 2

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class AssemblyVersion(HarpMessage[int]):
    ADDRESS: int = 3

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class CoreVersionHigh(HarpMessage[int]):
    ADDRESS: int = 4

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class CoreVersionLow(HarpMessage[int]):
    ADDRESS: int = 5

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class FirmwareVersionHigh(HarpMessage[int]):
    ADDRESS: int = 6

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class FirmwareVersionLow(HarpMessage[int]):
    ADDRESS: int = 7

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class TimestampSeconds(HarpMessage[int]):
    ADDRESS: int = 8

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U32, PayloadType.TIMESTAMPED_U32
        ] = PayloadType.U32,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class TimestampMicroseconds(HarpMessage[int]):
    ADDRESS: int = 9

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U16, PayloadType.TIMESTAMPED_U16
        ] = PayloadType.U16,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class OperationControl(HarpMessage[int]):
    ADDRESS: int = 10

    def __init__(
        self,
        payload: Optional[OperationControlPayload] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class ResetDevice(HarpMessage[int]):
    ADDRESS: int = 11

    def __init__(
        self,
        payload: Optional[ResetFlags] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class DeviceName(HarpMessage[bytes]):
    ADDRESS: int = 12

    def __init__(
        self,
        payload: Optional[bytes] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class SerialNumber(HarpMessage[int]):
    ADDRESS: int = 13

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U16, PayloadType.TIMESTAMPED_U16
        ] = PayloadType.U16,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class ClockConfiguration(HarpMessage[int]):
    ADDRESS: int = 14

    def __init__(
        self,
        payload: Optional[ClockConfigurationFlags] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class TimestampOffset(HarpMessage[int]):
    ADDRESS: int = 15

    def __init__(
        self,
        payload: Optional[int] = None,
        message_type: MessageType = MessageType.READ,
        payload_type: Literal[
            PayloadType.U8, PayloadType.TIMESTAMPED_U8
        ] = PayloadType.U8,
        *,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        super().__init__(
            message_type, self.ADDRESS, payload_type, payload, timestamp, port
        )


class CommonRegisters(IntEnum):
    """Enum for all available registers in the Common device.

    Attributes
    ----------
    WHO_AM_I : int
        Specifies the identity class of the device.
    HARDWARE_VERSION_HIGH : int
        Specifies the major hardware version of the device.
    HARDWARE_VERSION_LOW : int
        Specifies the minor hardware version of the device.
    ASSEMBLY_VERSION : int
        Specifies the version of the assembled components in the device.
    CORE_VERSION_HIGH : int
        Specifies the major version of the Harp core implemented by the device.
    CORE_VERSION_LOW : int
        Specifies the minor version of the Harp core implemented by the device.
    FIRMWARE_VERSION_HIGH : int
        Specifies the major version of the Harp core implemented by the device.
    FIRMWARE_VERSION_LOW : int
        Specifies the minor version of the Harp core implemented by the device.
    TIMESTAMP_SECONDS : int
        Stores the integral part of the system timestamp, in seconds.
    TIMESTAMP_MICROSECONDS : int
        Stores the fractional part of the system timestamp, in microseconds.
    OPERATION_CONTROL : int
        Stores the configuration mode of the device.
    RESET_DEVICE : int
        Resets the device and saves non-volatile registers.
    DEVICE_NAME : int
        Stores the user-specified device name.
    SERIAL_NUMBER : int
        Specifies the unique serial number of the device.
    CLOCK_CONFIGURATION : int
        Specifies the configuration for the device synchronization clock.
    TIMESTAMP_OFFSET : int
        Specifies an offset value to be added to the device's timestamp if above zero. The register is sensitive to 500 microsecond increments. This register is non-volatile.
    """

    WHO_AM_I = 0
    HARDWARE_VERSION_HIGH = 1
    HARDWARE_VERSION_LOW = 2
    ASSEMBLY_VERSION = 3
    CORE_VERSION_HIGH = 4
    CORE_VERSION_LOW = 5
    FIRMWARE_VERSION_HIGH = 6
    FIRMWARE_VERSION_LOW = 7
    TIMESTAMP_SECONDS = 8
    TIMESTAMP_MICROSECONDS = 9
    OPERATION_CONTROL = 10
    RESET_DEVICE = 11
    DEVICE_NAME = 12
    SERIAL_NUMBER = 13
    CLOCK_CONFIGURATION = 14
    TIMESTAMP_OFFSET = 15


COMMON_REGISTERS = {
    CommonRegisters.WHO_AM_I: WhoAmI,
    CommonRegisters.HARDWARE_VERSION_HIGH: HardwareVersionHigh,
    CommonRegisters.HARDWARE_VERSION_LOW: HardwareVersionLow,
    CommonRegisters.ASSEMBLY_VERSION: AssemblyVersion,
    CommonRegisters.CORE_VERSION_HIGH: CoreVersionHigh,
    CommonRegisters.CORE_VERSION_LOW: CoreVersionLow,
    CommonRegisters.FIRMWARE_VERSION_HIGH: FirmwareVersionHigh,
    CommonRegisters.FIRMWARE_VERSION_LOW: FirmwareVersionLow,
    CommonRegisters.TIMESTAMP_SECONDS: TimestampSeconds,
    CommonRegisters.TIMESTAMP_MICROSECONDS: TimestampMicroseconds,
    CommonRegisters.OPERATION_CONTROL: OperationControl,
    CommonRegisters.RESET_DEVICE: ResetDevice,
    CommonRegisters.DEVICE_NAME: DeviceName,
    CommonRegisters.SERIAL_NUMBER: SerialNumber,
    CommonRegisters.CLOCK_CONFIGURATION: ClockConfiguration,
    CommonRegisters.TIMESTAMP_OFFSET: TimestampOffset,
}


COMMON_SPECS = {
    CommonRegisters.WHO_AM_I: Spec(
        addr=0,
        payload_type=PayloadType.U16,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.HARDWARE_VERSION_HIGH: Spec(
        addr=1,
        payload_type=PayloadType.U8,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.HARDWARE_VERSION_LOW: Spec(
        addr=2,
        payload_type=PayloadType.U8,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.ASSEMBLY_VERSION: Spec(
        addr=3,
        payload_type=PayloadType.U8,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.CORE_VERSION_HIGH: Spec(
        addr=4,
        payload_type=PayloadType.U8,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.CORE_VERSION_LOW: Spec(
        addr=5,
        payload_type=PayloadType.U8,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.FIRMWARE_VERSION_HIGH: Spec(
        addr=6,
        payload_type=PayloadType.U8,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.FIRMWARE_VERSION_LOW: Spec(
        addr=7,
        payload_type=PayloadType.U8,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.TIMESTAMP_SECONDS: Spec(
        addr=8,
        payload_type=PayloadType.U32,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.TIMESTAMP_MICROSECONDS: Spec(
        addr=9,
        payload_type=PayloadType.U16,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.OPERATION_CONTROL: Spec(
        addr=10,
        payload_type=PayloadType.U8,
        decode=lambda payload: OperationControlPayload(
            OperationMode=OperationMode(payload & 0x3),
            DumpRegisters=bool((payload & 0x8) != 0),
            MuteReplies=bool((payload & 0x10) != 0),
            VisualIndicators=LedState((payload & 0x20) != 0),
            OperationLed=LedState((payload & 0x40) != 0),
            Heartbeat=(payload & 0x80) != 0,
        ),
        encode=lambda value: (int(value.OperationMode) & 0x3)
        | (0x8 if bool(value.DumpRegisters) else 0)
        | (0x10 if bool(value.MuteReplies) else 0)
        | (0x20 if bool(value.VisualIndicators) else 0)
        | (0x40 if bool(value.OperationLed) else 0)
        | (0x80 if bool(value.Heartbeat) else 0),
    ),
    CommonRegisters.RESET_DEVICE: Spec(
        addr=11,
        payload_type=PayloadType.U8,
        decode=lambda payload: ResetFlags(payload),
        encode=lambda value: int(value),
    ),
    CommonRegisters.DEVICE_NAME: Spec(
        addr=12,
        payload_type=PayloadType.U8,
        decode=_id,
        encode=_id,
    ),
    CommonRegisters.SERIAL_NUMBER: Spec(
        addr=13,
        payload_type=PayloadType.U16,
        decode=int,
        encode=_id,
    ),
    CommonRegisters.CLOCK_CONFIGURATION: Spec(
        addr=14,
        payload_type=PayloadType.U8,
        decode=lambda payload: ClockConfigurationFlags(payload),
        encode=lambda value: int(value),
    ),
    CommonRegisters.TIMESTAMP_OFFSET: Spec(
        addr=15,
        payload_type=PayloadType.U8,
        decode=int,
        encode=_id,
    ),
}


class Device:
    """
    The `Device` class provides the interface for interacting with Harp devices. This implementation of the Harp device was based on the official documentation available on the [harp-tech website](https://harp-tech.org/protocol/Device.html).

    Attributes
    ----------
    WHO_AM_I : int
        Specifies the identity class of the device.
    HARDWARE_VERSION_HIGH : int
        Specifies the major hardware version of the device.
    HARDWARE_VERSION_LOW : int
        Specifies the minor hardware version of the device.
    ASSEMBLY_VERSION : int
        Specifies the version of the assembled components in the device.
    CORE_VERSION_HIGH : int
        Specifies the major version of the Harp core implemented by the device.
    CORE_VERSION_LOW : int
        Specifies the minor version of the Harp core implemented by the device.
    FIRMWARE_VERSION_HIGH : int
        Specifies the major version of the Harp core implemented by the device.
    FIRMWARE_VERSION_LOW : int
        Specifies the minor version of the Harp core implemented by the device.
    OPERATION_CONTROL : OperationControlPayload
        Stores the configuration mode of the device.
    RESET_DEVICE : ResetFlags
        Resets the device and saves non-volatile registers.
    DEVICE_NAME : bytes
        Stores the user-specified device name.
    SERIAL_NUMBER : int
        Specifies the unique serial number of the device.
    CLOCK_CONFIGURATION : ClockConfigurationFlags
        Specifies the configuration for the device synchronization clock.
    TIMESTAMP_OFFSET : int
        Specifies an offset value to be added to the device's timestamp if above zero. The register is sensitive to 500 microsecond increments. This register is non-volatile.
    """

    COMMON_SPECS: ClassVar[dict[Any, Spec]] = COMMON_SPECS
    DEVICE_SPECS: ClassVar[dict[Any, Spec]] = {}

    COMMON_REGISTERS_TYPE: ClassVar[dict[Any, type[HarpMessage]]] = COMMON_REGISTERS
    DEVICE_REGISTERS_TYPE: ClassVar[dict[Any, type[HarpMessage]]] = {}

    WHO_AM_I: int
    HARDWARE_VERSION_HIGH: int
    HARDWARE_VERSION_LOW: int
    ASSEMBLY_VERSION: int
    CORE_VERSION_HIGH: int
    CORE_VERSION_LOW: int
    FIRMWARE_VERSION_HIGH: int
    FIRMWARE_VERSION_LOW: int
    OPERATION_CONTROL: OperationControlPayload
    RESET_DEVICE: ResetFlags
    DEVICE_NAME: bytes
    SERIAL_NUMBER: int
    CLOCK_CONFIGURATION: ClockConfigurationFlags
    TIMESTAMP_OFFSET: int

    _ser: HarpSerial
    _dump_file_path: Optional[Path]
    _dump_file: Optional[BufferedWriter] = None
    _timeout: float

    def __init__(
        self,
        serial_port: str,
        dump_file_path: Optional[str] = None,
        timeout: float = 1,
        timeout_strategy: TimeoutStrategy = TimeoutStrategy.RAISE,
    ):
        """
        Parameters
        ----------
        serial_port : str
            The serial port used to establish the connection with the Harp device. It must be denoted as `/dev/ttyUSBx` in Linux and `COMx` in Windows, where `x` is the number of the serial port
        dump_file_path: str, optional
            The binary file to which all Harp messages will be written
        timeout: float, optional
            The timeout in seconds when waiting for a reply from the device
        timeout_strategy: TimeoutStrategy, optional
            The strategy to handle timeouts when waiting for a reply from the device
        """
        self.log = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        self._serial_port = serial_port
        self._dump_file_path = None
        if dump_file_path is not None:
            self._dump_file_path = Path() / dump_file_path
        self._timeout = timeout
        self._timeout_strategy = timeout_strategy

        # Connect to the Harp device and load the data stored in the device's common registers
        self.connect()
        self.load()

    def load(self) -> None:
        """
        Loads the data stored in the device's common registers.
        """
        self.WHO_AM_I = self.read_who_am_i()
        self.HARDWARE_VERSION_HIGH = self.read_hardware_version_high()
        self.HARDWARE_VERSION_LOW = self.read_hardware_version_low()
        self.ASSEMBLY_VERSION = self.read_assembly_version()
        self.CORE_VERSION_HIGH = self.read_core_version_high()
        self.CORE_VERSION_LOW = self.read_core_version_low()
        self.FIRMWARE_VERSION_HIGH = self.read_firmware_version_high()
        self.FIRMWARE_VERSION_LOW = self.read_firmware_version_low()
        self.OPERATION_CONTROL = self.read_operation_control()
        self.RESET_DEVICE = self.read_reset_device()
        self.DEVICE_NAME = self.read_device_name()
        self.SERIAL_NUMBER = self.read_serial_number()
        self.CLOCK_CONFIGURATION = self.read_clock_configuration()
        self.TIMESTAMP_OFFSET = self.read_timestamp_offset()

    def info(self) -> None:
        """
        Prints the device information.
        """
        print("Device info:")
        print(f"Who Am I: {self.WHO_AM_I}")
        print(f"Hardware Version High: {self.HARDWARE_VERSION_HIGH}")
        print(f"Hardware Version Low: {self.HARDWARE_VERSION_LOW}")
        print(f"Assembly Version: {self.ASSEMBLY_VERSION}")
        print(f"Core Version High: {self.CORE_VERSION_HIGH}")
        print(f"Core Version Low: {self.CORE_VERSION_LOW}")
        print(f"Firmware Version High: {self.FIRMWARE_VERSION_HIGH}")
        print(f"Firmware Version Low: {self.FIRMWARE_VERSION_LOW}")
        print(f"Operation Control: {self.OPERATION_CONTROL}")
        print(f"Reset Device: {self.RESET_DEVICE}")
        print(f"Device Name: {self.DEVICE_NAME}")
        print(f"Serial Number: {self.SERIAL_NUMBER}")
        print(f"Clock Configuration: {self.CLOCK_CONFIGURATION}")
        print(f"Timestamp Offset: {self.TIMESTAMP_OFFSET}")
        # print(f"* Mode: {self._read_device_mode().name}")

    def connect(self) -> None:
        """
        Connects to the Harp device.
        """
        self._ser = HarpSerial(
            self._serial_port,
            baudrate=1000000,
            timeout=self._timeout,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            rtscts=True,
        )

        # open file if it is defined
        if self._dump_file_path is not None:
            self._dump_file = open(self._dump_file_path, "ab")

    def disconnect(self) -> None:
        """
        Disconnects from the Harp device.
        """
        # close file if it exists
        if self._dump_file:
            self._dump_file.close()
            self._dump_file = None

        self._ser.close()

    def _join_specs(self) -> dict[int, Spec]:
        return self.COMMON_SPECS | self.DEVICE_SPECS

    def _send_checked(self, msg: HarpMessage) -> Optional[HarpMessage]:
        reply = self.send(msg)
        if reply is not None and reply.is_error:
            # Route read vs write exception appropriately
            if msg.message_type == MessageType.READ:
                raise HarpReadException(f"{msg.address}", reply)
            else:
                raise HarpWriteException(f"{msg.address}", reply)
        return reply

    def read_reg(self, reg: CommonRegisters):
        spec = self._join_specs()[reg]
        reply = self._send_checked(
            HarpMessage(MessageType.READ, spec.addr, spec.payload_type)
        )
        if reply is None:
            return None
        return spec.decode(reply.payload)

    def write_reg(self, reg: CommonRegisters, value):
        spec = self._join_specs()[reg]
        payload = spec.encode(value)
        reply = self._send_checked(
            HarpMessage(
                MessageType.WRITE, spec.addr, spec.payload_type, payload=payload
            )
        )
        return spec.decode(reply.payload) if reply is not None else None

    def dump_registers(self) -> list:
        """
        Asserts the DUMP bit to dump the values of all core and app registers
        as Harp Read Reply Messages. More information on the DUMP bit can be found [here](https://harp-tech.org/protocol/Device.html#r_operation_ctrl-u16--operation-mode-configuration).

        Returns
        -------
        list
            The list containing the reply Harp messages for all the device's registers
        """
        address = CommonRegisters.OPERATION_CONTROL
        reg_value = self.send(HarpMessage(MessageType.READ, address, PayloadType.U8))

        if reg_value is None:
            return []

        reg_value = reg_value.payload

        # Assert DUMP bit
        reg_value |= OperationCtrl.DUMP
        self.send(HarpMessage(MessageType.WRITE, address, PayloadType.U8, reg_value))

        # Receive the contents of all registers as Harp Read Reply Messages
        replies = []
        while True:
            msg = self._read()
            if msg is None:
                break
            else:
                replies.append(msg)
                self._dump_reply(msg.frame)
        return replies

    def read_who_am_i(self) -> WhoAmI:
        """
        Reads the WhoAmI register.

        Returns
        -------
        int
            The value stored in the WhoAmI register
        """
        message = WhoAmI()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return WhoAmI(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_who_am_i(self) -> int:
        """
        Reads the WhoAmI register.

        Returns
        -------
        int
            The value stored in the WhoAmI register
        """
        return self.read_reg(CommonRegisters.WHO_AM_I)

    def read_hardware_version_high(self) -> HardwareVersionHigh:
        """
        Reads the HardwareVersionHigh register.

        Returns
        -------
        int
            The value stored in the HardwareVersionHigh register
        """
        message = HardwareVersionHigh()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return HardwareVersionHigh(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_hardware_version_high(self) -> int:
        """
        Reads the HardwareVersionHigh register.

        Returns
        -------
        int
            The value stored in the HardwareVersionHigh register
        """
        return self.read_reg(CommonRegisters.HARDWARE_VERSION_HIGH)

    def read_hardware_version_low(self) -> HardwareVersionLow:
        """
        Reads the HardwareVersionLow register.

        Returns
        -------
        int
            The value stored in the HardwareVersionLow register
        """
        message = HardwareVersionLow()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return HardwareVersionLow(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_hardware_version_low(self) -> int:
        """
        Reads the HardwareVersionLow register.

        Returns
        -------
        int
            The value stored in the HardwareVersionLow register
        """
        return self.read_reg(CommonRegisters.HARDWARE_VERSION_LOW)

    def read_assembly_version(self) -> AssemblyVersion:
        """
        Reads the AssemblyVersion register.

        Returns
        -------
        int
            The value stored in the AssemblyVersion register
        """
        message = AssemblyVersion()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return AssemblyVersion(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_assembly_version(self) -> int:
        """
        Reads the AssemblyVersion register.

        Returns
        -------
        int
            The value stored in the AssemblyVersion register
        """
        return self.read_reg(CommonRegisters.ASSEMBLY_VERSION)

    def read_core_version_high(self) -> CoreVersionHigh:
        """
        Reads the CoreVersionHigh register.

        Returns
        -------
        int
            The value stored in the CoreVersionHigh register
        """
        message = CoreVersionHigh()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return CoreVersionHigh(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_core_version_high(self) -> int:
        """
        Reads the CoreVersionHigh register.

        Returns
        -------
        int
            The value stored in the CoreVersionHigh register
        """
        return self.read_reg(CommonRegisters.CORE_VERSION_HIGH)

    def read_core_version_low(self) -> CoreVersionLow:
        """
        Reads the CoreVersionLow register.

        Returns
        -------
        int
            The value stored in the CoreVersionLow register
        """
        message = CoreVersionLow()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return CoreVersionLow(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_core_version_low(self) -> int:
        """
        Reads the CoreVersionLow register.

        Returns
        -------
        int
            The value stored in the CoreVersionLow register
        """
        return self.read_reg(CommonRegisters.CORE_VERSION_LOW)

    def read_firmware_version_high(self) -> FirmwareVersionHigh:
        """
        Reads the FirmwareVersionHigh register.

        Returns
        -------
        int
            The value stored in the FirmwareVersionHigh register
        """
        message = FirmwareVersionHigh()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return FirmwareVersionHigh(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_firmware_version_high(self) -> int:
        """
        Reads the FirmwareVersionHigh register.

        Returns
        -------
        int
            The value stored in the FirmwareVersionHigh register
        """
        return self.read_reg(CommonRegisters.FIRMWARE_VERSION_HIGH)

    def read_firmware_version_low(self) -> FirmwareVersionLow:
        """
        Reads the FirmwareVersionLow register.

        Returns
        -------
        int
            The value stored in the FirmwareVersionLow register
        """
        message = FirmwareVersionLow()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return FirmwareVersionLow(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_firmware_version_low(self) -> int:
        """
        Reads the FirmwareVersionLow register.

        Returns
        -------
        int
            The value stored in the FirmwareVersionLow register
        """
        return self.read_reg(CommonRegisters.FIRMWARE_VERSION_LOW)

    def read_timestamp_seconds(self) -> TimestampSeconds:
        """
        Reads the TimestampSeconds register.

        Returns
        -------
        int
            The value stored in the TimestampSeconds register
        """
        message = TimestampSeconds()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return TimestampSeconds(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_timestamp_seconds(self) -> int:
        """
        Reads the TimestampSeconds register.

        Returns
        -------
        int
            The value stored in the TimestampSeconds register
        """
        return self.read_reg(CommonRegisters.TIMESTAMP_SECONDS)

    def write_timestamp_seconds(self, value: int) -> TimestampSeconds:
        """
        Writes a value to the TimestampSeconds register.

        Parameters
        ----------
        value : int
            The value to be written to the TimestampSeconds register
        """
        message = TimestampSeconds(
            message_type=MessageType.WRITE,
            payload=value,
        )
        reply = self.send(message)
        if reply is not None:
            return TimestampSeconds(
                reply.payload,
                reply.message_type,
                reply.payload_type,
                timestamp=reply.timestamp,
                port=reply.port,
            )
        return None

    def write2_timestamp_seconds(self, value: int) -> int:
        """
        Writes a value to the TimestampSeconds register.

        Parameters
        ----------
        value : int
            The value to be written to the TimestampSeconds register
        """
        return self.write_reg(CommonRegisters.TIMESTAMP_SECONDS, value)

    def read_timestamp_microseconds(self) -> TimestampMicroseconds:
        """
        Reads the TimestampMicroseconds register.

        Returns
        -------
        int
            The value stored in the TimestampMicroseconds register
        """
        message = TimestampMicroseconds()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return TimestampMicroseconds(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_timestamp_microseconds(self) -> int:
        """
        Reads the TimestampMicroseconds register.

        Returns
        -------
        int
            The value stored in the TimestampMicroseconds register
        """
        return self.read_reg(CommonRegisters.TIMESTAMP_MICROSECONDS)

    def read_operation_control(self) -> OperationControl:
        """
        Reads the OperationControl register.

        Returns
        -------
        OperationControlPayload
            The value stored in the OperationControl register
        """
        message = OperationControl()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return OperationControl(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_operation_control(self) -> OperationControlPayload:
        """
        Reads the OperationControl register.

        Returns
        -------
        OperationControlPayload
            The value stored in the OperationControl register
        """
        return self.read_reg(CommonRegisters.OPERATION_CONTROL)

    def write_operation_control(
        self, value: OperationControlPayload
    ) -> OperationControl:
        """
        Writes a value to the OperationControl register.

        Parameters
        ----------
        value : OperationControlPayload
            The value to be written to the OperationControl register
        """
        message = OperationControl(
            message_type=MessageType.WRITE,
            payload=value,
        )
        reply = self.send(message)
        if reply is not None:
            return OperationControl(
                reply.payload,
                reply.message_type,
                reply.payload_type,
                timestamp=reply.timestamp,
                port=reply.port,
            )
        return None

    def write2_operation_control(
        self, value: OperationControlPayload
    ) -> OperationControlPayload:
        """
        Writes a value to the OperationControl register.

        Parameters
        ----------
        value : OperationControlPayload
            The value to be written to the OperationControl register
        """
        return self.write_reg(CommonRegisters.OPERATION_CONTROL, value)

    def read_reset_device(self) -> ResetDevice:
        """
        Reads the ResetDevice register.

        Returns
        -------
        ResetFlags
            The value stored in the ResetDevice register
        """
        message = ResetDevice()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return ResetDevice(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_reset_device(self) -> ResetFlags:
        """
        Reads the ResetDevice register.

        Returns
        -------
        ResetFlags
            The value stored in the ResetDevice register
        """
        return self.read_reg(CommonRegisters.RESET_DEVICE)

    def write_reset_device(self, value: ResetFlags) -> ResetDevice:
        """
        Writes a value to the ResetDevice register.

        Parameters
        ----------
        value : ResetFlags
            The value to be written to the ResetDevice register
        """
        message = ResetDevice(
            message_type=MessageType.WRITE,
            payload=value,
        )
        reply = self.send(message)
        if reply is not None:
            return ResetDevice(
                reply.payload,
                reply.message_type,
                reply.payload_type,
                timestamp=reply.timestamp,
                port=reply.port,
            )
        return None

    def write2_reset_device(self, value: ResetFlags) -> ResetFlags:
        """
        Writes a value to the ResetDevice register.

        Parameters
        ----------
        value : ResetFlags
            The value to be written to the ResetDevice register
        """
        return self.write_reg(CommonRegisters.RESET_DEVICE, value)

    def read_device_name(self) -> DeviceName:
        """
        Reads the DeviceName register.

        Returns
        -------
        bytes
            The value stored in the DeviceName register
        """
        message = DeviceName()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return DeviceName(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_device_name(self) -> bytes:
        """
        Reads the DeviceName register.

        Returns
        -------
        bytes
            The value stored in the DeviceName register
        """
        return self.read_reg(CommonRegisters.DEVICE_NAME)

    def write_device_name(self, value: bytes) -> DeviceName:
        """
        Writes a value to the DeviceName register.

        Parameters
        ----------
        value : bytes
            The value to be written to the DeviceName register
        """
        message = DeviceName(
            message_type=MessageType.WRITE,
            payload=value,
        )
        reply = self.send(message)
        if reply is not None:
            return DeviceName(
                reply.payload,
                reply.message_type,
                reply.payload_type,
                timestamp=reply.timestamp,
                port=reply.port,
            )
        return None

    def write2_device_name(self, value: bytes) -> bytes:
        """
        Writes a value to the DeviceName register.

        Parameters
        ----------
        value : bytes
            The value to be written to the DeviceName register
        """
        return self.write_reg(CommonRegisters.DEVICE_NAME, value)

    def read_serial_number(self) -> SerialNumber:
        """
        Reads the SerialNumber register.

        Returns
        -------
        int
            The value stored in the SerialNumber register
        """
        message = SerialNumber()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return SerialNumber(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_serial_number(self) -> int:
        """
        Reads the SerialNumber register.

        Returns
        -------
        int
            The value stored in the SerialNumber register
        """
        return self.read_reg(CommonRegisters.SERIAL_NUMBER)

    def write_serial_number(self, value: int) -> SerialNumber:
        """
        Writes a value to the SerialNumber register.

        Parameters
        ----------
        value : int
            The value to be written to the SerialNumber register
        """
        message = SerialNumber(
            message_type=MessageType.WRITE,
            payload=value,
        )
        reply = self.send(message)
        if reply is not None:
            return SerialNumber(
                reply.payload,
                reply.message_type,
                reply.payload_type,
                timestamp=reply.timestamp,
                port=reply.port,
            )
        return None

    def write2_serial_number(self, value: int) -> int:
        """
        Writes a value to the SerialNumber register.

        Parameters
        ----------
        value : int
            The value to be written to the SerialNumber register
        """
        return self.write_reg(CommonRegisters.SERIAL_NUMBER, value)

    def read_clock_configuration(self) -> ClockConfiguration:
        """
        Reads the ClockConfiguration register.

        Returns
        -------
        ClockConfigurationFlags
            The value stored in the ClockConfiguration register
        """
        message = ClockConfiguration()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return ClockConfiguration(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_clock_configuration(self) -> ClockConfigurationFlags:
        """
        Reads the ClockConfiguration register.

        Returns
        -------
        ClockConfigurationFlags
            The value stored in the ClockConfiguration register
        """
        return self.read_reg(CommonRegisters.CLOCK_CONFIGURATION)

    def write_clock_configuration(
        self, value: ClockConfigurationFlags
    ) -> ClockConfiguration:
        """
        Writes a value to the ClockConfiguration register.

        Parameters
        ----------
        value : ClockConfigurationFlags
            The value to be written to the ClockConfiguration register
        """
        message = ClockConfiguration(
            message_type=MessageType.WRITE,
            payload=value,
        )
        reply = self.send(message)
        if reply is not None:
            return ClockConfiguration(
                reply.payload,
                reply.message_type,
                reply.payload_type,
                timestamp=reply.timestamp,
                port=reply.port,
            )
        return None

    def write2_clock_configuration(
        self, value: ClockConfigurationFlags
    ) -> ClockConfigurationFlags:
        """
        Writes a value to the ClockConfiguration register.

        Parameters
        ----------
        value : ClockConfigurationFlags
            The value to be written to the ClockConfiguration register
        """
        return self.write_reg(CommonRegisters.CLOCK_CONFIGURATION, value)

    def read_timestamp_offset(self) -> TimestampOffset:
        """
        Reads the TimestampOffset register.

        Returns
        -------
        int
            The value stored in the TimestampOffset register
        """
        message = TimestampOffset()
        reply = self.send(message)
        if reply is None:
            raise HarpTimeoutException(self._timeout, message)

        return TimestampOffset(
            reply.payload,
            payload_type=reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

    def read2_timestamp_offset(self) -> int:
        """
        Reads the TimestampOffset register.

        Returns
        -------
        int
            The value stored in the TimestampOffset register
        """
        return self.read_reg(CommonRegisters.TIMESTAMP_OFFSET)

    def write_timestamp_offset(self, value: int) -> TimestampOffset:
        """
        Writes a value to the TimestampOffset register.

        Parameters
        ----------
        value : int
            The value to be written to the TimestampOffset register
        """
        message = TimestampOffset(
            message_type=MessageType.WRITE,
            payload=value,
        )
        reply = self.send(message)
        if reply is not None:
            return TimestampOffset(
                reply.payload,
                reply.message_type,
                reply.payload_type,
                timestamp=reply.timestamp,
                port=reply.port,
            )
        return None

    def write2_timestamp_offset(self, value: int) -> int:
        """
        Writes a value to the TimestampOffset register.

        Parameters
        ----------
        value : int
            The value to be written to the TimestampOffset register
        """
        return self.write_reg(CommonRegisters.TIMESTAMP_OFFSET, value)

    def send(
        self,
        message: HarpMessage,
        *,
        expect_reply: bool = True,
        timeout_strategy: TimeoutStrategy | None = None,
    ) -> HarpMessage | None:
        """
        Sends a Harp message and (optionally) waits for a reply.

        Parameters
        ----------
        message : HarpMessage
            The HarpMessage to be sent to the device
        expect_reply : bool, optional
            If False, do not wait for a reply (fire-and-forget)
        timeout_strategy : TimeoutStrategy | None
            Override the device-level timeout strategy for this call

        Returns
        -------
        HarpMessage | None
            Reply (or None when allowed by the timeout strategy or expect_reply=False)

        Raises
        -------
        HarpTimeoutException
            If no reply is received and the effective strategy requires raising
        """
        if (
            type(message) is not HarpMessage
            and any(isinstance(message, t) for t in self.COMMON_REGISTERS_TYPE.values())
            and any(isinstance(message, t) for t in self.DEVICE_REGISTERS_TYPE.values())
        ):
            raise TypeError("message must be a HarpMessage instance")

        self._ser.write(message.frame)

        if not expect_reply:
            return None

        strategy = timeout_strategy or self._timeout_strategy

        reply = self._read()
        if reply is None:
            hte = HarpTimeoutException(self._timeout, message)
            if strategy in (
                TimeoutStrategy.LOG_AND_RAISE,
                TimeoutStrategy.LOG_AND_NONE,
            ):
                self.log.warning(str(hte))
            if strategy in (TimeoutStrategy.RAISE, TimeoutStrategy.LOG_AND_RAISE):
                raise hte
        else:
            self._dump_reply(reply.frame)
        return reply

    def _read(self) -> HarpMessage | None:
        """
        Reads an incoming serial message in a blocking way.

        Returns
        -------
        HarpMessage | None
            The incoming Harp message in case it exists

        Raises
        -------
        TimeoutError
            If no reply is received within the timeout period
        """
        try:
            return self._ser.msg_q.get(block=True, timeout=self._timeout)
        except queue.Empty:
            return None

    def _dump_reply(self, reply: bytearray):
        """
        Dumps the reply to a Harp message in the dump file in case it exists.
        """
        if self._dump_file:
            self._dump_file.write(reply)

    def get_events(self) -> list[HarpMessage]:
        """
        Gets all events from the event queue.

        Returns
        -------
        list
            The list containing every Harp event message that were on the queue
        """
        msgs = []
        while True:
            try:
                msg = self._ser.event_q.get(timeout=False)
                self._dump_reply(msg.frame)
                msgs.append(msg)
            except queue.Empty:
                break
        return msgs

    def event_count(self) -> int:
        """
        Gets the number of events in the event queue.

        Returns
        -------
        int
            The number of events in the event queue
        """
        return self._ser.event_q.qsize()

    def __enter__(self):
        """
        Support for using Device with 'with' statement.

        Returns
        -------
        Device
            The Device instance
        """
        # Connection is already established in __init__
        # but we could add additional setup if needed
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Cleanup resources when exiting the 'with' block.

        Parameters
        ----------
        exc_type : Exception type or None
            Type of the exception that caused the context to be exited
        exc_val : Exception or None
            Exception instance that caused the context to be exited
        exc_tb : traceback or None
            Traceback if an exception occurred
        """
        self.disconnect()
        # Return False to propagate exceptions that occurred in the with block
        return False

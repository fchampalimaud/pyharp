from __future__ import annotations

import logging
import queue
from dataclasses import dataclass
from enum import Enum, IntEnum, IntFlag
from io import BufferedWriter
from pathlib import Path
from typing import Literal, Optional

import serial
from harp.protocol import (
    CommonRegisters,
    MessageType,
    OperationCtrl,
    PayloadType,
)
from harp.protocol.exceptions import HarpTimeoutException
from harp.protocol.messages import HarpMessage
from harp.serial.harp_serial import HarpSerial


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


COMMON_REGISTERS = {
    0: WhoAmI,
    1: HardwareVersionHigh,
    2: HardwareVersionLow,
    3: AssemblyVersion,
    4: CoreVersionHigh,
    5: CoreVersionLow,
    6: FirmwareVersionHigh,
    7: FirmwareVersionLow,
    8: TimestampSeconds,
    9: TimestampMicroseconds,
    10: OperationControl,
    11: ResetDevice,
    12: DeviceName,
    13: SerialNumber,
    14: ClockConfiguration,
    15: TimestampOffset,
}


class Device:
    """
    The `Device` class provides the interface for interacting with Harp devices. This implementation of the Harp device was based on the official documentation available on the [harp-tech website](https://harp-tech.org/protocol/Device.html).

    Attributes
    ----------
    WHO_AM_I : int
        The device ID number. A list of devices can be found [here](https://github.com/harp-tech/protocol/blob/main/whoami.md)
    HW_VERSION_H : int
        The major hardware version
    HW_VERSION_L : int
        The minor hardware version
    ASSEMBLY_VERSION : int
        The version of the assembled components
    HARP_VERSION_H : int
        The major Harp core version
    HARP_VERSION_L : int
        The minor Harp core version
    FIRMWARE_VERSION_H : int
        The major firmware version
    FIRMWARE_VERSION_L : int
        The minor firmware version
    DEVICE_NAME : str
        The device name stored in the Harp device
    SERIAL_NUMBER : int, optional
        The serial number of the device
    """

    WHO_AM_I: int
    HW_VERSION_H: int
    HW_VERSION_L: int
    ASSEMBLY_VERSION: int
    CORE_VERSION_H: int
    CORE_VERSION_L: int
    FIRMWARE_VERSION_H: int
    FIRMWARE_VERSION_L: int
    DEVICE_NAME: str
    SERIAL_NUMBER: int
    CLOCK_CONFIG: int
    TIMESTAMP_OFFSET: int

    COMMON_REGISTERS_TYPE = COMMON_REGISTERS
    DEVICE_REGISTERS_TYPE = {}

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
        self.HW_VERSION_H = self.read_hardware_version_high()
        self.HW_VERSION_L = self.read_hardware_version_low()
        self.ASSEMBLY_VERSION = self.read_assembly_version()
        self.CORE_VERSION_H = self.read_core_version_high()
        self.CORE_VERSION_L = self.read_core_version_low()
        self.FIRMWARE_VERSION_H = self.read_firmware_version_high()
        self.FIRMWARE_VERSION_L = self.read_firmware_version_low()
        self.DEVICE_NAME = self.read_device_name()
        self.SERIAL_NUMBER = self.read_serial_number()
        self.CLOCK_CONFIG = self.read_clock_configuration()
        self.TIMESTAMP_OFFSET = self.read_timestamp_offset()

    def info(self) -> None:
        """
        Prints the device information.
        """
        print("Device info:")
        print(f"* Who am I: ({self.WHO_AM_I})")
        print(f"* HW version: {self.HW_VERSION_H}.{self.HW_VERSION_L}")
        print(f"* Assembly version: {self.ASSEMBLY_VERSION}")
        print(f"* HARP version: {self.CORE_VERSION_H}.{self.CORE_VERSION_L}")
        print(
            f"* Firmware version: {self.FIRMWARE_VERSION_H}.{self.FIRMWARE_VERSION_L}"
        )
        print(f"* Device name: {self.DEVICE_NAME}")
        print(f"* Serial number: {self.SERIAL_NUMBER}")
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

    def dump_registers(self) -> list:
        """
        Asserts the DUMP bit to dump the values of all core and app registers
        as Harp Read Reply Messages. More information on the DUMP bit can be found [here](https://harp-tech.org/protocol/Device.html#r_operation_ctrl-u16--operation-mode-configuration).

        Returns
        -------
        list
            The list containing the reply Harp messages for all the device's registers
        """
        address = CommonRegisters.OPERATION_CTRL
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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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
            reply.payload_type,
            timestamp=reply.timestamp,
            port=reply.port,
        )

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

from __future__ import annotations

import math
import struct
from typing import Generic, Optional, TypeVar, Union

from harp.protocol import MessageType, PayloadType

T = TypeVar("T")


class HarpMessage(Generic[T]):
    """
    The `HarpMessage` class implements the Harp message as described in the [protocol](https://harp-tech.org/protocol/BinaryProtocol-8bit.html).

    Attributes
    ----------
    frame : bytearray
        The bytearray containing the whole Harp message
    message_type : MessageType
        The message type
    length : int
        The length parameter of the Harp message
    address : int
        The address of the register to which the Harp message refers to
    port : int
        Indicates the origin or destination of the Harp message in case the device is a hub of Harp devices. The value 255 points to the device itself (default value).
    payload_type : PayloadType
        The payload type
    checksum : int
        The sum of all bytes contained in the Harp message
    """

    BASE_LENGTH: int = 4
    _frame: bytearray = bytearray()
    _timestamp: Optional[float] = None

    def __init__(
        self,
        message_type: MessageType,
        address: int,
        payload_type: PayloadType,
        payload: Optional[T] = None,
        timestamp: Optional[float] = None,
        port: int = 255,
    ):
        """
        Parameters
        ----------
        message_type : MessageType
            The message type.
        payload_type : PayloadType
            The payload type.
        address : int
            The address of the register that the message will interact with.
        value: int | list[int] | float | list[float], optional
            The payload of the message. If message_type == MessageType.WRITE, the value cannot be None
        """
        self._payload = payload
        raw_timestamp = self._get_raw_timestamp(payload_type, timestamp)
        raw_payload = self._get_raw_payload(payload_type, payload)

        self._frame = bytearray()
        self._frame.append(message_type)
        self._frame.append(self.BASE_LENGTH + len(raw_timestamp) + len(raw_payload))
        self._frame.append(address)
        self._frame.append(port)
        self._frame.append(payload_type)
        self._frame += raw_timestamp
        self._frame += raw_payload
        self._frame.append(self._calculate_checksum())

    def _get_raw_timestamp(
        self,
        payload_type: PayloadType,
        timestamp: Optional[float],
    ) -> bytearray:
        if payload_type.has_timestamp() and timestamp is None:
            from harp.protocol.exceptions import HarpException

            raise HarpException(
                "The payload type provided indicates the message should have a timestamp, but a timestamp was not provided."
            )

        raw_timestamp = bytearray()

        if timestamp is not None:
            seconds = int(math.floor(timestamp))
            raw_timestamp += seconds.to_bytes(length=4, byteorder="little")
            microseconds = int((timestamp - seconds) / (32 * 10**-6))
            raw_timestamp += microseconds.to_bytes(length=2, byteorder="little")

        return raw_timestamp

    # TODO: improve function
    def _get_raw_payload(
        self,
        payload_type: PayloadType,
        payload: Optional[T],
    ) -> bytearray:
        if payload_type.is_float() and not (
            isinstance(payload, float)
            or (
                isinstance(payload, list)
                and all(isinstance(item, float) for item in payload)
            )
            or payload is None
        ):
            from harp.protocol.exceptions import HarpException

            raise HarpException(
                "The payload type provided indicates the payload should be a float or a list[float], but the payload provided is not."
            )
        elif not payload_type.is_float() and not (
            isinstance(payload, int)
            or (
                isinstance(payload, list)
                and all(isinstance(item, int) for item in payload)
            )
            or payload is None
        ):
            from harp.protocol.exceptions import HarpException

            raise HarpException(
                "The payload type provided indicates the payload should be an int or a list[int], but the payload provided is not."
            )

        raw_payload = bytearray()

        if payload is not None:
            if isinstance(payload, int) or isinstance(payload, float):
                values = [payload]
            elif isinstance(payload, list):
                values = payload

            for val in values:
                if isinstance(val, float):
                    raw_payload += struct.pack("<f", val)
                else:
                    raw_payload += val.to_bytes(
                        payload_type.type_size(),
                        byteorder="little",
                        signed=payload_type.is_signed(),
                    )

        return raw_payload

    def _calculate_checksum(self) -> int:
        """
        Calculates the checksum of the Harp message.

        Returns
        -------
        int
            The value of the checksum
        """
        checksum: int = 0
        for i in self.frame:
            checksum += i
        return checksum & 255

    @property
    def frame(self) -> bytearray:
        """
        The bytearray containing the whole Harp message.

        Returns
        -------
        bytearray
            The bytearray containing the whole Harp message
        """
        return self._frame

    @property
    def message_type(self) -> MessageType:
        """
        The message type.

        Returns
        -------
        MessageType
            The message type
        """
        return MessageType(self._frame[0])

    @property
    def length(self) -> int:
        """
        The length parameter of the Harp message.

        Returns
        -------
        int
            The length parameter of the Harp message
        """
        return self._frame[1]

    @property
    def address(self) -> int:
        """
        The address of the register to which the Harp message refers to.

        Returns
        -------
        int
            The address of the register to which the Harp message refers to
        """
        return self._frame[2]

    @property
    def port(self) -> int:
        """
        Indicates the origin or destination of the Harp message in case the device is a hub of Harp devices. The value 255 points to the device itself (default value).

        Returns
        -------
        int
            The port value
        """
        return self._frame[3]

    @property
    def payload_type(self) -> PayloadType:
        """
        The payload type.

        Returns
        -------
        PayloadType
            The payload type
        """
        return PayloadType(self._frame[4])

    @property
    def timestamp(self) -> Optional[float]:
        if self.payload_type.has_timestamp():
            return (
                int.from_bytes(self.frame[5:9], byteorder="little", signed=False)
                + int.from_bytes(self.frame[9:11], byteorder="little", signed=False)
                * 32e-6
            )
        return None

    @property
    def payload(self) -> Optional[T]:
        """
        The payload sent in the write Harp message.

        Returns
        -------
        Union[int, list[int]]
            The payload sent in the write Harp message
        """
        return self._payload

    @property
    def checksum(self) -> int:
        """
        The sum of all bytes contained in the Harp message.

        Returns
        -------
        int
            The sum of all bytes contained in the Harp message
        """
        return self._frame[-1]

    @property
    def is_error(self) -> bool:
        """
        Indicates if this HarpMessage is an error message or not.

        Returns
        -------
        bool
            Returns True if this HarpMessage is an error message, False otherwise.
        """
        return self.message_type.is_error()

    # def payload_as_string(self) -> str:
    #     """
    #     Returns the payload as a str.

    #     Returns
    #     -------
    #     str
    #         The payload parsed as a str
    #     """
    #     return self._raw_payload.decode("utf-8").rstrip("\x00")

    def __repr__(self) -> str:
        """
        Prints debug representation of the reply message.

        Returns
        -------
        str
            The debug representation of the reply message
        """
        return self.__str__() + f"\r\nRaw Frame: {self.frame}"

    def __str__(self) -> str:
        """
        Prints friendly representation of a Harp message.

        Returns
        -------
        str
            The representation of the Harp message
        """
        payload_str = ""
        format_str = ""
        if self.payload_type in [PayloadType.FLOAT, PayloadType.TIMESTAMPED_FLOAT]:
            format_str = ".6f"
        else:
            bytes_per_word = self.payload_type & 0x07
            format_str = f"0{bytes_per_word}b"

        payload_str = "".join(
            f"{item:{format_str}} "
            for item in (
                self.payload if isinstance(self.payload, list) else [self.payload]
            )
        )

        # Check if the object has a 'timestamp' property and it's not None
        timestamp_line = ""
        if hasattr(self, "timestamp"):
            ts = getattr(self, "timestamp")
            if ts is not None:
                timestamp_line = f"Timestamp: {ts}\r\n"

        return (
            f"Type: {self.message_type.name}\r\n"
            + f"Length: {self.length}\r\n"
            + f"Address: {self.address}\r\n"
            + f"Port: {self.port}\r\n"
            + timestamp_line
            + f"Payload Type: {self.payload_type.name}\r\n"
            + f"Payload Length: {len(self.payload) if self.payload is list else 1}\r\n"
            + f"Payload: {payload_str}\r\n"
            + f"Checksum: {self.checksum}"
        )


def convert_from_message_bytes(message: bytearray) -> HarpMessage:
    payload_type = PayloadType(message[4])

    timestamp = None
    raw_payload = message[5:-1]
    if payload_type.has_timestamp():
        timestamp = (
            int.from_bytes(message[5:9], byteorder="little", signed=False)
            + int.from_bytes(message[9:11], byteorder="little", signed=False) * 32e-6
        )
        raw_payload = message[11:-1]

    payload = None
    if len(raw_payload) != 0:
        payload = _get_payload(payload_type, raw_payload)

    return HarpMessage(
        message_type=MessageType(message[0]),
        address=message[2],
        payload_type=PayloadType(message[4]),
        payload=payload,
        timestamp=timestamp,
        port=message[3],
    )


def _get_payload(
    payload_type: PayloadType, raw_payload: bytearray
) -> Union[int, float, list[int], list[float]]:
    type_size = payload_type & 0b1111

    if payload_type.is_float():
        if len(raw_payload) == 4:
            return struct.unpack("<f", raw_payload)[0]
        else:
            return [
                struct.unpack("<f", raw_payload[i : i + 4])[0]
                for i in range(len(raw_payload), 4)
            ]
    else:
        if len(raw_payload) == type_size:
            return int.from_bytes(
                raw_payload, byteorder="little", signed=payload_type.is_signed()
            )
        else:
            return [
                int.from_bytes(
                    raw_payload[i : i + type_size],
                    byteorder="little",
                    signed=payload_type.is_signed(),
                )
                for i in range(0, len(raw_payload), type_size)
            ]

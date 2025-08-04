# The MIT License (MIT)
#
# Copyright (c) 2018 Dean Miller for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# pylint: disable=missing-docstring,invalid-name,too-many-public-methods

"""
`adafruit_seesaw.encoder`
====================================================
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import ClassVar, List


try:
    from micropython import const
except ImportError:
    def const(x):
        return x


from adafruit_seesaw.seesaw import Seesaw


__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/georgeharker/Adafruit_CircuitPython_seesaw.git"

_ENCODER_BASE = const(0x11)


_ENCODER_STATUS = const(0x00)
_ENCODER_EVENT = const(0x10)
_ENCODER_INTENSET = const(0x20)
_ENCODER_INTENCLR = const(0x30)
_ENCODER_POSITION = const(0x40)
_ENCODER_DELTA = const(0x50)
_ENCODER_COUNT = const(0x60)
_ENCODER_FIFO = const(0x70)

_NUM_ENCODERS = const(8)


class ResponseType(IntEnum):
    # Types for the repsonse
    TYPE_VALUE = 0,
    TYPE_DELTA = 1,
    TYPE_PRESS = 2,
    TYPE_COUNT = 3,
    TYPE_STATUS = 4,
    TYPE_INVALID = 0xff


class EncoderError(Exception):
    pass


class EncoderEdge(IntEnum):
    #: Indicates that the switch is currently pressed
    EDGE_HIGH = 0
    #: Indicates that the switch is currently released
    EDGE_LOW = 1
    #: Indicates that the switch was recently pressed
    EDGE_FALLING = 2
    #: Indicates that the switch was recently released
    EDGE_RISING = 3
    #: Indicates that the value was recently changed
    VALUE_CHANGE = 4
    #: Indicates that the delta was recently updated
    DELTA = 5


class EncoderEventType(IntEnum):
    PRESS = 0
    VALUE = 1
    DELTA = 2


@dataclass
class EncoderEvent:
    """Holds information about an encoder event in its properties

       :param event_type: the event type
       :param int num: The number of the encoder
    """
    event_type: int  # EncoderEventType
    number: int


@dataclass
class EncoderPressEvent(EncoderEvent):
    """Holds information about an encoder value event in its properties

       :param int edge: One of the EDGE propertes of `EncoderEdge`
    """
    edge: int  # EncoderEdge


@dataclass
class EncoderValueEvent(EncoderEvent):
    """Holds information about an encoder value event in its properties

       :param int value: The value of the encoder
    """
    value: int


@dataclass
class EncoderDeltaEvent(EncoderEvent):
    """Holds information about an encoder event in its properties

       :param int delta: The delta of the encoder
    """
    delta: int


@dataclass
class SeesawEncoderResponse:
    response_type: ResponseType
    enc: int
    data: int

    def __post_init__(self):
        try:
            self.response_type = ResponseType(self.response_type)
        except Exception as e:  # noqa: F841
            # print(e)
            self.responseType = ResponseType.TYPE_INVALID

    unpacker: ClassVar[struct.Struct] = struct.Struct('<BBh')

    @classmethod
    def unpack(cls, buf: bytearray):
        return SeesawEncoderResponse(*cls.unpacker.unpack(buf))

    @classmethod
    def unpack_from(cls, buf: bytearray, frm: int):
        return SeesawEncoderResponse(*cls.unpacker.unpack_from(buf, frm))

    def data_encoderpressevent(self) -> EncoderPressEvent:
        return EncoderPressEvent(
            event_type=EncoderEventType.PRESS,
            number=self.enc, edge=self.data)

    def data_encoderdeltaevent(self) -> EncoderDeltaEvent:
        return EncoderDeltaEvent(
            event_type=EncoderEventType.DELTA,
            number=self.enc, delta=self.data)

    def data_encodervalueevent(self) -> EncoderValueEvent:
        return EncoderValueEvent(
            event_type=EncoderEventType.VALUE,
            number=self.enc, value=self.data)

    def data_encoderevent(self) -> EncoderEvent:
        if self.response_type == ResponseType.TYPE_PRESS:
            return self.data_encoderpressevent()
        elif self.response_type == ResponseType.TYPE_DELTA:
            return self.data_encoderdeltaevent()
        elif self.response_type == ResponseType.TYPE_VALUE:
            return self.data_encodervalueevent()
        raise AssertionError("Bad response type")


# pylint: enable=too-few-public-methods


class Encoder(Seesaw):
    """On compatible SeeSaw devices, reads from an encoder.

       :param ~busio.I2C i2c_bus: Bus the SeeSaw is connected to
       :param int addr: I2C address of the SeeSaw device
       :param ~digitalio.DigitalInOut drdy: Pin connected to SeeSaw's 'ready' output"""

    packer: ClassVar[struct.Struct] = struct.Struct('<I')

    def __init__(self, i2c_bus, addr: int = 0x49, drdy=None, num_encoders: int = _NUM_ENCODERS):
        super().__init__(i2c_bus, addr, drdy,
                         rd_delay=0.0005, wr_delay=0.0005)
        self._interrupt_enabled = False
        self._num_encoders = num_encoders
        self._tx_errors = 0
        self._tx_count = 0

    def _read_reg(self, reg: int, response_type: ResponseType):
        try:
            self._tx_count += 1
            buf = self.readn(_ENCODER_BASE, reg, 4)
            d = SeesawEncoderResponse.unpack(buf)
            if d.response_type != response_type:
                raise EncoderError(f'CORRUPTED {list([f"{x:x}" for x in buf])}')
        except OSError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            raise e
        except EncoderError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            raise e
        return d.data

    def _write_reg(self, reg: int, cmd: bytearray | bytes):
        self.write(_ENCODER_BASE, reg, cmd)

    def _encoder_reg(self, reg: int, encoder: int):
        return reg | (encoder & 0xF)

    @property
    def interrupt_enabled(self) -> bool:
        """Retrieve or set the interrupt enable flag"""
        return self._interrupt_enabled

    @interrupt_enabled.setter
    def interrupt_enabled(self, value: bool) -> None:
        if value not in (True, False):
            raise ValueError("interrupt_enabled must be True or False")

        self._interrupt_enabled = value
        for enc in range(self._num_encoders):
            cmd = self.packer.pack(0x1)
            if value:
                self._write_reg(self._encoder_reg(_ENCODER_INTENSET, enc), cmd)
            else:
                self._write_reg(self._encoder_reg(_ENCODER_INTENCLR, enc), cmd)

    def value(self, encoder: int) -> int:
        return self._read_reg(self._encoder_reg(_ENCODER_POSITION, encoder),
                              ResponseType.TYPE_VALUE)

    def set_value(self, encoder: int, value: int) -> None:
        cmd = self.packer.pack(value)
        self._write_reg(self._encoder_reg(_ENCODER_POSITION, encoder), cmd)

    def delta(self, encoder: int) -> int:
        return self._read_reg(self._encoder_reg(_ENCODER_DELTA, encoder),
                              ResponseType.TYPE_DELTA)

    def zero_delta(self, encoder: int) -> None:
        cmd = self.packer.pack(0)
        self._write_reg(self._encoder_reg(_ENCODER_DELTA, encoder), cmd)

    @property
    def count(self) -> int:
        """Retrieve or set the number of event"""
        try:
            self._tx_count += 1
            buf = self.readn(_ENCODER_BASE, _ENCODER_COUNT, 4)
            d = SeesawEncoderResponse.unpack(buf)
            if d.response_type != ResponseType.TYPE_COUNT:
                raise EncoderError(f'CORRUPTED {list([f"{x:x}" for x in buf])}')
            if d.data < 0:
                raise EncoderError(f'CORRUPTED {list([f"{x:x}" for x in buf])}')
        except OSError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            return 0
        except EncoderError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            return 0
        return d.data

    # pylint: disable=unused-argument, no-self-use
    @count.setter
    def count(self, value) -> None:
        raise AttributeError("count is read only")

    # pylint: enable=unused-argument, no-self-use

    def set_event(self, enc: int,
                  edge: int,  # EncoderEdge
                  enable: bool):
        """Control which kinds of events are set

           :param int enc: The encoder number
           :param int edge: The type of event
           :param bool enable: True to enable the event, False to disable it"""

        if enable not in (True, False):
            raise ValueError("event enable must be True or False")
        if edge > 5 or edge < 0:
            raise ValueError("invalid edge")

        cmd = self.packer.pack((enable << 16) | (1 << edge))
        self.write(_ENCODER_BASE, self._encoder_reg(_ENCODER_EVENT, enc), cmd)

    def read_encoders(self, num: int) -> List[SeesawEncoderResponse]:
        """Read data from the keypad

        :param int num: The number of bytes to read"""
        buf = bytearray(num * 4)
        try:
            self._tx_count += 1
            self.read(_ENCODER_BASE, _ENCODER_FIFO, buf)
            return [SeesawEncoderResponse.unpack_from(buf, i * 4)
                    for i in range(0, num)]
        except OSError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            return []
        except EncoderError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            return []

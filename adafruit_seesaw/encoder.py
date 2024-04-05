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
from typing import NamedTuple
from typing import ClassVar

try:
    from micropython import const
except ImportError:

    def const(x):
        return x


from adafruit_seesaw.seesaw import Seesaw

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_seesaw.git"

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


@dataclass
class SeesawEncoderResponse:
    response_type: ResponseType
    enc: int
    data: int

    def __post_init__(self):
        try:
            self.response_type = ResponseType(self.response_type)
        except Exception as e:
            # print(e)
            self.responseType = ResponseType.TYPE_INVALID

    unpacker: ClassVar[struct.Struct] = struct.Struct('<BBh')

    @classmethod
    def unpack(cls, buf: bytearray):
        return SeesawEncoderResponse(*cls.unpacker.unpack(buf))

    @classmethod
    def unpack_from(cls, buf: bytearray, frm: int):
        return SeesawEncoderResponse(*cls.unpacker.unpack_from(buf, frm))


# pylint: disable=too-few-public-methods
class EncoderEvent:
    """Holds information about a key event in its properties

       :param int num: The number of the key
       :param int edge: One of the EDGE propertes of `adafruit_seesaw.keypad.Keypad`
    """

    def __init__(self, num, edge):
        self.number = int(num)
        self.edge = int(edge)


# pylint: enable=too-few-public-methods


class Encoder(Seesaw):
    """On compatible SeeSaw devices, reads from an encoder.

       :param ~busio.I2C i2c_bus: Bus the SeeSaw is connected to
       :param int addr: I2C address of the SeeSaw device
       :param ~digitalio.DigitalInOut drdy: Pin connected to SeeSaw's 'ready' output"""

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

    packer: ClassVar[struct.Struct] = struct.Struct('>I')

    def __init__(self, i2c_bus, addr=0x49, drdy=None, num_encoders = _NUM_ENCODERS):
        super(Encoder, self).__init__(i2c_bus, addr, drdy)
        self._interrupt_enabled = False
        self._num_encoders = num_encoders

    @property
    def interrupt_enabled(self):
        """Retrieve or set the interrupt enable flag"""
        return self._interrupt_enabled

    @interrupt_enabled.setter
    def interrupt_enabled(self, value):
        if value not in (True, False):
            raise ValueError("interrupt_enabled must be True or False")

        self._interrupt_enabled = value
        for enc in range(self._num_encoders):
            if value:
                self.write8(_ENCODER_BASE, _ENCODER_INTENSET, enc | 0x01 << 4)
            else:
                self.write8(_ENCODER_BASE, _ENCODER_INTENCLR, enc | 0x01 << 4)

    @property
    def count(self):
        """Retrieve or set the number of event"""
        try:
            buf = self.readn(_ENCODER_BASE, _ENCODER_COUNT, 4)
            d = SeesawEncoderResponse.unpack(buf)
            if d.response_type != ResponseType.TYPE_COUNT:
                raise EncoderError("CORRUPTED %s" % list(["%x" % x for x in buf]))
                return 0
            if d.data < 0:
                raise EncoderError("CORRUPTED %s" % list(["%x" % x for x in buf]))
        except OSError as e:
            # print(e)
            return 0
        except EncoderError as e:
            # print(e)
            return 0
        return d.data

    # pylint: disable=unused-argument, no-self-use
    @count.setter
    def count(self, value):
        raise AttributeError("count is read only")

    # pylint: enable=unused-argument, no-self-use

    def set_event(self, enc, edge, enable):
        """Control which kinds of events are set

           :param int enc: The encoder number
           :param int edge: The type of event
           :param bool enable: True to enable the event, False to disable it"""

        if enable not in (True, False):
            raise ValueError("event enable must be True or False")
        if edge > 5 or edge < 0:
            raise ValueError("invalid edge")

        cmd = self.packer.pack((enable << 20) | (1 << (edge + 4)) | enc)
        self.write(_ENCODER_BASE, _ENCODER_EVENT, cmd)

    def read_encoders(self, num):
        """Read data from the keypad

        :param int num: The number of bytes to read"""
        buf = bytearray(num * 4)
        try:
            self.read(_ENCODER_BASE, _ENCODER_FIFO, buf)
            return [SeesawEncoderResponse.unpack_from(buf, i * 4)
                    for i in range(0, num)]
        except OSError as e:
            # print(e)
            return []
        except EncoderError as e:
            # print(e)
            return []

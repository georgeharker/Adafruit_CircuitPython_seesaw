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
`adafruit_seesaw.keypad`
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

_KEYPAD_BASE = const(0x10)

_KEYPAD_STATUS = const(0x00)
_KEYPAD_EVENT = const(0x01)
_KEYPAD_INTENSET = const(0x02)
_KEYPAD_INTENCLR = const(0x03)
_KEYPAD_COUNT = const(0x04)
_KEYPAD_FIFO = const(0x10)

DEFAULT_RD_DELAY = const(0.008)
DEFAULT_WR_DELAY = const(0.008)


class ResponseType(IntEnum):
    # Types for the repsonse
    TYPE_KEY = 0
    TYPE_COUNT = 1
    TYPE_STATUS = 2
    TYPE_INVALID = 0xff


class KeypadError(Exception):
    pass


class KeypadEdge(IntEnum):
    #: Indicates that the key is currently pressed
    EDGE_HIGH = 0
    #: Indicates that the key is currently released
    EDGE_LOW = 1
    #: Indicates that the key was recently pressed
    EDGE_FALLING = 2
    #: Indicates that the key was recently released
    EDGE_RISING = 3


@dataclass
class KeyEvent:
    """Holds information about a key event in its properties

       :param int num: The number of the key
       :param int edge: One of the EDGE propertes of `KeypadEdge`
    """
    number: int
    edge: KeypadEdge


@dataclass
class SeesawKeyResponse:
    response_type: ResponseType
    data: int

    unpacker: ClassVar[struct.Struct] = struct.Struct('<BB')

    def __post_init__(self):
        try:
            self.response_type = ResponseType(self.response_type)
        except Exception as e:  # noqa: F841
            # print(e)
            self.responseType = ResponseType.TYPE_INVALID

    @classmethod
    def unpack(cls, buf: bytearray):
        return SeesawKeyResponse(*cls.unpacker.unpack(buf))

    @classmethod
    def unpack_from(cls, buf: bytearray, frm: int):
        return SeesawKeyResponse(*cls.unpacker.unpack_from(buf, frm))

    def data_keyevent(self) -> KeyEvent:
        assert self.response_type == ResponseType.TYPE_KEY
        return KeyEvent((self.data >> 2) & 0x3f, KeypadEdge(self.data & 0x03))


class Keypad(Seesaw):
    """On compatible SeeSaw devices, reads from a keypad.

       :param ~busio.I2C i2c_bus: Bus the SeeSaw is connected to
       :param int addr: I2C address of the SeeSaw device
       :param ~digitalio.DigitalInOut drdy: Pin connected to SeeSaw's 'ready' output"""

    def __init__(self, i2c_bus, addr: int = 0x49, drdy=None):
        super().__init__(i2c_bus, addr, drdy,
                         rd_delay=DEFAULT_RD_DELAY,
                         wr_delay=DEFAULT_WR_DELAY)
        self._interrupt_enabled = False
        self._tx_errors = 0
        self._tx_count = 0

    @property
    def interrupt_enabled(self) -> bool:
        """Retrieve or set the interrupt enable flag"""
        return self._interrupt_enabled

    @interrupt_enabled.setter
    def interrupt_enabled(self, value: bool):
        if value not in (True, False):
            raise ValueError("interrupt_enabled must be True or False")

        self._interrupt_enabled = value
        if value:
            self.write8(_KEYPAD_BASE, _KEYPAD_INTENSET, 1)
        else:
            self.write8(_KEYPAD_BASE, _KEYPAD_INTENCLR, 1)

    @property
    def count(self) -> int:
        """Retrieve or set the number of keys"""
        try:
            self._tx_count += 1
            buf = self.readn(_KEYPAD_BASE, _KEYPAD_COUNT, 2)
            d = SeesawKeyResponse.unpack(buf)
            if d.response_type != ResponseType.TYPE_COUNT:
                raise KeypadError(f'CORRUPTED {list([f"{x:x}" for x in buf])}')
            if d.data < 0:
                raise KeypadError(f'CORRUPTED {list([f"{x:x}" for x in buf])}')
        except OSError as e:  # noqa: F841
            # print(e)
            self._tx_errors += 1
            return 0
        except KeypadError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            return 0
        return d.data

    # pylint: disable=unused-argument, no-self-use
    @count.setter
    def count(self, value):
        raise AttributeError("count is read only")

    # pylint: enable=unused-argument, no-self-use
    def set_event(self, key: int, edge: int, enable: bool) -> None:
        """Control which kinds of events are set

           :param int key: The key number
           :param int edge: The type of event
           :param bool enable: True to enable the event, False to disable it"""

        if enable not in (True, False):
            raise ValueError("event enable must be True or False")
        if edge > 3 or edge < 0:
            raise ValueError("invalid edge")

        cmd = bytearray(2)
        cmd[0] = key
        cmd[1] = (1 << (edge + 1)) | enable

        self.write(_KEYPAD_BASE, _KEYPAD_EVENT, cmd)

    def read_keypad(self, num=None) -> List[SeesawKeyResponse]:
        """Read data from the keypad
        :param int num: The number of bytes to read"""
        if num is None:
            num = self.count
        buf = bytearray(num * 2)
        try:
            self._tx_count += 1
            self.read(_KEYPAD_BASE, _KEYPAD_FIFO, buf)

            return [SeesawKeyResponse.unpack_from(buf, i * 2)
                    for i in range(0, num)]
        except OSError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            return []
        except KeypadError as e:  # noqa: F841
            self._tx_errors += 1
            # print(e)
            return []

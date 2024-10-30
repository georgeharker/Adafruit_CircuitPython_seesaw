# The MIT License (MIT)
#
# Copyright (c) 2017 Dean Miller for Adafruit Industries
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
`adafruit_seesaw.neopixel`
====================================================
"""

from typing import Optional, Sequence, Tuple, Union, cast

from .seesaw import Seesaw


try:
    import struct
except ImportError:
    import ustruct as struct  # type: ignore
try:
    from micropython import const
except ImportError:

    def const(x):
        return x


__version__ = "1.2.3"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_seesaw.git"

_NEOPIXEL_BASE = const(0x0E)

_NEOPIXEL_STATUS = const(0x00)
_NEOPIXEL_PIN = const(0x01)
_NEOPIXEL_SPEED = const(0x02)
_NEOPIXEL_BUF_LENGTH = const(0x03)
_NEOPIXEL_BUF = const(0x04)
_NEOPIXEL_SHOW = const(0x05)
_NEOPIXEL_PARTIAL_BUF = const(0x06)

# Pixel color order constants
RGB = (0, 1, 2)
"""Red Green Blue"""
GRB = (1, 0, 2)
"""Green Red Blue"""
RGBW = (0, 1, 2, 3)
"""Red Green Blue White"""
GRBW = (1, 0, 2, 3)
"""Green Red Blue White"""


PixelType3 = Tuple[int, int, int]
PixelType4 = Tuple[int, int, int, int]
PixelType = Union[PixelType3, PixelType4]

ColorType3 = Tuple[float, float, float]
ColorType4 = Tuple[float, float, float, float]
ColorType = Union[ColorType3, ColorType4, int]

class NeoPixel:
    """Control NeoPixels connected to a seesaw

    :param ~adafruit_seesaw.seesaw.Seesaw seesaw: The device
    :param int pin: The pin number on the device
    :param int n: The number of pixels
    :param int bpp: The number of bytes per pixel
    :param float brightness: The brightness, from 0.0 to 1.0
    :param bool auto_write: Automatically update the pixels when changed
    :param tuple pixel_order: The layout of the pixels.
        Use one of the order constants such as RGBW.
"""
    _seesaw: Seesaw
    _pin: int
    _bpp: int
    _wr_delay: float
    auto_write: bool
    _n: int
    _brightness: float
    _pixel_order: PixelType

    def __init__(
        self,
        seesaw: Seesaw,
        pin: int,
        n: int,
        *,
        bpp: int = 3,
        brightness: float = 1.0,
        auto_write: bool = False,
        pixel_order: Optional[PixelType] = None,
        wr_delay: float = 0.0001
    ):
        self._seesaw = seesaw
        self._pin = pin
        self._bpp = bpp
        self._wr_delay = wr_delay
        self.auto_write = auto_write
        self._n = n
        self._brightness = min(max(brightness, 0.0), 1.0)
        self._pixel_order = GRBW if pixel_order is None else pixel_order
        assert len(self._pixel_order) > self._bpp

        cmd = bytearray([pin])
        self._seesaw.write(_NEOPIXEL_BASE, _NEOPIXEL_PIN, cmd)
        cmd2 = struct.pack(">H", n * self._bpp)
        self._seesaw.write(_NEOPIXEL_BASE, _NEOPIXEL_BUF_LENGTH, cmd2)

    @property
    def brightness(self) -> float:
        """Overall brightness of the pixel"""
        return self._brightness

    @brightness.setter
    def brightness(self, brightness: float):
        # pylint: disable=attribute-defined-outside-init
        self._brightness = min(max(brightness, 0.0), 1.0)
        if self.auto_write:
            self.show()

    def deinit(self):
        pass

    def __len__(self):
        return self._n

    def _color_to_components(self, color: ColorType) -> Tuple[int, int, int, int]:
        r = g = b = w = 0.0
        if isinstance(color, int):
            w = color >> 24
            r = (color >> 16) & 0xff
            g = (color >> 8) & 0xff
            b = color & 0xff
        else:
            if self._bpp == 3:
                r, g, b = cast(ColorType3, color)
            else:
                r, g, b, w = cast(ColorType4, color)

        # if all components are the same and we have a white pixel then use it
        # instead of the individual components.
        if self._bpp == 4 and r == g == b and w == 0:
            w = r
            r = 0
            g = 0
            b = 0

        if self.brightness < 0.99:
            r = int(r * self.brightness)
            g = int(g * self.brightness)
            b = int(b * self.brightness)
            if self._bpp == 4:
                w = int(w * self.brightness)
        return cast(int, r), cast(int, g), cast(int, b), cast(int, w)

    def setbuf(self, colors: Sequence[ColorType]) -> None:
        """Set all pixels to a new value"""
        cmd = bytearray(2 + self._bpp * len(colors))
        struct.pack_into(">H", cmd, 0, 0)
        i = 0
        for color in colors:
            r, g, b, w = self._color_to_components(color)

            # store colors in correct slots
            cmd[2 + self._pixel_order[0] + i * self._bpp] = r
            cmd[2 + self._pixel_order[1] + i * self._bpp] = g
            cmd[2 + self._pixel_order[2] + i * self._bpp] = b
            if self._bpp == 4:
                cmd[2 + cast(PixelType4, self._pixel_order)[3] + i * self._bpp] = w
            i += 1

        self._seesaw.write(_NEOPIXEL_BASE, _NEOPIXEL_BUF, cmd, delay=self._wr_delay)
        if self.auto_write:
            self.show()

    def update(self, updates: Sequence[Tuple[int, ColorType]]) -> None:
        """Set all pixels to a new value"""
        stride = self._bpp + 1
        cmd = bytearray(1 + stride * len(updates))
        cmd[0] = self._bpp
        i = 0
        for key, color in updates:
            r, g, b, w = self._color_to_components(color)

            # store colors in correct slots
            cmd[1 + i * stride] = key
            cmd[2 + self._pixel_order[0] + i * stride] = r
            cmd[2 + self._pixel_order[1] + i * stride] = g
            cmd[2 + self._pixel_order[2] + i * stride] = b
            if self._bpp == 4:
                cmd[2 + cast(PixelType4, self._pixel_order)[3] + i * stride] = w
            i += 1

        self._seesaw.write(_NEOPIXEL_BASE, _NEOPIXEL_PARTIAL_BUF, cmd, delay=self._wr_delay)
        if self.auto_write:
            self.show()

    def __setitem__(self, key: int, color: ColorType):
        """Set one pixel to a new value"""
        cmd = bytearray(2 + self._bpp)
        struct.pack_into(">H", cmd, 0, key * self._bpp)

        r, g, b, w = self._color_to_components(color)

        # Store colors in correct slots
        cmd[2 + self._pixel_order[0]] = r
        cmd[2 + self._pixel_order[1]] = g
        cmd[2 + self._pixel_order[2]] = b
        if self._bpp == 4:
            cmd[2 + cast(PixelType4, self._pixel_order)[3]] = w

        self._seesaw.write(_NEOPIXEL_BASE, _NEOPIXEL_BUF, cmd, delay=self._wr_delay)
        if self.auto_write:
            self.show()

    def __getitem__(self, key):
        pass

    def fill(self, color) -> None:
        """Set all pixels to the same value"""
        # Suppress auto_write while filling.
        current_auto_write = self.auto_write
        self.auto_write = False
        for i in range(self._n):
            self[i] = color
        if current_auto_write:
            self.show()
        self.auto_write = current_auto_write

    def show(self) -> None:
        """Update the pixels even if auto_write is False"""
        self._seesaw.write(_NEOPIXEL_BASE, _NEOPIXEL_SHOW, delay=self._wr_delay)

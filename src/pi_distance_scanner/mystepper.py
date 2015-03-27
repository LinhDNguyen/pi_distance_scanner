# Python control of stepper motors.

# Copyright (C) 2008-2011  W. Trevor King
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import logging as _logging
import logging.handlers as _logging_handlers
from time import sleep as _sleep


__version__ = '0.4'


LOG = _logging.getLogger('stepper')
"Stepper logger"

LOG.setLevel(_logging.WARN)
_formatter = _logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s')

_stream_handler = _logging.StreamHandler()
_stream_handler.setLevel(_logging.DEBUG)
_stream_handler.setFormatter(_formatter)
LOG.addHandler(_stream_handler)


def _binary(i, width=4):
    """Convert `i` to a binary string of width `width`.

    >>> _binary(0)
    '0000'
    >>> _binary(1)
    '0001'
    """
    str = bin(i)[2:]
    return '0'*(width-len(str)) + str


class Stepper (object):
    """Stepper motor control

    inputs:

    write      (fn(value)) write the 4-bit integer `value` to
               appropriate digital output channels.
    full_step  (boolean) select full or half stepping
    logic      (boolean) select active high (True) or active low (False)
    delay      (float) time delay between steps in seconds, in case the
                motor response is slower than the digital output
                driver.
    step_size  (float) approximate step size in meters
    backlash   (int) generous estimate of backlash in half-steps

    >>> from pycomedi.device import Device
    >>> from pycomedi.channel import DigitalChannel
    >>> from pycomedi.constant import SUBDEVICE_TYPE, IO_DIRECTION

    >>> device = Device('/dev/comedi0')
    >>> device.open()

    >>> subdevice = device.find_subdevice_by_type(SUBDEVICE_TYPE.dio)
    >>> channels = [subdevice.channel(i, factory=DigitalChannel)
    ...             for i in (0, 1, 2, 3)]
    >>> for chan in channels:
    ...     chan.dio_config(IO_DIRECTION.output)

    >>> def write(value):
    ...     subdevice.dio_bitfield(bits=value, write_mask=2**4-1)

    >>> s = Stepper(write=write)
    >>> s.position
    0
    >>> s.single_step(1)
    >>> s.position
    2
    >>> s.single_step(1)
    >>> s.position
    4
    >>> s.single_step(1)
    >>> s.position
    6
    >>> s.single_step(-1)
    >>> s.position
    4
    >>> s.single_step(-1)
    >>> s.position
    2
    >>> s.single_step(-1)
    >>> s.position
    0
    >>> s.full_step = False
    >>> s.single_step(-1)
    >>> s.position
    -1
    >>> s.single_step(-1)
    >>> s.position
    -2
    >>> s.single_step(-1)
    >>> s.position
    -3
    >>> s.single_step(1)
    >>> s.position
    -2
    >>> s.single_step(1)
    >>> s.position
    -1
    >>> s.single_step(1)
    >>> s.position
    0
    >>> s.step_to(1000)
    >>> s.position
    1000
    >>> s.step_to(-1000)
    >>> s.position
    -1000
    >>> s.step_relative(1000)
    >>> s.position
    0

    >>> device.close()
    """
    def __init__(self, write, full_step=True, logic=True, delay=1e-2,
                 step_size=170e-9, backlash=100):
        self._write = write
        self.full_step = full_step
        self.logic = logic
        self.delay = delay
        self.step_size = step_size
        self.backlash = backlash
        self.port_values = [1,  # binary ---1  setup for logic == True
                            3,  # binary --11
                            2,  # binary --1-
                            6,  # binary -11-
                            4,  # binary -1--
                            12, # binary 11--
                            8,  # binary 1---
                            9]  # binary 1--1
        self._set_position(0)

    def _get_output(self, position):
        """Get the port value that places the stepper in `position`.

        >>> s = Stepper(write=lambda value: value, logic=True)
        >>> _binary(s._get_output(0))
        '0001'
        >>> _binary(s._get_output(1))
        '0101'
        >>> _binary(s._get_output(2))
        '0100'
        >>> _binary(s._get_output(-79))
        '0101'
        >>> _binary(s._get_output(81))
        '0101'
        >>> s.logic = False
        >>> _binary(s._get_output(0))
        '1110'
        >>> _binary(s._get_output(1))
        '1010'
        >>> _binary(s._get_output(2))
        '1011'
        """
        value = self.port_values[position % len(self.port_values)]
        if not self.logic:
            value = 2**4 - 1 - value
        return value

    def _set_position(self, position):
        self.position = position  # current stepper index in half steps
        output = self._get_output(position)
        LOG.debug('set postition to %d (%s)' % (position, _binary(output)))
        self._write(output)

    def single_step(self, direction):
        LOG.debug('single step')
        if self.full_step and self.position % 2 == 1:
            self.position -= 1  # round down to a full step
        if direction > 0:
            step = 1
        elif direction < 0:
            step = -1
        else:
            raise ValueError(direction)  # no step
        if self.full_step:
            step *= 2
        self._set_position(self.position + step)
        if self.delay > 0:
            _sleep(self.delay)

    def step_to(self, target_position):
        if target_position != int(target_position):
            raise ValueError(
                'target_position %s must be an int' % target_position)
        if self.full_step and target_position % 2 == 1:
            target_position -= 1  # round down to a full step
        if target_position > self.position:
            direction = 1
        else:
            direction = -1
        while self.position != target_position:
            LOG.debug('stepping %s -> %s (%s)' % (target_position, self.position, direction))
            self.single_step(direction)

    def step_relative(self, relative_target_position, backlash_safe=False):
        """Step relative to the current position.

        If `backlash_safe` is `True` and `relative_target_position` is
        negative, step back an additional `.backlash` half-steps and
        then come back to the target position.  This takes the slack
        out of the drive chain and ensures that you actually do move
        back to the target location.  Note that as the drive chain
        loosens up after the motion completes, the stepper position
        will creep forward again.
        """
        target = self.position + relative_target_position
        if backlash_safe and relative_target_position < 0:
            self.step_to(target - self.backlash)
        self.step_to(target)

"""
This module controls Cobolt laser. All models in 04-01 series

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

from core.module import Base, ConfigOption
from interface.simple_laser_interface import SimpleLaserInterface
from interface.simple_laser_llkkinterface import LaserState
from interface.simple_laser_interface import ShutterState

import serial
import time
from enum import Enum

# Define an enumeration for control modes
class ControlMode(Enum):
    POWER = 0
    CURRENT = 1

class ShutterState(Enum):
    CLOSED = 0
    OPEN = 1
    UNKNOWN = 2
    NOSHUTTER = 3

class LaserState(Enum):
    OFF = 0
    ON = 1
    # LOCKED = 2
    # UNKNOWN = 3


class CoboltLaser(Base, SimpleLaserInterface):
    _modtype = 'laser'
    _modclass = 'hardware'

    _termination = '\r\n'
    _model_name = 'UNKNOWN'  #Mambo or Calypso
    #_model_name = 'mambo/calypso'

    _com_port = ConfigOption('com_port', missing='error')


    def on_activate(self):
        """
        Activate the module
        """
        self.cobolt = serial.Serial(self._com_port, timeout=1)

        connected = self.connect_laser()

        if not connected:
            self.log.error('Laser is not connected')
            return -1
        else:
            # self._model_name = self._communicate('SYST:INF:MOD?') #no such command for cobolt
            return 0


    def on_deactivate(self):
        """
        Deactivate the module
        """

        self.disconnect_laser()

    def connect_laser(self):
        """ Connect to Instrument.

        @return bool: connection success
        """
        #command: l? checks if the laser is ON or OFF
        #command returns: 0=OFF, 1=ON
        #The command isn't really important, we send any command to check
        #if the laser responds (connected or not)

        response = self._communicate('l?')[0]

        # if response.startswith('ERR-100'):
        if 'error' in response.lower():
            return False
        else:
            return True

    def disconnect_laser(self):
        """ Close the connection to the instrument.
                """
        self.off()
        self.cobolt.close()


        ########################## communication methods ###############################

    def _send(self, message):
        """ Send a message to the laser

        @param string message: message to be delivered to the laser
        """
        new_message = message + self._termination
        self.cobolt.write(new_message.encode('ascii'))

    def _communicate(self, message):
        """ Send and receive messages to and from the laser

        @param string message: message to be delivered to the laser

        @returns string response: message received from the laser
        """
        self._send(message)
        time.sleep(0.1)
        response_len = self.cobolt.inWaiting()
        response = []

        while response_len > 0:
            this_response_line = self.cobolt.readline().decode('ascii')
            if (response_len == 4) and ((this_response_line == 'OK') or (this_response_line == "OK")):
                response.append('')
            else:
                response.append(this_response_line)
            response_len = self.cobolt.inWaiting()

        # Potentially multi-line responses - need to be joined into string
        full_response = ''.join(response)

        # if full_response == 'ERR-100':
        if 'error' in full_response.lower():
            self.log.warning(self._model_name + ' does not support the command ' + message)
            return '-1'

        return full_response

    ########################## internal methods ####################################

    def _get_diode_temperature(self):
        """ Get laser diode temperature

        @return float: laser diode temperature
        """
        # response = float(self._communicate('SOUR:TEMP:DIOD?').split('C')[0])
        # return response
        pass

    def _get_internal_temperature(self):
        """ Get internal laser temperature

        @return float: internal laser temperature
        """
        # return float(self._communicate('SOUR:TEMP:INT?').split('C')[0])
        pass

    def _get_baseplate_temperature(self):
        """ Get laser base plate temperature

        @return float: laser base plate temperature
        """
        pass

    def _get_interlock_status(self):
        """ Get the status of the system interlock

        @returns bool interlock: status of the interlock
        """
        #ilk?: get interlock state command
        response = self._communicate('ilk?')

        if response.lower() == '0':
            return True
        elif response.lower() == '1':
            return False
        else:
            return False

    def _set_laser_to_11(self):
        """ Set the laser power to 11
        """
        # self.set_power(0.165)
        pass

    ####################
    #Below are the SimpleLaserInterface functions
    ####################
    def get_power_range(self):
        """ Return laser power
        Power range for cobolt calypso/mambo models
        @return tuple(p1, p2): Laser power range in watts
        """
        # for mambo the high power value is 100 mW,
        # but 75 mW has been chosen if no model is specified
        if self._model_name == 'mambo':
            power_low = 50e-3
            power_high = 100e-3
        else:
            power_low = 50e-3
            power_high = 75e-3

        return (power_low,power_high)

    def get_power(self):
        """ Return laser power
        @return float: Actual laser power in watts
        """
        response = self._communicate('pa?')

        return float(response)

    def set_power(self, power):
        """ Set laer power ins watts
          @param float power: laser power setpoint in watts
        """
        self._communicate('p {}'.format(power))

    def get_power_setpoint(self):
        """ Return laser power setpoint
        @return float: Laser power setpoint in watts
        """
        response = self._communicate('p?')

        return float(response)

    def get_current_unit(self):
        """ Return laser current unit
        @return str: unit
        """
        # Ampere
        return 'A'

    def get_current(self):
        """ Return laser current
        @return float: actual laser current as ampere or percentage of maximum current
        """
        response = self._communicate('i?')

        return float(response)

    def get_current_range(self):
        """ Return laser current range
        @return tuple(c1, c2): Laser current range in current units
        """
        self.log.warning('No current range could be found')
        return -1

    def get_current_setpoint(self):
        """ Return laser current
        @return float: Laser current setpoint in amperes
        """
        self.log.warning('No current setpoint could be found')
        return -1

    def set_current(self, current):
        """ Set laser current
        @param float current: Laser current value in amperes
        """
        self._communicate('slc {}'.format(current))

    def allowed_control_modes(self):
        """ Get available control mode of laser
          @return list: list with enum control modes
        """
        return [ControlMode.POWER, ControlMode.CURRENT]

    def get_control_mode(self):
        """ Get control mode of laser
          @return enum ControlMode: control mode
        """
        self.log.warning('No available command to get the control mode')

    def set_control_mode(self, control_mode):
        """ Set laser control mode.
          @param enum control_mode: desired control mode
        """
        control_mode_str = str(control_mode)
        if 'power' in control_mode_str.lower():
            self._communicate('cp')
        elif 'current' in control_mode_str.lower():
            self._communicate('ci')
        else:
            self.log.error(self._model_name + 'has no such mode')
            return -1

    def on(self):
        """ Turn on laser. Does not open shutter if one is present.
          @return enum LaserState: actual laser state
        """
        status = self.get_laser_state()
        if status == LaserState.OFF:
            self._communicate('@cob1')
            return self.get_laser_state()
        else:
            return self.get_laser_state()

    def off(self):
        """ Turn ooff laser. Does not close shutter if one is present.
          @return enum LaserState: actual laser state
        """
        status = self.get_laser_state()
        if status == LaserState.ON:
            self._communicate('l0')
            return self.get_laser_state()
        else:
            return self.get_laser_state()
        return LaserState.OFF

    def get_laser_state(self):
        """ Get laser state.
          @return enum LaserState: laser state
        """
        response = int(self._communicate('l?'))
        return LaserState(response)

    def set_laser_state(self, state):
        """ Set laser state.
          @param enum state: desired laser state
          @return enum LaserState: actual laser state
        """
        if state == self.get_laser_state():
            return self.get_laser_state()
        else:
            if state == LaserState.OFF:
                self.off()
                return self.get_laser_state()
            elif state == LaserState.ON:
                self.on()
                return self.get_laser_state()
            else:
                self.log.error('No such state is available')
                return -1

    def get_shutter_state(self):
        """ Get shutter state. Has a state for no shutter present.
          @return enum ShutterState: actual shutter state
        """
        self.log.info('Cobolt-04-01 series have manual shutters')
        return ShutterState.UNKNOWN

    def set_shutter_state(self, state):
        """ Set shutter state.
          @param enum state: desired shutter state
          @return enum ShutterState: actual shutter state
        """
        self.log.warning('The shutter is manual')
        return self.get_shutter_state()

    def get_temperatures(self):
        """ Get all available temperatures from laser.
          @return dict: dict of name, value for temperatures
        """
        self.log.warning('No available command to get the temperature')
        return {}

    def get_temperature_setpoints(self):
        """ Get all available temperature setpoints from laser.
          @return dict: dict of name, value for temperature setpoints
        """
        self.log.warning('No available command to get the temperature setpoints')
        return {}

    def set_temperatures(self, temps):
        """ Set laser temperatures.
          @param temps: dict of name, value to be set
          @return dict: dict of name, value of temperatures that were set
        """
        self.log.warning('No available command to set the temperature')
        return {}

    def get_extra_info(self):
        """ Show diagnostic information about lasers.
          @return str: diagnostic info as a string
          operating fault:0 = no fault
                          1 = temperature error
                          3 = open interlock
                          4 = constant power fault
          serial number:  serial number
          Operating hours: Operation hours
        """
        operating_fault = self._communicate('f?')
        serial_number = self._communicate('sn?')
        operating_hours = self._communicate('hrs?')
        diagnostic_str = f"(Operating fault: {operating_fault}), (serial number: {serial_number}), (operating hours: {operating_hours})"
        return diagnostic_str

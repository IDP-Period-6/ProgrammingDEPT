# *-------------------------------------------------------------------------*/
# *                                                                         */
# *    Copyright (c) Innovation First 2019-2024, All rights reserved.       */
# *                                                                         */
# *    Module:     vex.py                                                   */
# *    Author:     James Pearman                                            */
# *    Created:    30 December 2019                                         */
# *                                                                         */
# *    Revisions:                                                           */
# *                V1.00     TBD - Initial release                          */
# *                                                                         */
# *-------------------------------------------------------------------------*/
#
# classes used for simulation
# allows debugging of python code
# version: 20240522_1800_00
#
# ----------------------------------------------------------
# pylint: disable=unnecessary-pass,unused-argument,line-too-long,too-many-lines
# pylint: disable=missing-module-docstring,missing-function-docstring,missing-class-docstring
# pylint: disable=invalid-name,unused-import,redefined-outer-name

import time
import math
import sys
from typing import Union
from typing import Any
from typing import Tuple
from typing import Callable
from typing import List

# ----------------------------------------------------------
# V5 brain port definition
#

class Ports:
    '''Smartport definitions'''
    PORT1 = 0
    PORT2 = 1
    PORT3 = 2
    PORT4 = 3
    PORT5 = 4
    PORT6 = 5
    PORT7 = 6
    PORT8 = 7
    PORT9 = 8
    PORT10 = 9
    PORT11 = 10
    PORT12 = 11
    PORT13 = 12
    PORT14 = 13
    PORT15 = 14
    PORT16 = 15
    PORT17 = 16
    PORT18 = 17
    PORT19 = 18
    PORT20 = 19
    PORT21 = 20
    PORT22 = 21

# ----------------------------------------------------------
# base class used for all enumerated types
# not instantiated directly
#

class vexEnum:
    '''Base class for all enumerated types'''
    value = 0
    name = ""

    def __init__(self, value, name):
        self.value = value
        self.name = name

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

# ----------------------------------------------------------

class PercentUnits:
    '''The measurement units for percentage values.'''
    class PercentUnits(vexEnum):
        pass
    PERCENT = PercentUnits(0, "PERCENT")
    '''A percentage unit that represents a value from 0% to 100%'''

# ----------------------------------------------------------

class TimeUnits:
    '''The measurement units for time values.'''
    class TimeUnits(vexEnum):
        pass
    SECONDS = TimeUnits(0, "SECONDS")
    '''A time unit that is measured in seconds.'''
    SEC = TimeUnits(0, "SECONDS")
    '''A time unit that is measured in seconds.'''
    MSEC = TimeUnits(1, "MSEC")
    '''A time unit that is measured in milliseconds.'''

# ----------------------------------------------------------

class CurrentUnits:
    '''The measurement units for current values.'''
    class CurrentUnits(vexEnum):
        pass
    AMP = CurrentUnits(0, "AMP")
    '''A current unit that is measured in amps.'''

# ----------------------------------------------------------

class VoltageUnits:
    '''The measurement units for voltage values.'''
    class VoltageUnits(vexEnum):
        pass
    VOLT = VoltageUnits(0, "VOLT")
    '''A voltage unit that is measured in volts.'''
    MV = VoltageUnits(0, "mV")
    '''A voltage unit that is measured in millivolts.'''

# ----------------------------------------------------------

class PowerUnits:
    '''The measurement units for power values.'''
    class PowerUnits(vexEnum):
        pass
    WATT = PowerUnits(0, "WATT")
    '''A power unit that is measured in watts.'''

# ----------------------------------------------------------

class TorqueUnits:
    '''The measurement units for torque values.'''
    class TorqueUnits(vexEnum):
        pass
    NM = TorqueUnits(0, "NM")
    '''A torque unit that is measured in Newton Meters.'''
    INLB = TorqueUnits(1, "INLB")
    '''A torque unit that is measured in Inch Pounds.'''

# ----------------------------------------------------------

class RotationUnits:
    '''The measurement units for rotation values.'''
    class RotationUnits(vexEnum):
        pass
    DEG = RotationUnits(0, "DEG")
    '''A rotation unit that is measured in degrees.'''
    REV = RotationUnits(1, "REV")
    '''A rotation unit that is measured in revolutions.'''
    RAW = RotationUnits(99, "RAW")
    '''A rotation unit that is measured in raw data form.'''

# ----------------------------------------------------------

class VelocityUnits:
    '''The measurement units for velocity values.'''
    class VelocityUnits(vexEnum):
        pass
    PERCENT = VelocityUnits(0, "PCT")
    '''A velocity unit that is measured in percentage.'''
    RPM = VelocityUnits(1, "RPM")
    '''A velocity unit that is measured in rotations per minute.'''
    DPS = VelocityUnits(2, "DPS")
    '''A velocity unit that is measured in degrees per second.'''

# ----------------------------------------------------------

class DistanceUnits:
    '''The measurement units for distance values.'''
    class DistanceUnits(vexEnum):
        pass
    MM = DistanceUnits(0, "MM")
    '''A distance unit that is measured in millimeters.'''
    IN = DistanceUnits(1, "IN")
    '''A distance unit that is measured in inches.'''
    CM = DistanceUnits(2, "CM")
    '''A distance unit that is measured in centimeters.'''

# ----------------------------------------------------------

class AnalogUnits:
    '''The measurement units for analog values.'''
    class AnalogUnits(vexEnum):
        pass
    PCT = AnalogUnits(0, "PCT")
    '''An analog unit that is measured in percentage.'''
    EIGHTBIT = AnalogUnits(0, "8BIT")
    '''An analog unit that is measured in an 8-bit analog value
     (a value with 256 possible states).'''
    TENBIT = AnalogUnits(0, "10BIT")
    '''An analog unit that is measured in an 10-bit analog value
     (a value with 1024 possible states).'''
    TWELVEBIT = AnalogUnits(0, "12BIT")
    '''An analog unit that is measured in an 12-bit analog value
     (a value with 4096 possible states).'''
    MV = AnalogUnits(0, "MV")
    '''An analog unit that is measured in millivolts.'''

# ----------------------------------------------------------

class TemperatureUnits:
    '''The measurement units for temperature values.'''
    class TemperatureUnits(vexEnum):
        pass
    CELSIUS = TemperatureUnits(0, "CELSIUS")
    '''A temperature unit that is measured in celsius.'''
    FAHRENHEIT = TemperatureUnits(0, "FAHRENHEIT")
    '''A temperature unit that is measured in fahrenheit.'''

# ----------------------------------------------------------

class DirectionType:
    '''The defined units for direction values.'''
    class DirectionType(vexEnum):
        pass
    FORWARD = DirectionType(0, "FORWARD")
    '''A direction unit that is defined as forward.'''
    REVERSE = DirectionType(1, "REVERSE")
    '''A direction unit that is defined as backward.'''
    UNDEFINED = DirectionType(2, "UNDEFINED")
    '''A direction unit used when direction is not known.'''

# ----------------------------------------------------------

class TurnType(vexEnum):
    '''The defined units for turn values.'''
    class TurnType(vexEnum):
        pass
    LEFT = TurnType(0, "LEFT")
    '''A turn unit that is defined as left turning.'''
    RIGHT = TurnType(1, "RIGHT")
    '''A turn unit that is defined as right turning.'''
    UNDEFINED = TurnType(2, "UNDEFINED")
    '''A turn unit unit used when direction is not known.'''

# ----------------------------------------------------------

class BrakeType:
    '''The defined units for motor brake values.'''
    class BrakeType(vexEnum):
        pass
    COAST = BrakeType(0, "COAST")
    '''A brake unit that is defined as motor coast.'''
    BRAKE = BrakeType(1, "BRAKE")
    '''A brake unit that is defined as motor brake.'''
    HOLD = BrakeType(2, "HOLD")
    '''A brake unit that is defined as motor hold.'''

# ----------------------------------------------------------

class GearSetting:
    '''The defined units for gear values.'''
    class GearSetting(vexEnum):
        pass
    RATIO_36_1 = GearSetting(0, "RATIO36_1")
    '''A gear unit that is defined as the red 36:1 gear cartridge used in
     V5 Smart Motors.'''
    RATIO_18_1 = GearSetting(1, "RATIO18_1")
    '''A gear unit that is defined as the green 18:1 gear cartridge used in
     V5 Smart Motors.'''
    RATIO_6_1 = GearSetting(2, "RATIO6_1")
    '''A gear unit that is defined as the blue 6:1 gear cartridge used in
     V5 Smart Motors.'''

# ----------------------------------------------------------

class FontType:
    '''A unit representing font type and size'''
    class FontType(vexEnum):
        pass
    MONO20 = FontType(0, "MONO20")
    '''monotype font of size 20'''
    MONO30 = FontType(1, "MONO30")
    '''monotype font of size 30'''
    MONO40 = FontType(2, "MONO40")
    '''monotype font of size 40'''
    MONO60 = FontType(3, "MONO60")
    '''monotype font of size 60'''
    PROP20 = FontType(4, "PROP20")
    '''proportional font of size 20'''
    PROP30 = FontType(5, "PROP30")
    '''proportional font of size 30'''
    PROP40 = FontType(6, "PROP40")
    '''proportional font of size 40'''
    PROP60 = FontType(7, "PROP60")
    '''proportional font of size 60'''
    MONO15 = FontType(8, "MONO15")
    '''proportional font of size 15'''
    MONO12 = FontType(9, "MONO12")
    '''proportional font of size 12'''
    CJK16 = FontType(10, "CJK16")
    '''Chinese/Japanese/Korean font of size 16'''

# ----------------------------------------------------------

class ThreeWireType:
    '''The defined units for 3-wire devices.'''
    class ThreeWireType(vexEnum):
        pass
    ANALOG_IN = ThreeWireType(0, "ANALOG_IN")
    '''A 3-wire sensor that is defined as an analog input.'''
    ANALOG_OUT = ThreeWireType(1, "ANALOG_OUT")
    '''A 3-wire sensor that is defined as an analog output.'''
    DIGITAL_IN = ThreeWireType(2, "DIGITAL_IN")
    '''A 3-wire sensor that is defined as an digital input.'''
    DIGITAL_OUT = ThreeWireType(3, "DIGITAL_OUT")
    '''A 3-wire sensor that is defined as an digital output.'''
    SWITCH = ThreeWireType(4, "BUTTON")
    '''A 3-wire sensor that is defined as a switch.'''
    POTENTIOMETER = ThreeWireType(5, "POT")
    '''A 3-wire sensor that is defined as a potentiometer.'''
    LINE_SENSOR = ThreeWireType(6, "LINE_SENSOR")
    '''A 3-wire sensor that is defined as a line sensor.'''
    LIGHT_SENSOR = ThreeWireType(7, "LIGHT_SENSOR")
    '''A 3-wire sensor that is defined as a light sensor.'''
    GYRO = ThreeWireType(8, "GYRO")
    '''A 3-wire sensor that is defined as a yaw rate gyro.'''
    ACCELEROMETER = ThreeWireType(9, "ACCELEROMETER")
    '''A 3-wire sensor that is defined as a accelerometer.'''
    MOTOR = ThreeWireType(10, "MOTOR")
    '''A 3-wire sensor that is defined as a legacy vex motor.'''
    SERVO = ThreeWireType(11, "SERVO")
    '''A 3-wire sensor that is defined as a legacy vex servo.'''
    ENCODER = ThreeWireType(12, "ENCODER")
    '''A 3-wire sensor that is defined as a quadrature encoder.'''
    SONAR = ThreeWireType(13, "SONAR")
    '''A 3-wire sensor that is defined as an ultrasonic sensor (sonar)'''
    SLEW_MOTOR = ThreeWireType(14, "SLEW_MOTOR")
    '''A 3-wire sensor that is defined as a legacy vex motor using slew rate control.'''

# ----------------------------------------------------------

class ControllerType:
    '''The defined types for controller devices.'''
    class ControllerType(vexEnum):
        pass
    PRIMARY = ControllerType(0, "PRIMARY")
    '''A controller defined as a primary controller.'''
    PARTNER = ControllerType(1, "PARTNER")
    '''A controller defined as a partner controller.'''

# ----------------------------------------------------------

class AxisType:
    '''The defined units for inertial sensor axis.'''
    class AxisType(vexEnum):
        pass
    XAXIS = AxisType(0, "XAXIS")
    '''The X axis of the Inertial sensor.'''
    YAXIS = AxisType(1, "YAXIS")
    '''The Y axis of the Inertial sensor.'''
    ZAXIS = AxisType(2, "ZAXIS")
    '''The Z axis of the Inertial sensor.'''

# ----------------------------------------------------------

class OrientationType:
    '''The defined units for inertial sensor orientation.'''
    class OrientationType(vexEnum):
        pass
    ROLL = OrientationType(0, "ROLL")
    '''roll, orientation around the X axis of the Inertial sensor.'''
    PITCH = OrientationType(1, "PITCH")
    '''pitch, orientation around the Y axis of the Inertial sensor.'''
    YAW = OrientationType(2, "YAW")
    '''yaw, orientation around the Z axis of the Inertial sensor.'''

# ----------------------------------------------------------

class ObjectSizeType:
    '''The defined units for distance sensor object size.'''
    class ObjectSizeType(vexEnum):
        pass
    NONE = ObjectSizeType(0, "NONE")
    SMALL = ObjectSizeType(1, "SMALL")
    MEDIUM = ObjectSizeType(2, "MEDIUM")
    LARGE = ObjectSizeType(3, "LARGE")

# ----------------------------------------------------------

class LedStateType:
    '''The defined units for optical sensor led state.'''
    class LedStateType(vexEnum):
        pass
    OFF = LedStateType(0, "OFF")
    ON = LedStateType(1, "ON")
    BLINK = LedStateType(2, "BLINK")

# ----------------------------------------------------------

class GestureType:
    '''The defined units for optical sensor gesture types.'''
    class GestureType(vexEnum):
        pass
    NONE = GestureType(0, "NONE")
    UP = GestureType(1, "UP")
    DOWN = GestureType(2, "DOWN")
    LEFT = GestureType(3, "LEFT")
    RIGHT = GestureType(4, "RIGHT")

# ----------------------------------------------------------

class VexlinkType:
    '''The defined units for vexlink types.'''
    class VexlinkType(vexEnum):
        pass
    MANAGER = VexlinkType(1, "MANAGER")
    '''A vexlink type that is defined as the manager radio.'''
    WORKER = VexlinkType(2, "WORKER")
    '''A vexlink type that is defined as the worker radio.'''
    GENERIC = VexlinkType(3, "GENERIC")
    '''A vexlink type that is defined as a raw unmanaged link.'''

# ----------------------------------------------------------
# globals
#
PERCENT = PercentUnits.PERCENT
'''A percentage unit that represents a value from 0% to 100%'''
FORWARD = DirectionType.FORWARD
'''A direction unit that is defined as forward.'''
REVERSE = DirectionType.REVERSE
'''A direction unit that is defined as backward.'''
LEFT = TurnType.LEFT
'''A turn unit that is defined as left turning.'''
RIGHT = TurnType.LEFT
'''A turn unit that is defined as right turning.'''
DEGREES = RotationUnits.DEG
'''A rotation unit that is measured in degrees.'''
TURNS = RotationUnits.REV
'''A rotation unit that is measured in revolutions.'''
RPM = VelocityUnits.RPM
'''A velocity unit that is measured in rotations per minute.'''
DPS = VelocityUnits.DPS
'''A velocity unit that is measured in degrees per second.'''
SECONDS = TimeUnits.SECONDS
'''A time unit that is measured in seconds.'''
MSEC = TimeUnits.MSEC
'''A time unit that is measured in milliseconds.'''
INCHES = DistanceUnits.IN
'''A distance unit that is measured in inches.'''
MM = DistanceUnits.MM
'''A distance unit that is measured in millimeters.'''
XAXIS = AxisType.XAXIS
'''The X axis of the Inertial sensor.'''
YAXIS = AxisType.YAXIS
'''The Y axis of the Inertial sensor.'''
ZAXIS = AxisType.ZAXIS
'''The Z axis of the Inertial sensor.'''
ROLL = OrientationType.ROLL
'''roll, orientation around the X axis of the Inertial sensor.'''
PITCH = OrientationType.PITCH
'''pitch, orientation around the Y axis of the Inertial sensor.'''
YAW = OrientationType.YAW
'''yaw, orientation around the Z axis of the Inertial sensor.'''
PRIMARY = ControllerType.PRIMARY
'''A controller defined as a primary controller.'''
PARTNER = ControllerType.PARTNER
'''A controller defined as a partner controller.'''
COAST = BrakeType.COAST
'''A brake unit that is defined as motor coast.'''
BRAKE = BrakeType.BRAKE
'''A brake unit that is defined as motor brake.'''
HOLD = BrakeType.HOLD
'''A brake unit that is defined as motor hold.'''
VOLT = VoltageUnits.VOLT
'''A voltage unit that is measured in volts.'''
MV = VoltageUnits.MV
'''A voltage unit that is measured in millivolts.'''

# most functions will take number in either format
vexnumber = Union[int, float]
VelocityPercentUnits = Union[VelocityUnits.VelocityUnits, PercentUnits.PercentUnits]
TorquePercentUnits = Union[TorqueUnits.TorqueUnits, PercentUnits.PercentUnits]
TorquePercentCurrentUnits = Union[TorqueUnits.TorqueUnits, PercentUnits.PercentUnits, CurrentUnits.CurrentUnits]
TemperaturePercentUnits = Union[TemperatureUnits.TemperatureUnits, PercentUnits.PercentUnits]
AnalogPercentUnits = Union[AnalogUnits.AnalogUnits, PercentUnits.PercentUnits]
RotationPercentUnits = Union[RotationUnits.RotationUnits, PercentUnits.PercentUnits]
RotationTimeUnits = Union[RotationUnits.RotationUnits, TimeUnits.TimeUnits]

# ----------------------------------------------------------

def info():
    '''### return a string with VEX Python version information
    '''
    return "VEX V5 Python"

def sleep(duration: vexnumber, units=TimeUnits.MSEC):
    '''### delay the current thread for the provided number of seconds or milliseconds.

    #### Arguments:
        duration: The number of seconds or milliseconds to sleep for
        units:    The units of duration, optional, default is milliseconds

    #### Returns:
        None
    '''
    if units == TimeUnits.SECONDS:
        time.sleep(duration / 1000)
    else:
        time.sleep(duration)

def wait(duration: vexnumber, units=TimeUnits.MSEC):
    '''### delay the current thread for the provided number of seconds or milliseconds.

    #### Arguments:
        duration: The number of seconds or milliseconds to sleep for
        units:    The units of duration, optional, default is milliseconds

    #### Returns:
        None
    '''
    if units == TimeUnits.SECONDS:
        time.sleep(duration / 1000)
    else:
        time.sleep(duration)

def on_screen_errors(value: int):
    '''### enable or disable the display of brain on screen errors

    #### Arguments:
        value : True or False

    #### Returns:
        None
    '''
    pass

def clear_errors():
    '''### clear any brain on screen errors

    #### Arguments:
        None

    #### Returns:
        None
    '''
    pass

# ----------------------------------------------------------

class Color:
    '''### Color class - create a new color

    This class is used to create instances of color objects

    #### Arguments:
        value : The color value, can be specified in various ways, see examples.

    #### Returns:
        An instance of the Color class

    #### Examples:
        # create blue using hex value\\
        c = Color(0x0000ff)\n
        # create blue using r, g, b values\\
        c = Color(0, 0, 255)\n
        # create blue using web string\\
        c = Color("#00F")\n
        # create blue using web string (alternate)\\
        c = Color("#0000FF")\n
        # create red using an existing object\\
        c = Color(Color.RED)
    '''
    class DefinedColor:
        def __init__(self, value):
            self.value = value

    BLACK = DefinedColor(0x000000)
    '''predefined Color black'''
    WHITE = DefinedColor(0xFFFFFF)
    '''predefined Color white'''
    RED = DefinedColor(0xFF0000)
    '''predefined Color red'''
    GREEN = DefinedColor(0x00FF00)
    '''predefined Color green'''
    BLUE = DefinedColor(0x0000FF)
    '''predefined Color blue'''
    YELLOW = DefinedColor(0xFFFF00)
    '''predefined Color yellow'''
    ORANGE = DefinedColor(0xffa500)
    '''predefined Color orange'''
    PURPLE = DefinedColor(0xff00ff)
    '''predefined Color purple'''
    CYAN = DefinedColor(0x00ffff)
    '''predefined Color cyan'''
    TRANSPARENT = DefinedColor(0x000000)
    '''predefined Color transparent'''

    def __init__(self, *args):
        pass

    def rgb(self, *args):
        '''### change existing Color instance to new rgb value

        #### Arguments:
            value : The color value, can be specified in various ways, see examples.

        #### Returns:
            integer value representing the color

        #### Examples:
            # create a color that is red
            c = Color(0xFF0000)
            # change color to blue using single value
            c.rgb(0x0000FF)
            # change color to green using three values
            c.rgb(0, 255, 0)
        '''
        return 0

    def hsv(self, hue: vexnumber, saturation: vexnumber, value: vexnumber):
        '''### change existing Color instance using hsv

        #### Arguments:
            hue : The hue of the color
            saturation : The saturation of the color
            value : The brightness of the color

        #### Returns:
            integer value representing the color

        #### Examples:
            # create a color that is red
            c.hsv( 0, 1.0, 1.0)
        '''
        return 0

    def web(self, value: str):
        '''### change existing Color instance using web string

        #### Arguments:
            value : The new color as a web string

        #### Returns:
            integer value representing the color

        #### Examples:
            # create a color that is red
            c.web('#F00')
        '''
        return 0

    def is_transparent(self):
        '''### return whether color is transparent or not

        #### Arguments:
            None

        #### Returns:
            True or False

        #### Examples:
        '''
        return False

# ----------------------------------------------------------

class Brain:
    '''### Brain class

    The Brain class creates a number of instances of internal classes that allow access\\
    to the screen, battery, 3wire ports and sd card on the V5 brain.

    #### Arguments:
        None

    #### Returns:
        An instance of the Brain class

    #### Examples:
        brain = Brain()
    '''
    class Lcd:
        '''### Brain.Lcd class

        A class used to access to screen on the V5 for drawing and receiving touch events.

        #### Arguments:
            None

        #### Returns:
            An instance of the Brain.Lcd class
        '''
        def __init__(self):
            self._row = 0
            self._col = 0
            self._originx = 0
            self._originy = 0

        def set_cursor(self, row: vexnumber, col: vexnumber):
            '''### Set the cursor position used for printing text on the screen

            row and column spacing will take into account the selected font.\\
            The base cell size if 10x20 pixels for the MONO20 font.\\
            text may not accurately print if using a proportional font.\\
            The top, left corner of the screen is position 1,1

            #### Arguments:
                row : The cursor row
                col : The cursor column

            #### Returns:
                None
            '''
            self._row = row
            self._col = col

        def column(self):
            '''Return the current column where text will be printed'''
            return self._col

        def row(self):
            '''Return the current row where text will be printed'''
            return self._row

        def set_origin(self, x: vexnumber, y: vexnumber):
            '''### Set the origin used for drawing graphics on the screen

            drawing functions consider the top left corner of the screen as the origin.\\
            This function can move the origin to an alternate position such as the center of the screen.

            #### Arguments:
                x : The origins x position relative to top left corner
                y : The origins y position relative to top left corner

            #### Returns:
                None
            '''
            self._originx = x
            self._originy = y

        def set_font(self, fontname: FontType.FontType):
            '''### Set the font type used for printing text on the screen

            #### Arguments:
                fontname : The font name

            #### Returns:
                None

            #### Examples:
                brain.screen.font_type(FontType.MONO40)
            '''
            pass

        def set_pen_width(self, width: vexnumber):
            '''### Set the pen width used for drawing lines, rectangles and circles

            #### Arguments:
                width : The pen width

            #### Returns:
                None
            '''
            pass

        def set_pen_color(self, color):
            '''### Set the pen color used for drawing lines, rectangles and circles

            The color can be passed in similar ways to the Color class.\\
            The color is specific to the running thread.

            #### Arguments:
                color : The pen color

            #### Returns:
                None

            #### Examples:
                # set pen color red using a hex value\\
                brain.screen.set_pen_color(0xFF0000)

                # set pen color blue using predefined color\\
                brain.screen.set_pen_color(Color.BLUE)

                # set pen color green using web string\\
                brain.screen.set_pen_color("#00FF00")
            '''
            pass

        def set_fill_color(self, color):
            '''### Set the fill color used for drawing rectangles and circles

            The color can be passed in similar ways to the Color class.\\
            The color is specific to the running thread.

            #### Arguments:
                color : The fill color

            #### Returns:
                None

            #### Examples:
                # set pen color red using a hex value\\
                brain.screen.set_fill_color(0xFF0000)

                # set pen color blue using predefined color\\
                brain.screen.set_fill_color(Color.BLUE)

                # set pen color green using web string\\
                brain.screen.set_fill_color("#00FF00")
            '''
            pass

        def clear_screen(self, color=Color.BLACK):
            '''### Clear the whole screen to a single color

            The color can be passed in similar ways to the Color class.\\

            #### Arguments:
                color (optional) : The color the screen will be set to, default is BLACK

            #### Returns:
                None

            #### Examples:
                # clear screen to black\\
                brain.screen.clear_screen()

                # clear screen to blue using predefined color\\
                brain.screen.clear_screen(Color.BLUE)
            '''
            pass

        # deprecated
        def clear_line(self, number=None, color=Color.BLACK):
            pass

        def clear_row(self, number=None, color=Color.BLACK):
            '''### Clear screen row to a single color

            The color can be passed in similar ways to the Color class.\\

            #### Arguments:
                row (optional) : The row to clear, default is current cursor row
                color (optional) : The color the screen will be set to, default is BLACK

            #### Returns:
                None

            #### Examples:
                # clear row to black\\
                brain.screen.clear_row()

                # clear row 2 to red\\
                brain.screen.clear_row(2, Color.RED)
            '''
            pass

        # deprecated
        def new_line(self):
            pass

        def next_row(self):
            '''### Move the cursor to the beginning of the next row

            #### Arguments:
                None

            #### Returns:
                None
            '''
            pass

        def draw_pixel(self, x: vexnumber, y: vexnumber):
            '''### Draw a pixel on the screen using the current pen color.

            #### Arguments:
                x : The x position to draw the pixel referenced to the screen origin.
                y : The y position to draw the pixel referenced to the screen origin.

            #### Returns:
                None

            #### Examples:
                # draw a red pixel on the screen\\
                brain.screen.set_pen_color(Color.RED)\\
                brain.screen.draw_pixel(10, 10)
            '''
            pass

        def draw_line(self, x1: vexnumber, y1: vexnumber, x2: vexnumber, y2: vexnumber):
            '''### Draw a line on the screen using the current pen color.

            #### Arguments:
                x1 : The x position of the beginning of the line referenced to the screen origin.
                y1 : The y position of the beginning of the line referenced to the screen origin.
                x2 : The x position of the end of the line referenced to the screen origin.
                y2 : The y position of the end of the line referenced to the screen origin.

            #### Returns:
                None

            #### Examples:
                # draw a red line on the screen\\
                brain.screen.set_pen_color(Color.RED)\\
                brain.screen.draw_line(10, 10, 20, 20)
            '''
            pass

        def draw_rectangle(self, x: vexnumber, y: vexnumber, width: vexnumber, height: vexnumber, color: Any=None):
            '''### Draw a rectangle on the screen using the current pen and fill colors.

            #### Arguments:
                x : The x position of the rectangle top/left corner referenced to the screen origin.
                y : The y position of the rectangle top/left corner referenced to the screen origin.
                width : The width of the rectangle.
                height : The height of the rectangle.
                color (optional) : An optional fill color, the current fill color will be used if not supplied

            #### Returns:
                None

            #### Examples:
                # draw a green rectangle on the screen that is filled using blue\\
                brain.screen.set_pen_color(Color.GREEN)\\
                brain.screen.set_fill_color(Color.BLUE)\\
                brain.screen.draw_rectangle(10, 10, 20, 20)

                # draw a green rectangle on the screen that is filled using red\\
                brain.screen.set_pen_color(Color.GREEN)\\
                brain.screen.draw_rectangle(50, 50, 20, 20, Color.RED)
            '''
            pass

        def draw_circle(self, x: vexnumber, y: vexnumber, radius: vexnumber, color: Any=None):
            '''### Draw a circle on the screen using the current pen and fill colors.

            #### Arguments:
                x : The x position of the circle center referenced to the screen origin.
                y : The y position of the circle center referenced to the screen origin.
                radius : The height of the circle.
                color (optional) : An optional fill color, the current fill color will be used if not supplied

            #### Returns:
                None

            #### Examples:
                # draw a green circle on the screen that is filled using blue\\
                brain.screen.set_pen_color(Color.GREEN)\\
                brain.screen.set_fill_color(Color.BLUE)\\
                brain.screen.draw_circle(50, 50, 10)

                # draw a green circle on the screen that is filled using red\\
                brain.screen.set_pen_color(Color.GREEN)\\
                brain.screen.draw_circle(100, 50, 10, Color.RED)
            '''
            pass

        def get_string_width(self, *args):
            '''### get width of a string

            #### Arguments:
                arguments are in the same format as can be passed to the print function.

            #### Returns:
                width of string as integer.
            '''
            return 0

        def get_string_height(self, *args):
            '''### get height of a string

            #### Arguments:
                arguments are in the same format as can be passed to the print function.

            #### Returns:
                height of string as integer.
            '''
            return 0

        def print(self, *args, **kwargs):
            '''### print text on the screen using current curser position.

            #### Arguments:
                Optional keyword arguments:
                sep : string inserted between values, default a space.
                precision : the number of decimal places to display when printing simple numbers, default is 2

            #### Returns:
                None

            #### Examples:
                # print the number 1 on the screen at current cursor position\\
                brain.screen.print(1)

                # print the numbers 1, 2, 3 and 4 on the screen at current cursor position separated by a '-'\\
                brain.screen.print(1, 2, 3, 4, sep='-')

                # print motor1 velocity on the screen using a format string\\
                brain.screen.print("motor  1 : % 7.2f" %(motor1.velocity()))
            '''
            pass

        def print_at(self, *args, **kwargs):
            '''### print text on the screen at x and coordinates.

            #### Arguments:
                Required keyword arguments
                x : The x position of the text referenced to the screen origin.
                y : The y position of the text referenced to the screen origin.

                Optional keyword arguments:
                sep : string inserted between values, default a space.
                precision : the number of decimal places to display when printing simple numbers, default is 2
                opaque : text does not clear background pixels if set to False. default is True.

            #### Returns:
                None

            #### Examples:
                # print the number 1 on the screen at position x=100, y=40\\
                brain.screen.print_at(1, x=100, y=40)

                # print the numbers 1, 2, 3 and 4 on the screen at position x=100, y=40\\
                brain.screen.print_at(1, 2, 3, 4, x=100, y=40)

                # print motor1 velocity on the screen using a format string at position x=100, y=40\\
                brain.screen.print_at("motor  1 : % 7.2f" %(motor1.velocity()), x=100, y=40)
            '''
            pass

        def pressed(self, callback: Callable[...,None], arg: tuple=()):
            '''### Register a function to be called when the screen is pressed

            #### Arguments:
                callback : A function that will be called when the screen is pressed
                arg (optional) : A tuple that is used to pass arguments to the callback function.

            #### Returns:
                An instance of the Event class

            #### Examples:
                def foo():
                    print("screen pressed")

                brain.screen.pressed(foo)
            '''
            return Event(callback, arg)

        def released(self, callback: Callable[...,None], arg: tuple=()):
            '''### Register a function to be called when the screen is released (touch removed)

            #### Arguments:
                callback : A function that will be called when the screen is released
                arg (optional) : A tuple that is used to pass arguments to the callback function.

            #### Returns:
                An instance of the Event class

            #### Examples:
                def foo():
                    print("screen released")

                brain.screen.released(foo)
            '''
            return Event(callback, arg)

        def x_position(self):
            '''### The X coordinate of the last screen event, press or release

            #### Arguments:
                None

            #### Returns:
                The X coordinate as an int

            #### Examples:
                def foo():
                    print("screen pressed at ", brain.screen.x_position())

                brain.screen.pressed(foo)
            '''
            return 0

        def y_position(self):
            '''### The Y coordinate of the last screen event, press or release

            #### Arguments:
                None

            #### Returns:
                The Y coordinate as an int

            #### Examples:
                def foo():
                    print("screen pressed at ", brain.screen.y_position())

                brain.screen.pressed(foo)
            '''
            return 0

        def pressing(self):
            '''### Returns whether the screen is currently being pressed (touched)

            #### Arguments:
                None

            #### Returns:
                True or False
            '''
            return False

        def draw_image_from_file(self, filename: str, x: vexnumber, y: vexnumber):
            '''### Display the named image from the SD Card

            #### Arguments:
                filename : The file name of the image.
                x : The X coordinate for the top left corner of the image on the screen
                y : The Y coordinate for the top left corner of the image on the screen

            #### Returns:
                True if successfully drawn, False on error

            #### Examples:
                # draw the vex.bmp image on the screen at coordinate 0, 0\\
                # an image named vex.bmp must be on the SD Card in the root folder\\
                brain.screen.draw_image_from_file('vex.bmp', 0, 0)
            '''
            pass

        def render(self):
            '''### Switch drawing to double buffered and render too screen.

            Once called, further drawing will not appear on the screen until the next time\\
            render is called.  This function will block until the screen can be updated.

            #### Arguments:
                None

            #### Returns:
                True if buffer was successfully rendered to screen.

            #### Examples:
            '''
            return True

        def set_clip_region(self, x: vexnumber, y: vexnumber, width: vexnumber, height: vexnumber):
            '''### Sets the clip region for drawing to the supplied rectangle.

            All drawing is clipped to the given rectangle.\\
            This is set on a per thread basis.

            #### Arguments:
                x : The x position of the rectangle top/left corner referenced to the screen origin.
                y : The y position of the rectangle top/left corner referenced to the screen origin.
                width : The width of the rectangle.
                height : The height of the rectangle.

            #### Returns:
                None

            #### Examples:
            '''
            pass

    class Battery:
        '''### Battery class - access the brain battery

        #### Arguments:
            None

        #### Returns:
            Instance of Battery class

        #### Examples:
        '''
        def __init__(self):
            pass

        def capacity(self):
            '''### read remaining capacity of the battery

            #### Arguments:
                None

            #### Returns:
                capacity as percentage

            #### Examples:
            '''
            return 100

        def temperature(self, units: TemperaturePercentUnits = PercentUnits.PERCENT):
            '''### read the temperature of the battery

            #### Arguments:
                units (optional) : PERCENT, CELSIUS or FAHRENHEIT, default is CELSIUS

            #### Returns:
                temperature in supplied units

            #### Examples:
            '''
            return 20

        def voltage(self, units = VoltageUnits.MV):
            '''### read the voltage of the battery

            #### Arguments:
                units (optional) : VOLTS or MV, default is MV

            #### Returns:
                voltage in supplied units

            #### Examples:
            '''
            return 12800

        def current(self, units = CurrentUnits.AMP):
            '''### read the current of the battery

            #### Arguments:
                units (optional) : AMP, default is mA but jot available as an enum.

            #### Returns:
                current in supplied units

            #### Examples:
            '''
            return 1

    class Sdcard:
        '''### Sdcard class - access the brain sdcard

        #### Arguments:
            None

        #### Returns:
            Instance of Sdcard class

        #### Examples:
        '''
        def __init__(self):
            pass

        def is_inserted(self):
            '''### returns status of SD Card

            #### Arguments:
                None

            #### Returns:
                True if an sdcard is inserted into the brain

            #### Examples:
            '''
            return True

        def filesize(self, filename: str):
            '''### returns the size in bytes of the named file

            #### Arguments:
                filename : The name of the file to check

            #### Returns:
                size of file in bytes

            #### Examples:
            '''
            return 0

        def loadfile(self, filename: str, *args):
            '''### load the named file

            #### Arguments:
                filename : The name of the file to read
                buffer (optional) : A bytearray to read the file into

            #### Returns:
                A bytearray with file data

            #### Examples:
                # read file into new bytearray\\
                b = brain.sdcard.loadfile('MyTextFile.txt')
            '''
            return bytearray([0])

        def savefile(self, filename: str, *args):
            '''### Save a bytearray into a named file

            If the optional bytearray is None, then an empty file is created.

            #### Arguments:
                filename : The name of the file to write
                buffer (optional) : A bytearray to write into the file

            #### Returns:
                The number of bytes written

            #### Examples:
                # write bytearray into file\\
                brain.sdcard.savefile('MyTextFile.txt', bytearray("Hello "))
            '''
            return 0

        def appendfile(self, filename: str, *args):
            '''### append a bytearray into a named file

            Append is used to add more data to an existing file.

            #### Arguments:
                filename : The name of the file to write
                buffer : A bytearray to write into the file

            #### Returns:
                The number of bytes written

            #### Examples:
                # append bytearray into file\\
                brain.sdcard.appendfile('MyTextFile.txt', bytearray("World "))
            '''
            return 0

        def size(self, filename: str):
            '''### returns the size in bytes of the named file

            #### Arguments:
                filename : The name of the file to check

            #### Returns:
                size of file in bytes

            #### Examples:
            '''
            return 0

        def exists(self, *args):
            '''### check to see if named file exists on the sd card

            #### Arguments:
                filename : The name of the file to check

            #### Returns:
                True if file exists

            #### Examples:
            '''
            return False

    def __init__(self, *args):
        self._index = 31
        self.screen = Brain.Lcd()
        ''' An instance of the Lcd class'''
        self.timer = Timer()
        ''' An instance of the Timer class'''
        self.battery = Brain.Battery()
        ''' An instance of the Battery class'''
        self.sdcard = Brain.Sdcard()
        ''' An instance of the Sdcard class'''
        self.three_wire_port = Triport(Ports.PORT22)
        ''' An instance of the Triport (3wire) class'''

    def program_stop(self):
        '''### stop execution of the current program

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
        '''
        pass

# ----------------------------------------------------------

class Competition:
    '''### Competition class - create a class used for competition control

    #### Arguments:
        driver : A function called as a thread when the driver control period starts.
        autonomous : A function called as a thread when the driver control period starts.

    #### Returns:
        An instance of the Competition class

    #### Examples:
        def driver():
            print("driver called")

        def auton():
            print("auton called")

        comp = Competition(driver, auton)
    '''
    def __init__(self, driver: Callable[[],None], autonomous: Callable[[],None]):
        self._driver_cb = driver
        self._auton_cb = autonomous

        self._driver_thread = None
        self._auton_thread = None

    @staticmethod
    def is_enabled():
        '''### return enable/disable state of the robot

        #### Arguments:
            None

        #### Returns:
            True if the robot is enabled
        '''
        return True

    @staticmethod
    def is_driver_control():
        '''### return driver control state of the robot

        #### Arguments:
            None

        #### Returns:
            True if driver control is enabled
        '''
        return True

    def is_autonomous(self=None):
        '''### return autonomous state of the robot

        #### Arguments:
            None

        #### Returns:
            True if autonomous is enabled
        '''
        return False

    def is_competition_switch(self=None):
        '''### return connection state of the competition switch

        #### Arguments:
            None

        #### Returns:
            True if competition switch is connected
        '''
        return True

    def is_field_control(self=None):
        '''### return connection state of field controller

        #### Arguments:
            None

        #### Returns:
            True if field controller is connected
        '''
        return False

# ----------------------------------------------------------

class Controller:
    '''### Controller class - create a class to access the controller

    #### Arguments:
        None

    #### Returns:
        An instance of the Controller class

    #### Examples:
    '''
    class Axis:
        '''### Axis class

        #### Arguments:
            None

        #### Returns:
            An instance of an Axis class

        #### Examples:
        '''
        def __init__(self, *args):
            pass

        def value(self):
            '''### Return the current position of the axis

            #### Arguments:
                None

            #### Returns:
                A value in the range +/- 127

            #### Examples:
                a = controller.axis1.position()
            '''
            return 0

        def position(self):
            '''### Return the current position of the axis in percentage

            #### Arguments:
                None

            #### Returns:
                A value in the range +/- 100

            #### Examples:
                a = controller.axis1.position()
            '''
            return 0

        def changed(self, callback: Callable[...,None], arg: tuple=()):
            '''### Register a function to be called when the axis value changes

            #### Arguments:
                callback : A function that will be called when the axis value changes
                arg (optional) : A tuple that is used to pass arguments to the callback function.

            #### Returns:
                An instance of the Event class

            #### Examples:
                def foo():
                    print("axis changed")

                controller.axis1.changed(foo)
            '''
            return Event(callback, arg)

    class Button:
        def __init__(self, *args):
            pass

        def pressing(self):
            '''### Returns whether a button is currently being pressed

            #### Arguments:
                None

            #### Returns:
                True or False
            '''
            return False

        def pressed(self, callback: Callable[...,None], arg: tuple=()):
            '''### Register a function to be called when a button is pressed

            #### Arguments:
                callback : A function that will be called when the button is pressed
                arg (optional) : A tuple that is used to pass arguments to the callback function.

            #### Returns:
                An instance of the Event class

            #### Examples:
                def foo():
                    print("button pressed")

                controller.buttonL1.pressed(foo)
            '''
            return Event(callback, arg)

        def released(self, callback: Callable[...,None], arg: tuple=()):
            '''### Register a function to be called when a button is released

            #### Arguments:
                callback : A function that will be called when the button is released
                arg (optional) : A tuple that is used to pass arguments to the callback function.

            #### Returns:
                An instance of the Event class

            #### Examples:
                def foo():
                    print("button released")

                controller.buttonL1.released(foo)
            '''
            return Event(callback, arg)

    class Lcd:
        '''### Controller.Lcd class

        A class used to access the screen on the V5 controller.

        #### Arguments:
            None

        #### Returns:
            An instance of the Brain.Lcd class
        '''
        def __init__(self, *args):
            self._row = 0
            self._col = 0
            pass

        def set_cursor(self, row: vexnumber, col: vexnumber):
            '''### Set the cursor position used for printing text on the screen

            V5 controller has at most 3 lines of text

            #### Arguments:
                row : The cursor row.  1, 2 or 3
                col : The cursor column.  The first column is 1.

            #### Returns:
                None
            '''
            self._row = row
            self._col = col

        def column(self):
            '''Return the current column where text will be printed'''
            return self._col

        def row(self):
            '''Return the current row where text will be printed'''
            return self._row

        def print(self, *args, **kwargs):
            '''### print text on the screen using current curser position.

            #### Arguments:
                Optional keyword arguments:
                sep : string inserted between values, default a space.
                precision : the number of decimal places to display when printing simple numbers, default is 2

            #### Returns:
                None

            #### Examples:
                # print the number 1 on the screen at current cursor position\\
                controller.screen.print(1)

                # print the numbers 1, 2, 3 and 4 on the screen at current cursor position separated by a '-'\\
                controller.screen.print(1, 2, 3, 4, sep='-')

                # print motor1 velocity on the screen using a format string\\
                controller.screen.print("motor  1 : % 7.2f" %(motor1.velocity()))
            '''
            pass

        def clear_screen(self):
            '''### Clear the whole screen

            #### Arguments:
                None

            #### Returns:
                None

            #### Examples:
                controller.screen.clear_screen()
            '''
            pass

        # deprecated
        def clear_line(self, number: vexnumber):
            pass

        def clear_row(self, number: vexnumber):
            '''### Clear screen row

            #### Arguments:
                row (optional) : The row to clear, 1, 2, or 3, default is current cursor row

            #### Returns:
                None

            #### Examples:
                # clear row 2\\
                controller.screen.clear_row(2)
            '''
            pass

        # deprecated
        def new_line(self):
            pass

        def next_row(self):
            '''### Move the cursor to the beginning of the next row

            #### Arguments:
                None

            #### Returns:
                None
            '''
            pass

    def __init__(self, *args):
        self.axis1 = Controller.Axis()
        '''The joystick axis 1 on the controller'''
        self.axis2 = Controller.Axis()
        '''The joystick axis 2 on the controller'''
        self.axis3 = Controller.Axis()
        '''The joystick axis 3 on the controller'''
        self.axis4 = Controller.Axis()
        '''The joystick axis 4 on the controller'''

        self.buttonL1 = Controller.Button()
        '''The L1 button on the controller'''
        self.buttonL2 = Controller.Button()
        '''The L2 button on the controller'''
        self.buttonR1 = Controller.Button()
        '''The R1 button on the controller'''
        self.buttonR2 = Controller.Button()
        '''The R2 button on the controller'''
        self.buttonUp = Controller.Button()
        '''The Up button on the controller'''
        self.buttonDown = Controller.Button()
        '''The Down button on the controller'''
        self.buttonLeft = Controller.Button()
        '''The Left button on the controller'''
        self.buttonRight = Controller.Button()
        '''The Right button on the controller'''
        self.buttonA = Controller.Button()
        '''The A button on the controller'''
        self.buttonB = Controller.Button()
        '''The B button on the controller'''
        self.buttonX = Controller.Button()
        '''The X button on the controller'''
        self.buttonY = Controller.Button()
        '''The Y button on the controller'''

        self.screen = Controller.Lcd()
        ''' An instance of the Lcd class'''

    def rumble(self, pattern: str):
        '''### Send a rumble string to the V5 controller

        #### Arguments:
            pattern : A pattern using '.' and '-' for short and long rumbles.

        #### Returns:
            None

        #### Examples:
            controller.rumble('..--')
        '''
        return 0

# ----------------------------------------------------------

class Event:
    '''### Event class - create a new event

    A function is registered that will be called when the event broadcast() function is called.
    More than one function can be assigned to a single event.

    #### Arguments:
        callback (optional) : A function that will be called when the event is broadcast.
        arg (optional) : A tuple that is used to pass arguments to the event callback function.

    #### Returns:
        An instance of the Event class

    #### Examples:
        def foo():
            print("foo")

        def bar():
            print("bar")

        e = Event(foo)\\
        e.set(bar)

        # There needs to be some small delay after events are created before they can be broadcast to\\
        sleep(20)

        # cause both foo and bar to be called\\
        e.broadcast()
    '''
    def __init__(self, callback=None, arg: tuple=()):
        pass

    def __call__(self, callback: Callable[...,None], arg: tuple=()):
        '''### Add callback function to an existing event

        #### Arguments:
            callback : A function that will be called when the event is broadcast.
            arg (optional) : A tuple that is used to pass arguments to the event callback function.

        #### Returns:
            None

        #### Examples:
            def bar():
                print("bar")

            # add callback function to existing event e\\
            e(bar)
        '''
        pass

    def set(self, callback: Callable[...,None], arg: tuple=()):
        '''### Add callback function to an existing event

        #### Arguments:
            callback : A function that will be called when the event is broadcast.
            arg (optional) : A tuple that is used to pass arguments to the event callback function.

        #### Returns:
            None

        #### Examples:
            def bar():
                print("bar")

            # add callback function to existing event e\\
            e.set(bar)
        '''
        pass

    def broadcast(self):
        '''### Broadcast to the event and cause all registered callback function to run

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            # broadcast to an existing event e\\
            e.broadcast()
        '''
        pass

    def broadcast_and_wait(self, timeout=60000):
        '''### Broadcast to the event and cause all registered callback function to run

        This is similar to broadcast except that it will wait for all registered callbacks to complete before returning.

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            # broadcast to an existing event e, wait for completion\\
            e.broadcast_and_wait()
        '''
        pass

# ----------------------------------------------------------

class Gps:
    '''### Gps class - a class for working with the gps sensor

    #### Arguments:
        port : The smartport this device is attached to
        origin_x (optional) : The X location of the GPS with respect to origin of the robot.
        origin_y (optional) : The Y location of the GPS with respect to origin of the robot.\\
          note. both X and Y must be supplied
        units (optional) : The units that X and Y location are specified in, default is MM
        heading_offset (optional) : The gps sensor rotational offset.

    #### Returns:
        An instance of the Gps class

    #### Examples:
        gps1 = Gps(Ports.PORT1)
    '''
    def __init__(self, port, *args):
        self._index = port

    def installed(self, *args):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def set_heading(self, value, units=RotationUnits.DEG):
        '''### set the gps heading to a new value

        The new value for heading should be in the range 0 - 359.99 degrees.

        #### Arguments:
            value : The new value to use for heading.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of heading to 180 degrees\\
            gps1.set_heading(180)
        '''
        pass

    def reset_heading(self):
        '''### Reset the gps heading to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def heading(self, units=RotationUnits.DEG):
        '''### read the current heading of the gps

        heading will be returned in the range 0 - 359.99 degrees

        #### Arguments:
            units (optional) : The units to return the heading in, default is DEGREES

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current heading for the gps\\
            value = gps1.heading()
        '''
        return 20

    def set_rotation(self, value, units=RotationUnits.DEG):
        '''### set the gps rotation to a new value

        #### Arguments:
            value : The new value to use for rotation.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of rotation to 180 degrees\\
            gps1.set_rotation(180)
        '''
        pass

    def reset_rotation(self):
        '''### Reset the gps rotation to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def rotation(self, units=RotationUnits.DEG):
        '''### read the current rotation of the gps

        rotation is not limited, it can be both positive and negative and shows the absolute angle of the gps.

        #### Arguments:
            units (optional) : The units to return the rotation in, default is DEGREES

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current rotation for the gps\\
            value = gps1.rotation()
        '''
        return 20

    def x_position(self, units=DistanceUnits.MM):
        '''### read the current x coordinate of the gps

        #### Arguments:
            units (optional) : The units to return the position in, default is MM

        #### Returns:
            A value for the x coordinate in the units specified.

        #### Examples:
            # get the current x coordinate for the gps\\
            posx = gps1.x_position()
        '''
        return 0

    def y_position(self, units=DistanceUnits.MM):
        '''### read the current y coordinate of the gps

        #### Arguments:
            units (optional) : The units to return the position in, default is MM

        #### Returns:
            A value for the y coordinate in the units specified.

        #### Examples:
            # get the current y coordinate for the gps\\
            posy = gps1.y_position()
        '''
        return 0

    def quality(self):
        '''### read the current quality of the gps data

        A quality of 100 indicates the gps can see the gps field strip and is returning good readings\\
        The value for quality will reduce as the confidence in x and y location lowers.

        #### Arguments:
            None

        #### Returns:
            A value of quality in the range 0 to 100

        #### Examples:
            # get the current location and heading quality for the gps\\
            q = gps1.quality()
        '''
        return 100

    def set_origin(self, x=0, y=0, units=DistanceUnits.MM):
        '''### set the origin of the gps sensor

        An alternate way of setting sensor origin if not provided in the Gps class constructor.

        #### Arguments:
            x : The X location of the GPS with respect to origin of the robot.
            y : The Y location of the GPS with respect to origin of the robot.\\
              note. both X and Y must be supplied
            units (optional) : The units that X and Y location are specified in, default is MM

        #### Returns:
            None

        #### Examples:
            # set the origin of the gps\\
            gps1.set_origin(6, -6, INCHES)
        '''
        pass

    def set_location(self, x, y, units=DistanceUnits.MM, angle=0, units_r=RotationUnits.DEG):
        '''### set the initial location of the robot

        This gives a hint as to the location of the robot/gps sensor when it is first initialized.\\
        This can be used if in the initial position the gps cannot see the gps field strip.

        #### Arguments:
            x : The initial X coordinate.
            y : The initial Y coordinate.\\
              note. both X and Y must be supplied
            units (optional) : The units that X and Y coordinates are specified in, default is MM
            angle (optional) : The initial heading of the robot.
            units_r (optional) : The units that angle is specified in, default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the initial location of the gps\\
            gps1.set_location(1000, -1000, MM, 90, DEGREES)
        '''
        pass

    def calibrate(self):
        '''not used on the GPS sensor'''
        pass

    def is_calibrating(self):
        '''not used on the GPS sensor'''
        return False

    def orientation(self, axis:OrientationType.OrientationType, units=RotationUnits.DEG):
        '''### read the orientation for one axis of the gps

        #### Arguments:
            axis : The axis to read
            units (optional) : The units to return the orientation in, default is DEGREES

        #### Returns:
            A value for the axis orientation in the units specified.

        #### Examples:
            # get the pitch value for the gps\\
            pitch = gps1.orientation(OrientationType.PITCH)
        '''
        return 0.0

    def gyro_rate(self, axis: AxisType.AxisType, units=VelocityUnits.DPS):
        '''### read the gyro rate for one axis of the gps

        #### Arguments:
            axis : The axis to read
            units (optional) : The units to return the gyro rate in, default is DPS

        #### Returns:
            A value for the gyro rate of the axis in the units specified.

        #### Examples:
            # get the gyro rate for the Z axis of the gps\\
            zrate = gps1.gyro_rate(ZAXIS)
        '''
        return 0.0

    def acceleration(self, axis: AxisType.AxisType):
        '''### read the acceleration for one axis of the gps

        #### Arguments:
            axis : The axis to read

        #### Returns:
            A value for the acceleration of the axis in units of gravity.

        #### Examples:
            # get the acceleration for the Z axis of the gps\\
            zaccel = gps1.acceleration(ZAXIS)
        '''
        return 0.0

    def set_sensor_rotation(self, value, units=RotationUnits.DEG):
        '''### set the sensor rotation of the gps sensor with respect to the robot.

        This allows heading and rotation methods to return angles relative to the robot rather than the gps.

        #### Arguments:
            value : The angle of the GPS with respect to the robot.
            units (optional) : The units that value is specified in, default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the sensor rotation of the gps\\
            gps1.set_sensor_rotation(180, DEGREES)
        '''
        pass

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the gps heading changes

        This is not particularly useful as gps heading is not stable and will cause many events.

        #### Arguments:
            callback : A function that will be called when the value of the gps heading changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("heading changed")

            gps1.changed(foo)
        '''
        return Event(callback, arg)

    def set_turn_type(self, turntype):
        '''### set the direction that returns positive values for heading

        An advanced function that is not generally used.

        #### Arguments:
            turntype : TurnType.LEFT or TurnType.RIGHT

        #### Returns:
            None
        '''
        pass

    def get_turn_type(self):
        '''### get the direction that returns positive values for heading

        An advanced function that is not generally used.

        #### Arguments:
            None

        #### Returns:
            The current TurnType, LEFT or RIGHT
        '''
        return TurnType.RIGHT

# ----------------------------------------------------------

class Inertial:
    '''### Inertial class - a class for working with the inertial sensor

    #### Arguments:
        port : The smartport this device is attached to

    #### Returns:
        An instance of the Inertial class

    #### Examples:
        imu1 = Inertial(Ports.PORT1)
    '''
    def __init__(self, port):
        self._index = port

    def installed(self, *args):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def set_heading(self, value, units=RotationUnits.DEG):
        '''### set the inertial sensor heading to a new value

        The new value for heading should be in the range 0 - 359.99 degrees.

        #### Arguments:
            value : The new value to use for heading.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of heading to 180 degrees\\
            imu1.set_heading(180)
        '''
        pass

    def reset_heading(self):
        '''### Reset the inertial sensor heading to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def heading(self, units=RotationUnits.DEG):
        '''### read the current heading of the inertial sensor

        heading will be returned in the range 0 - 359.99 degrees

        #### Arguments:
            units (optional) : The units to return the heading in, default is DEGREES

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current heading for the inertial sensor\\
            value = imu1.heading()
        '''
        return 20

    def set_rotation(self, value, units=RotationUnits.DEG):
        '''### set the inertial sensor rotation to a new value

        #### Arguments:
            value : The new value to use for rotation.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of rotation to 180 degrees\\
            imu1.set_rotation(180)
        '''
        pass

    def reset_rotation(self):
        '''### Reset the inertial sensor rotation to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def rotation(self, units=RotationUnits.DEG):
        '''### read the current rotation of the inertial sensor

        rotation is not limited, it can be both positive and negative and shows the absolute angle of the gps.

        #### Arguments:
            units (optional) : The units to return the rotation in, default is DEGREES

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current rotation for the inertial sensor\\
            value = imu1.rotation()
        '''
        return 20

    def calibrate(self):
        '''### Start calibration of the inertial sensor

        Calibration should done when the inertial sensor is not moving.

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            # start calibration\\
            imu1.calibrate()\\
            # wait for completion\\
            while imu1.is_calibrating():\\
                sleep(50, MSEC)
        '''
        pass

    def is_calibrating(self):
        '''### check the calibration status of the inertial sensor

        Calibration should done when the inertial sensor is not moving.

        #### Arguments:
            None

        #### Returns:
            True when the inertial sensor is calibrating

        #### Examples:
            # start calibration\\
            imu1.calibrate()\\
            # wait for completion\\
            while imu1.is_calibrating():\\
                sleep(50, MSEC)
        '''
        return False

    def orientation(self, axis: OrientationType.OrientationType, units=RotationUnits.DEG):
        '''### read the orientation for one axis of the inertial sensor

        #### Arguments:
            axis : The axis to read
            units (optional) : The units to return the orientation in, default is DEGREES

        #### Returns:
            A value for the axis orientation in the units specified.

        #### Examples:
            # get the pitch value for the inertial sensor\\
            pitch = imu1.orientation(OrientationType.PITCH)
        '''
        return 0.0

    def gyro_rate(self, axis: AxisType.AxisType, units=VelocityUnits.DPS):
        '''### read the gyro rate for one axis of the inertial sensor

        #### Arguments:
            axis : The axis to read
            units (optional) : The units to return the gyro rate in, default is DPS

        #### Returns:
            A value for the gyro rate of the axis in the units specified.

        #### Examples:
            # get the gyro rate for the Z axis of the inertial sensor\\
            zrate = imu1.gyro_rate(ZAXIS)
        '''
        return 0.0

    def acceleration(self, axis: AxisType.AxisType):
        '''### read the acceleration for one axis of the inertial sensor

        #### Arguments:
            axis : The axis to read

        #### Returns:
            A value for the acceleration of the axis in units of gravity.

        #### Examples:
            # get the acceleration for the Z axis of the inertial sensor\\
            zaccel = imu1.acceleration(ZAXIS)
        '''
        return 0.0

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the inertial sensor heading changes

        #### Arguments:
            callback : A function that will be called when the value of the inertial sensor heading changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("heading changed")

            imu1.changed(foo)
        '''
        return Event(callback, arg)

    def collision(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the inertial sensor detects a collision

        #### Arguments:
            callback : A function that will be called when the inertial sensor detects a collision
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("collision")

            imu1.collision(foo)
        '''
        return Event(callback, arg)

    def set_turn_type(self, turntype):
        '''### set the direction that returns positive values for heading

        An advanced function that is not generally used.

        #### Arguments:
            turntype : TurnType.LEFT or TurnType.RIGHT

        #### Returns:
            None
        '''
        pass

    def get_turn_type(self):
        '''### get the direction that returns positive values for heading

        An advanced function that is not generally used.

        #### Arguments:
            None

        #### Returns:
            The current TurnType, LEFT or RIGHT
        '''
        return TurnType.RIGHT

# ----------------------------------------------------------

class Motor:
    '''### Motor class - use this to create an instance of a V5 smart motor

    #### Arguments:
        port : The smartport this device is attached to
        gears (optional) : The gear cartridge installed in the motor, default is the green 18_1
        reverse (optional) : Should the motor's spin direction be reversed, default is False

    #### Returns:
        A new Motor object.

    #### Examples:
        motor1 = Motor(Ports.PORT1)\\
        motor2 = Motor(Ports.PORT2, GearSetting.RATIO_36_1)\\
        motor3 = Motor(Ports.PORT3, True)\\
        motor4 = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
    '''
    def __init__(self, port: int, *args):
        self._index = port

        self._timeout = 10000
        self._velocity = 50
        self._mode = BrakeType.COAST
        self._brakeMode = BrakeType.COAST
        self._spinMode = False

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the motor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def set_velocity(self, value: vexnumber, units: VelocityPercentUnits=VelocityUnits.RPM):
        '''### Set default velocity for the motor
        This will be the velocity used for subsequent calls to spin if a velocity is not provided
        to that function.

        #### Arguments:
            value : The new velocity
            units : The units for the supplied velocity, the default is RPM

        #### Returns:
            None
        '''
        self._velocity = value

    def set_reversed(self, value: bool):
        '''### Set the reverse flag for the motor
        Setting the reverse flag will cause spin commands to run the motor in reverse.

        #### Arguments:
            value : Reverse flag, True or False

        #### Returns:
            None
        '''
        pass

    def set_stopping(self, value: BrakeType.BrakeType):
        '''### Set the stopping mode of the motor
        Setting the action for the motor when stopped.

        #### Arguments:
            value : The stopping mode, COAST, BRAKE or HOLD

        #### Returns:
            None
        '''
        pass

    def reset_position(self):
        '''### Reset the motor position to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set_position(self, value: vexnumber, units=RotationUnits.DEG):
        '''### Set the current position of the motor
        The position returned by the position() function is set to this value.

        #### Arguments:
            value : The new position
            units : The units for the provided position, the default is DEGREES

        #### Returns:
            None
        '''
        pass

    def set_timeout(self, value: vexnumber, units=TimeUnits.MSEC):
        '''### Set the timeout value used by the motor
        The timeout value is used when performing spin_to_position and spin_for commands.  If timeout is
         reached and the motor has not completed moving, then the spin... function will return False.

        #### Arguments:
            value : The new timeout
            units : The units for the provided timeout, the default is MSEC

        #### Returns:
            None
        '''
        self._timeout = value

    def get_timeout(self):
        '''### Returns the current value of motor timeout

        #### Arguments:
            None

        #### Returns:
            The current timeout value
        '''
        return self._timeout

    def spin(self, direction: DirectionType.DirectionType, *args, **kwargs):
        '''### Spin the motor using the provided arguments

        #### Arguments:
            direction : The direction to spin the motor, FORWARD or REVERSE
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units (optional) : The units of the provided velocity, default is RPM

        #### Returns:
            None

        #### Examples:
            # spin motor forward at velocity set with set_velocity\\
            motor1.spin(FORWARD)\n
            # spin motor forward at 50 rpm\\
            motor1.spin(FORWARD, 50)\n
            # spin with negative velocity, ie. backwards\\
            motor1.spin(FORWARD, -20)\n
            # spin motor forwards with 100% velocity\\
            motor1.spin(FORWARD, 100, PERCENT)\n
            # spin motor forwards at 50 rpm\\
            motor1.spin(FORWARD, 50, RPM)\n
            # spin motor forwards at 360 dps\\
            motor1.spin(FORWARD, 360.0, VelocityUnits.DPS)
        '''
        pass

    def spin_to_position(self, rotation: vexnumber, *args, **kwargs):
        '''### Spin the motor to an absolute position using the provided arguments
        Move the motor to the requested position.\\
        This function supports keyword arguments.

        #### Arguments:
            rotation : The position to spin the motor to
            units (optional) : The units for the provided position, the default is DEGREES
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None

        #### Examples:
            # spin to 180 degrees\\
            motor1.spin_to_position(180)\n
            # spin to 2 TURNS (revolutions)\\
            motor1.spin_to_position(2, TURNS) \n
            # spin to 180 degrees at 25 rpm\\
            motor1.spin_to_position(180, DEGREES, 25, RPM)\n
            # spin to 180 degrees and do not wait for completion\\
            motor1.spin_to_position(180, DEGREES, False)\n
            # spin to 180 degrees and do not wait for completion\\
            motor1.spin_to_position(180, DEGREES, wait=False)
        '''
        pass

    def spin_for(self, direction: DirectionType.DirectionType, rot_or_time: vexnumber, *args, **kwargs):
        '''### Spin the motor to a relative position using the provided arguments
        Move the motor to the requested position or for the specified amount of time.\\
        The position is relative (ie. an offset) to the current position\\
        This function supports keyword arguments.

        #### Arguments:
            dir : The direction to spin the motor, FORWARD or REVERSE
            rot_or_time : The relative position to spin the motor to or tha amount of time to spin for
            units (optional) : The units for the provided position or time, the default is DEGREES or MSEC
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None

        #### Examples:
            # spin 180 degrees from the current position\\
            motor1.spin_for(FORWARD, 180)\n
            # spin reverse 2 TURNS (revolutions) from the current position\\
            motor1.spin_for(REVERSE, 2, TURNS)\n
            # spin 180 degrees from the current position at 25 rpm\\
            motor1.spin_for(FORWARD, 180, DEGREES, 25, RPM)\n
            # spin 180 degrees  from the current position and do not wait for completion\\
            motor1.spin_for(FORWARD, 180, DEGREES, False)\n
            # spin 180 degrees  from the current position and do not wait for completion\\
            motor1.spin_for(FORWARD, 180, DEGREES, wait=False)
        '''
        pass

    def is_spinning(self):
        '''### Returns the current status of the spin_to_position or spin_for command
        This function is used when False has been passed as the wait parameter to spin_to_position or spin_for\\
        It will return True if the motor is still spinning or False if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current spin_to_position or spin_for status
        '''
        return True

    def is_done(self):
        '''### Returns the current status of the spin_to_position or spin_for command
        This function is used when False has been passed as the wait parameter to spin_to_position or spin_for\\
        It will return False if the motor is still spinning or True if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current spin_to_position or spin_for status
        '''
        return True

    def is_spinning_mode(self):
        return False

    def stop(self, mode=None):
        '''### Stop the motor, set to 0 velocity and set current stopping_mode
        The motor will be stopped and set to COAST, BRAKE or HOLD

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set_max_torque(self, value, units: TorquePercentCurrentUnits):
        '''### Set the maximum torque the motor will use
        The torque can be set as torque, current or percent of maximum.

        #### Arguments:
            value : the new maximum torque to use
            units : the units that value is passed in

        #### Returns:
            None

        #### Examples:
            # set maximum torque to 2 Nm\\
            motor1.set_max_torque(2, TorqueUnits.NM)\n
            # set maximum torque to 1 Amp\\
            motor1.set_max_torque(1, CurrentUnits.AMP)\n
            # set maximum torque to 20 percent\\
            motor1.set_max_torque(20, PERCENT)
        '''
        pass

    def direction(self):
        '''### Returns the current direction the motor is spinning in

        #### Arguments:
            None

        #### Returns:
            The spin direction, FORWARD, REVERSE or UNDEFINED
        '''
        return DirectionType.FORWARD

    def position(self, *args):
        '''### Returns the position of the motor

        #### Arguments:
            units (optional) : The units for the returned position, the default is DEGREES

        #### Returns:
            The motor position in provided units
        '''
        return 20

    def velocity(self, *args):
        '''### Returns the velocity of the motor

        #### Arguments:
            units (optional) : The units for the returned velocity, the default is RPM

        #### Returns:
            The motor velocity in provided units
        '''
        return 2

    def current(self, *args):
        '''### Returns the current the motor is using

        #### Arguments:
            units (optional) : The units for the returned current, the default is AMP

        #### Returns:
            The motor current in provided units
        '''
        return 1

    def power(self, *args):
        '''### Returns the power the motor is providing

        #### Arguments:
            units (optional) : The units for the returned power, the default is WATT

        #### Returns:
            The motor power in provided units
        '''
        return 1

    def torque(self, *args):
        '''### Returns the torque the motor is providing

        #### Arguments:
            units (optional) : The units for the returned torque, the default is NM

        #### Returns:
            The motor torque in provided units
        '''
        return 1

    def efficiency(self, *args):
        '''### Returns the efficiency of the motor

        #### Arguments:
            units (optional) : The units for the efficiency, the only valid value is PERCENT

        #### Returns:
            The motor efficiency in percent
        '''
        return 1

    def temperature(self, *args):
        '''### Returns the temperature of the motor

        #### Arguments:
            units (optional) : The units for the returned temperature, the default is CELSIUS

        #### Returns:
            The motor temperature in provided units
        '''
        return 1

    def command(self, *args):
        '''### Returns the last velocity sent to the motor

        #### Arguments:
            units (optional) : The units for the returned velocity, the default is RPM

        #### Returns:
            The motor command velocity in provided units
        '''
        return self._velocity

    def get_motor_type(self):
        '''### Returns the type of the motor

        #### Arguments:
            None

        #### Returns:
            The motor type as an integer
        '''
        return 0

# ----------------------------------------------------------

class Motor55:
    '''### Motor55 class - use this to create an instance of a V5 MC55

    #### Arguments:
        port : The smartport this device is attached to
        reverse (optional) : Should the motor's spin direction be reversed, default is False

    #### Returns:
        A new Motor object.

    #### Examples:
        motor1 = Motor(Ports.PORT1)\\
        motor3 = Motor(Ports.PORT3, True)
    '''
    def __init__(self, port: int, *args):
        self._index = port

        self._voltage = 3.0
        self._mode = BrakeType.COAST
        self._brakeMode = BrakeType.COAST
        self._spinMode = False
        self._max_voltage_mv = 8000

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the motor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def set_velocity(self, value: vexnumber, units: PercentUnits=PercentUnits.PERCENT):
        '''### Set default velocity for the motor
        This will be the velocity used for subsequent calls to spin if a velocity is not provided
        to that function.

        #### Arguments:
            value : The new velocity
            units : The units for the supplied velocity, the only valid unit is PERCENT

        #### Returns:
            None
        '''
        self._velocity = value

    def set_reversed(self, value: bool):
        '''### Set the reverse flag for the motor
        Setting the reverse flag will cause spin commands to run the motor in reverse.

        #### Arguments:
            value : Reverse flag, True or False

        #### Returns:
            None
        '''
        pass

    def set_stopping(self, value: BrakeType.BrakeType):
        '''### Set the stopping mode of the motor
        Setting the action for the motor when stopped.

        #### Arguments:
            value : The stopping mode, COAST or BRAKE

        #### Returns:
            None
        '''
        pass

    def spin(self, direction: DirectionType.DirectionType, *args, **kwargs):
        '''### Spin the motor using the provided arguments

        #### Arguments:
            direction : The direction to spin the motor, FORWARD or REVERSE
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units (optional) : The units of the provided velocity, default is VOLT

        #### Returns:
            None

        #### Examples:
            # spin motor forward at velocity set with set_velocity\\
            motor1.spin(FORWARD)\n
            # spin with negative velocity, -2 volts, ie. backwards\\
            motor1.spin(FORWARD, -2)\n
            # spin motor forwards at 3 volts\\
            motor1.spin(FORWARD, 3, VOLT)
        '''
        pass

    def stop(self, mode=None):
        '''### Stop the motor, set to 0 velocity and set current stopping_mode
        The motor will be stopped and set to COAST, BRAKE or HOLD

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set_max_torque(self, value, units = CurrentUnits.AMP):
        '''### Set the maximum torque the motor will use
        The torque can only be set as current.

        #### Arguments:
            value : the new maximum torque to use
            units : the units that value is passed in

        #### Returns:
            None

        #### Examples:
            # set maximum torque to 1 Amp\\
            motor1.set_max_torque(1, CurrentUnits.AMP)
        '''
        pass

    def current(self, *args):
        '''### Returns the current the motor is using

        #### Arguments:
            units (optional) : The units for the returned current, the default is AMP

        #### Returns:
            The motor current in provided units
        '''
        return 1

    def temperature(self, *args):
        '''### Returns the temperature of the motor

        #### Arguments:
            units (optional) : The units for the returned temperature, the default is CELSIUS

        #### Returns:
            The motor temperature in provided units
        '''
        return 1

    def get_motor_type(self):
        '''### Returns the type of the motor

        #### Arguments:
            None

        #### Returns:
            The motor type as an integer
        '''
        return 2

    def get_max_voltage(self):
        '''### Returns the maximum voltage for the motor corresponding to 100%

        #### Arguments:
            None

        #### Returns:
            The motor maximum voltage in mV
        '''
        return 2

# ----------------------------------------------------------

class Thread:
    '''### Thread class - create a new thread of execution

    This class is used to create a new thread using the vexos scheduler.

    #### Arguments:
        callback : A function used as the entry point for the thread
        arg (optional) : A tuple that is used to pass arguments to the thread entry function.

    #### Returns:
        An instance of the Thread class

    #### Examples:
        def foo():
            print('the callback was called')

        t1 = Thread( foo )

        def bar(p1, p2):
            print('the callback was called with ', p1, ' and ', p2)

        t2 = Thread( bar, (1,2) )
    '''
    def __init__(self, callback: Callable[...,None], arg: tuple=()):
        pass

    def stop(self):
        '''### Stop a thread

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    @staticmethod
    def sleep_for(duration: vexnumber, units=TimeUnits.MSEC):
        '''### sleep a thread

        #### Arguments:
            duration : time to sleep this thread for
            units (optional) : units of time, default is MSEC

        #### Returns:
            None
        '''

# ----------------------------------------------------------

class Timer:
    '''### Timer class - create a new timer

    This class is used to create a new timer\\
    A timer can be used to measure time, access the system time and run a function at a time in the future.

    #### Arguments:
        None

    #### Returns:
        An instance of the Timer class

    #### Examples:
        t1 = Timer()
    '''
    def __init__(self):
        pass

    def time(self, units=TimeUnits.MSEC):
        '''### return the current time for this timer

        #### Arguments:
            units (optional) : the units that the time should be returned in, default is MSEC

        #### Returns:
            An the current time in specified units.

        #### Examples:
        '''
        return 0

    def value(self):
        '''### return the current time for this timer in seconds

        #### Arguments:
            None

        #### Returns:
            An the current time in seconds.

        #### Examples:
        '''
        return 0.0

    def clear(self):
        '''### reset the timer to 0

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
        '''
        pass

    def reset(self):
        '''### reset the timer to 0

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
        '''
        pass

    def system(self):
        '''### return the system time in mS

        #### Arguments:
            None

        #### Returns:
            system time in mS

        #### Examples:
        '''
        return 0

    def system_high_res(self):
        '''### return the high resolution system time in uS

        #### Arguments:
            None

        #### Returns:
            system time in uS

        #### Examples:
        '''
        return 0

    def event(self, callback: Callable[...,None], delay: int, arg: tuple=()):
        '''### register a function to be called in the future

        #### Arguments:
            callback : A function that will called after the supplied delay
            delay : The delay before the callback function is called.
            arg (optional) : A tuple that is used to pass arguments to the function.

        #### Returns:
            None

        #### Examples:
            def foo(arg):
                print('timer has expired ', arg)

            t1 = Timer()\\
            t1.event(foo, 1000, ('Hello',))
        '''
        return 0

# ----------------------------------------------------------

class Triport:
    class TriportPort:
        def __init__(self, port, index):
            self._index = port
            self._id = id

        def value(self, *args):
            return 0

        def type(self, *args):
            return 0

        def pressed(self, callback: Callable[...,None], arg: tuple=()):
            return Event(callback, arg)

        def released(self, callback: Callable[...,None], arg: tuple=()):
            return Event(callback, arg)

        def changed(self, callback: Callable[...,None], arg: tuple=()):
            return Event(callback, arg)

    def __init__(self, port):
        self._index = port
        self.a = Triport.TriportPort(port, 0)
        self.b = Triport.TriportPort(port, 1)
        self.c = Triport.TriportPort(port, 2)
        self.d = Triport.TriportPort(port, 3)
        self.e = Triport.TriportPort(port, 4)
        self.f = Triport.TriportPort(port, 5)
        self.g = Triport.TriportPort(port, 6)
        self.h = Triport.TriportPort(port, 7)

    def index(self):
        return self._index

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

# ----------------------------------------------------------

class Limit:
    '''### Limit class - create a new limit switch

    #### Arguments:
        port : The 3wire port the limit switch is connected to

    #### Returns:
        An instance of the Limit class

    #### Examples:
        limit1 = Limit(brain.three_wire_port.a)
    '''
    def __init__(self, port: Triport.TriportPort):
        self._index = port

    def value(self):
        '''### The current value of the limit switch

        #### Arguments:
            None

        #### Returns:
            1 or 0
        '''
        return 0

    def type(self):
        return 0

    def pressed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the limit switch is pressed

        #### Arguments:
            callback : A function that will be called when the limit switch is pressed
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("switch pressed")

            limit1.pressed(foo)
        '''
        return Event(callback, arg)

    def released(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the limit switch is released

        #### Arguments:
            callback : A function that will be called when the limit switch is released
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("switch released")

            limit1.released(foo)
        '''
        return Event(callback, arg)

    def pressing(self):
        '''### Returns whether the limit switch is currently being pressed

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return 0

# ----------------------------------------------------------

class Bumper:
    '''### Bumper class - create a new bumper switch

    #### Arguments:
        port : The 3wire port the bumper switch is connected to

    #### Returns:
        An instance of the Bumper class

    #### Examples:
        bumper1 = Bumper(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self):
        '''### The current value of the bumper switch

        #### Arguments:
            None

        #### Returns:
            1 or 0
        '''
        return 0

    def type(self):
        return 0

    def pressed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the bumper switch is pressed

        #### Arguments:
            callback : A function that will be called when the bumper switch is pressed
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("switch pressed")

            bumper1.pressed(foo)
        '''
        return Event(callback, arg)

    def released(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the bumper switch is released

        #### Arguments:
            callback : A function that will be called when the bumper switch is released
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("switch released")

            bumper1.released(foo)
        '''
        return Event(callback, arg)

    def pressing(self):
        '''### Returns whether the bumper switch is currently being pressed

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return 0

# ----------------------------------------------------------

class DigitalIn:
    '''### DigitalIn class - create a new digital input

    #### Arguments:
        port : The 3wire port to use for the digital input

    #### Returns:
        An instance of the DigitalIn class

    #### Examples:
        dig1 = DigitalIn(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self):
        '''### The current value of the digital input

        #### Arguments:
            None

        #### Returns:
            1 or 0
        '''
        return 0

    def type(self):
        return 0

    def high(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the digital input goes to the logic high state

        #### Arguments:
            callback : A function that will be called when the digital input goes to the logic high state
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("input high")

            dig1.high(foo)
        '''
        return Event(callback, arg)

    def low(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the digital input goes to the logic low state

        #### Arguments:
            callback : A function that will be called when the digital input goes to the logic low state
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("input low")

            dig1.low(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class DigitalOut:
    '''### DigitalOut class - create a new digital output

    #### Arguments:
        port : The 3wire port to use for the digital output

    #### Returns:
        An instance of the DigitalOut class

    #### Examples:
        dig1 = DigitalOut(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self):
        '''### The current value of the digital output

        #### Arguments:
            None

        #### Returns:
            1 or 0
        '''
        return 0

    def type(self):
        return 0

    def set(self, value):
        '''### Set the output level for the digital output

        #### Arguments:
            value : 0, 1, True or False

        #### Returns:
            None

        #### Examples:
            dig1.set(True)
        '''
        pass

# ----------------------------------------------------------

class Led:
    '''### Led class - create a new led

    #### Arguments:
        port : The 3wire port to use for the led

    #### Returns:
        An instance of the Led class

    #### Examples:
        led1 = Led(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self):
        '''### The current value of the led

        #### Arguments:
            None

        #### Returns:
            1 or 0
        '''
        return 0

    def type(self):
        return 0

    def on(self):
        '''### Turn the led on

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            led1.on()
        '''
        pass

    def off(self):
        '''### Turn the led off

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            led1.off()
        '''
        pass

# ----------------------------------------------------------

class Pneumatics:
    '''### Pneumatics class - create a new pneumatics driver class

    #### Arguments:
        port : The 3wire port to use for the pneumatics

    #### Returns:
        An instance of the Pneumatics class

    #### Examples:
        p1 = Pneumatics(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self):
        '''### The current state of the pneumatics driver

        #### Arguments:
            None

        #### Returns:
            1 or 0
        '''
        return 0

    def type(self):
        return 0

    def open(self):
        '''### Set the pneumatics driver to the open state

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            p1.open()
        '''
        pass

    def close(self):
        '''### Set the pneumatics driver to the close state

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            p1.close()
        '''
        pass

# ----------------------------------------------------------

class Potentiometer:
    '''### Potentiometer class - create a new potentiometer

    #### Arguments:
        port : The 3wire port to use for the potentiometer

    #### Returns:
        An instance of the Potentiometer class

    #### Examples:
        pot1 = Potentiometer(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self, units: AnalogPercentUnits = AnalogUnits.TWELVEBIT):
        '''### The current value of the potentiometer

        #### Arguments:
            units (optional) : A valid AnalogUnits type or PERCENT, the default is 12 bit analog

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get potentiometer in range 0 - 4095\\
            value = pot1.value()

            # get potentiometer in range 0 - 1023\\
            value = pot1.value(AnalogUnits.TENBIT)
        '''
        return 0

    def type(self):
        return 0

    def angle(self, units:RotationPercentUnits = RotationUnits.DEG):
        '''### The current angle of the potentiometer

        #### Arguments:
            units (optional) : A valid RotationUnits type or PERCENT, the default is DEGREES

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get potentiometer in range 0 - 250 degrees\\
            angle = pot1.angle()

            # get potentiometer in range 0 - 100%\\
            angle = pot1.angle(PERCENT)
        '''
        return 0

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the potentiometer changes

        #### Arguments:
            callback : A function that will be called when the value of the potentiometer changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("pot changed")

            pot1.changed(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class PotentiometerV2:
    '''### PotentiometerV2 class - create a new potentiometer

    #### Arguments:
        port : The 3wire port to use for the potentiometer

    #### Returns:
        An instance of the PotentiometerV2 class

    #### Examples:
        pot1 = PotentiometerV2(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self, units: AnalogPercentUnits = AnalogUnits.TWELVEBIT):
        '''### The current value of the potentiometer

        #### Arguments:
            units (optional) : A valid AnalogUnits type or PERCENT, the default is 12 bit analog

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get potentiometer in range 0 - 4095\\
            value = pot1.value()

            # get potentiometer in range 0 - 1023\\
            value = pot1.value(AnalogUnits.TENBIT)
        '''
        return 0

    def type(self):
        return 0

    def angle(self, units:RotationPercentUnits = RotationUnits.DEG):
        '''### The current angle of the potentiometer

        #### Arguments:
            units (optional) : A valid RotationUnits type or PERCENT, the default is DEGREES

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get potentiometer in range 0 - 250 degrees\\
            angle = pot1.angle()

            # get potentiometer in range 0 - 100%\\
            angle = pot1.angle(PERCENT)
        '''
        return 0

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the potentiometer changes

        #### Arguments:
            callback : A function that will be called when the value of the potentiometer changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("pot changed")

            pot1.changed(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class Line:
    '''### Line class - create a new line sensor

    #### Arguments:
        port : The 3wire port to use for the line sensor

    #### Returns:
        An instance of the Line class

    #### Examples:
        line1 = Line(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self, units: AnalogPercentUnits = AnalogUnits.TWELVEBIT):
        '''### The current value of the line sensor

        #### Arguments:
            units (optional) : A valid AnalogUnits type or PERCENT, the default is 12 bit analog

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get line sensor in range 0 - 4095\\
            value = line1.value()

            # get line sensor in range 0 - 1023\\
            value = line1.value(AnalogUnits.TENBIT)
        '''
        return 0

    def type(self):
        return 0

    def reflectivity(self, units=PercentUnits.PERCENT):
        '''### The current reflectivity of the line sensor

        The reflectivity of the line sensor is an estimation based on the raw value of the sensor.\\
        A reflectivity of 0% is a raw value of approximated 3000 or greater\\
        A reflectivity of 100% is a raw value of 0

        #### Arguments:
            units (optional) : The only valid value is PERCENT

        #### Returns:
            A value in the range 0 to 100%

        #### Examples:
            # get line sensor reflectivity in range of 0 -100%\\
            value = line1.reflectivity()
        '''
        return 0

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the line sensor changes

        #### Arguments:
            callback : A function that will be called when the value of the line sensor changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("line sensor changed")

            line1.changed(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class ObjectDetector:
    '''### ObjectDetector class - create a new object detector

    #### Arguments:
        port : The 3wire port to use for the object detector
        reverse_det (optional) : If set to True, the detection logic is reversed

    #### Returns:
        An instance of the ObjectDetector class

    #### Examples:
        objdet1 = ObjectDetector(brain.three_wire_port.a)
    '''
    def __init__(self, port, reverse_det=False):
        self._index = port

    def value(self, units: AnalogPercentUnits = AnalogUnits.TWELVEBIT):
        '''### The current value of the object detector

        #### Arguments:
            units (optional) : A valid AnalogUnits type or PERCENT, the default is 12 bit analog

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get the object detector raw value in the range 0 - 4095\\
            value = objdet1.value()

            # get the object detector raw value in the range 0 - 1023\\
            value = objdet1.value(AnalogUnits.TENBIT)
        '''
        return 0

    def type(self):
        return 0

    def reflectivity(self, units=PercentUnits.PERCENT):
        '''### The current reflectivity of the object detector

        The reflectivity of the object detector is an estimation based on the raw value of the sensor.\\
        A reflectivity of 0% is a raw value of approximated 3000 or greater\\
        A reflectivity of 100% is a raw value of 0

        #### Arguments:
            units (optional) : The only valid value is PERCENT

        #### Returns:
            A value in the range 0 to 100%

        #### Examples:
            # get object detector reflectivity in range of 0 -100%\\
            value = objdet1.reflectivity()
        '''
        return 0

    def set_threshold(self, value: vexnumber, units: PercentUnits.PercentUnits = PERCENT):
        '''### set the object detector threshold to the requested value

        #### Arguments:
            value : reflectivity threshold for object detection, 0 to 100%

        #### Returns:
            None

        #### Examples:
            objdet1.set_threshold(50)
        '''
        pass

    def is_object_detected(self):
        '''### Returns if an object is detected

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the object detector changes

        #### Arguments:
            callback : A function that will be called when the value of the object detector changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("object detector value changed")

            objdet1.changed(foo)
        '''
        return Event(callback, arg)

    def object_detected(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the object detector detects an object

        The object detector will trigger this event when the reflectivity passes the value set by set_threshold.\\
        The default operation is to trigger when the reflectivity is increasing past the threshold.

        #### Arguments:
            callback : A function that will be called when the object detector detects an object
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("object detected")

            objdet1.object_detected(foo)
        '''
        return Event(callback, arg)

    def object_lost(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the object detector loses an object

        The object detector will trigger this event when the reflectivity passes the value set by set_threshold.\\
        The default operation is to trigger when the reflectivity is decreasing past the threshold.

        #### Arguments:
            callback : A function that will be called when the object detector loses an object
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("object lost")

            objdet1.object_lost(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class Light:
    '''### Light class - create a new light sensor

    #### Arguments:
        port : The 3wire port to use for the light sensor

    #### Returns:
        An instance of the Light class

    #### Examples:
        light1 = Light(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self, units: AnalogPercentUnits = AnalogUnits.TWELVEBIT):
        '''### The current value of the light sensor

        #### Arguments:
            units (optional) : A valid AnalogUnits type or PERCENT, the default is 12 bit analog

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get light sensor in range 0 - 4095\\
            value = light1.value()

            # get light sensor in range 0 - 1023\\
            value = light1.value(AnalogUnits.TENBIT)
        '''
        return 0

    def type(self):
        return 0

    def brightness(self, units=PercentUnits.PERCENT):
        '''### The current brightness of light falling on the light sensor

        The brightness of the light sensor is an estimation based on the raw value of the sensor.\\
        A brightness of 0% is a raw value of approximated 900 or greater\\
        A brightness of 100% is a raw value of 0

        #### Arguments:
            units (optional) : The only valid value is PERCENT

        #### Returns:
            A value in the range 0 to 100%

        #### Examples:
            # get light sensor brightness in range of 0 -100%\\
            value = light1.brightness()
        '''
        return 0

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the light sensor changes

        #### Arguments:
            callback : A function that will be called when the value of the light sensor changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("light sensor changed")

            light1.changed(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class Gyro:
    '''### Gyro class - create a new gyro sensor

    #### Arguments:
        port : The 3wire port to use for the gyro sensor

    #### Returns:
        An instance of the Gyro class

    #### Examples:
        gyro1 = Gyro(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port
        self.calsim = 0

    def value(self, units: RotationPercentUnits = DEGREES):
        '''### The current value of the gyro

        This method is generally not used, see heading() and rotation()

        #### Arguments:
            units (optional) : A valid RotationUnits type or PERCENT, the default is DEGREES

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get gyro value in range 0 - 360 degrees\\
            value = gyro1.value()
        '''
        return 0

    def type(self):
        return 0

    def calibrate(self):
        '''### Start calibration of the gyro

        Calibration should done when the gyro is not moving.

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            # start calibration\\
            gyro1.calibrate()\\
            # wait for completion\\
            while gyro1.is_calibrating():\\
                sleep(50, MSEC)
        '''
        self.calsim = 0

    def is_calibrating(self):
        '''### check the calibration status of the gyro

        Calibration should done when the gyro is not moving.

        #### Arguments:
            None

        #### Returns:
            True when the gyro is calibrating

        #### Examples:
            # start calibration\\
            gyro1.calibrate()\\
            # wait for completion\\
            while gyro1.is_calibrating():\\
                sleep(50, MSEC)
        '''
        self.calsim += 1
        if self.calsim < 3:
            return True
        else:
            return False

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the gyro heading changes

        #### Arguments:
            callback : A function that will be called when the value of the gyro heading changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("gyro changed")

            gyro1.changed(foo)
        '''
        return Event(callback, arg)

    def reset_heading(self):
        '''### Reset the gyro heading to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def reset_rotation(self):
        '''### Reset the gyro rotation to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set_heading(self, value: vexnumber, units=RotationUnits.DEG):
        '''### set the gyro heading to a new value

        The new value for heading should be in the range 0 - 359.99 degrees.

        #### Arguments:
            value : The new value to use for heading.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of heading to 180 degrees\\
            gyro1.set_heading(180)
        '''
        pass

    def heading(self, units=RotationUnits.DEG):
        '''### read the current heading of the gyro

        heading will be returned in the range 0 - 359.99 degrees

        #### Arguments:
            units (optional) : The units to return the heading in, default is DEGREES

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current heading for the gyro\\
            value = gyro1.heading()
        '''
        return 20

    def set_rotation(self, value, units=RotationUnits.DEG):
        '''### set the gyro rotation to a new value

        #### Arguments:
            value : The new value to use for rotation.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of rotation to 180 degrees\\
            gyro1.set_rotation(180)
        '''
        pass

    def rotation(self, units=RotationUnits.DEG):
        '''### read the current rotation of the gyro

        rotation is not limited, it can be both positive and negative and shows the absolute angle of the gyro.

        #### Arguments:
            units (optional) : The units to return the rotation in, default is DEGREES

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current rotation for the gyro\\
            value = gyro1.rotation()
        '''
        return 20

    def set_turn_type(self, turntype: TurnType.TurnType):
        '''### set the direction that returns positive values for heading

        An advanced function that is not generally used.

        #### Arguments:
            turntype : TurnType.LEFT or TurnType.RIGHT

        #### Returns:
            None
        '''
        pass

    def get_turn_type(self):
        '''### get the direction that returns positive values for heading

        An advanced function that is not generally used.

        #### Arguments:
            None

        #### Returns:
            The current TurnType, LEFT or RIGHT
        '''
        return TurnType.RIGHT

# ----------------------------------------------------------

class Accelerometer:
    '''### Accelerometer class - create a new accelerometer

    For full functionality, three Accelerometer instances would need to be created, one for each axis.

    #### Arguments:
        port : The 3wire port to use for the accelerometer
        sensitivity (optional) : set high sensitivity mode (+/- 2G), use True or 1

    #### Returns:
        An instance of the Accelerometer class

    #### Examples:
        accx = Accelerometer(brain.three_wire_port.a)\\
        accy = Accelerometer(brain.three_wire_port.b)\\
        accz = Accelerometer(brain.three_wire_port.c)
    '''
    def __init__(self, port, sensitivity=0):
        self._index = port

    def value(self, units: AnalogPercentUnits = AnalogUnits.TWELVEBIT):
        '''### The current value of the accelerometer

        #### Arguments:
            units (optional) : A valid AnalogUnits type or PERCENT, the default is 12 bit analog

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get accelerometer in range 0 - 4095\\
            value = accz.value()

            # get accelerometer in range 0 - 1023\\
            value = accz.value(AnalogUnits.TENBIT)
        '''
        return 0

    def type(self):
        return 0

    def acceleration(self):
        '''### The current value of the accelerometer scaled to units of gravity

        #### Arguments:
            None

        #### Returns:
            A value in the range +/- 6 or +/-2G if high sensitivity mode is set

        #### Examples:
            # get accelerometer in range+/- 6G
            value = accz.acceleration()
        '''
        return 1.0

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the accelerometer changes

        #### Arguments:
            callback : A function that will be called when the value of the accelerometer changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("accelerometer changed")

            accz.changed(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class AnalogIn:
    '''### AnalogIn class - create a new analog input

    #### Arguments:
        port : The 3wire port to use for the analog input

    #### Returns:
        An instance of the AnalogIn class

    #### Examples:
        ana1 = AnalogIn(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self, units: AnalogPercentUnits = AnalogUnits.TWELVEBIT):
        '''### The current value of the analog input

        #### Arguments:
            units (optional) : A valid AnalogUnits type or PERCENT, the default is 12 bit analog

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get analog input in range 0 - 4095\\
            value = ana1.value()

            # get analog input in range 0 - 1023\\
            value = ana1.value(AnalogUnits.TENBIT)
        '''
        return 0

    def type(self):
        return 0

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the analog input changes

        #### Arguments:
            callback : A function that will be called when the value of the analog input changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("analog input changed")

            ana1.changed(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class Encoder:
    '''### Encoder class - create a new encoder sensor

    An encoder uses two adjacent 3wire ports.\\
    valid port pairs are a/b, c/d, e/f and g/h

    #### Arguments:
        port : The 3wire port to use for the encoder sensor

    #### Returns:
        An instance of the Encoder class

    #### Examples:
        enc1 = Encoder(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port
        self.calsim = 0

    def value(self):
        '''### The current value of the encoder in raw counts

        One full turn of the encoder is 360 counts.

        #### Arguments:
            None

        #### Returns:
            A value for encoder counts.

        #### Examples:
            # get encoder raw counts\\
            value = enc1.value()
        '''
        return 0

    def type(self):
        return 0

    def reset_position(self):
        '''### Reset the encoder position to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set_position(self, value, units=RotationUnits.DEG):
        '''### set the encoder position to a new value

        #### Arguments:
            value : The new value to use for position.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of position to 180 degrees\\
            enc1.set_position(180)
        '''
        pass

    def position(self, units=RotationUnits.DEG):
        '''### The current position of the encoder

        #### Arguments:
            units (optional) : The rotation units to return the position value in, default is DEGREES.

        #### Returns:
            A value for encoder position in the specified units.

        #### Examples:
            # get encoder position\\
            value = enc1.position()
        '''
        return 20

    def velocity(self, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### The current velocity of the encoder

        #### Arguments:
            units (optional) : The velocity units to return the value in, default is RPM.

        #### Returns:
            A value for encoder velocity in the specified units.

        #### Examples:
            # get encoder velocity in rpm\\
            value = enc1.velocity()
        '''
        return 20

# ----------------------------------------------------------

class Sonar:
    '''### Sonar class - create a new sonar (ultrasonic) sensor

    A sonar uses two adjacent 3wire ports.\\
    valid port pairs are a/b, c/d, e/f and g/h\\
    connect the wire labeled "output" to the lower 3wire port, eg. a

    #### Arguments:
        port : The 3wire port to use for the sonar sensor

    #### Returns:
        An instance of the Sonar class

    #### Examples:
        sonar1 = Sonar(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self, units: AnalogPercentUnits = AnalogUnits.TWELVEBIT):
        '''### The current value of the sonar

        This method has no practical use, see distance.

        #### Arguments:
            units (optional) : A valid AnalogUnits type or PERCENT, the default is 12 bit analog

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get sonar raw value\\
            value = sonar1.value()
        '''
        return 0

    def type(self):
        return 0

    def distance(self, units: DistanceUnits.DistanceUnits):
        '''### The current distance the sonar is detecting an object at.

        The sonar will return a large positive number if no object is detected in range.

        #### Arguments:
            units : The distance units to return the position value in.

        #### Returns:
            A value for sonar distance in the specified units.

        #### Examples:
            # get sonar distance in mm\\
            value = sonar1.distance(MM)
        '''
        return 20

    def found_object(self):
        '''### Check for an object in the range 0 - 1000mm

        The sonar will return True if an object is detected closer than 1000mm.

        #### Arguments:
            None

        #### Returns:
            True of an object is detected.

        #### Examples:
            # is an object closer than 1000mm\\
            if sonar1.found_object():\\
                print("object found")
        '''
        return False

# ----------------------------------------------------------

class Pwm:
    '''### Pwm class - create a new pwm output

    The pwm class will create raw RC style pwm waveform.\\
    A pwm output of 0% corresponds to pulse width of 1.5mS every 16mS\\
    A pwm output of 100% corresponds to pulse width of 2mS\\
    A pwm output of -100% corresponds to pulse width of 1mS

    #### Arguments:
        port : The 3wire port to use for the pwm output

    #### Returns:
        An instance of the Pwm class

    #### Examples:
        pwm1 = Pwm(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self):
        '''### Read the current PWM value in percent.

        #### Arguments:
            None

        #### Returns:
            A value in the range -100 to +100 percent.

        #### Examples:
            # get pwm1 current value\\
            value = pwm1.value()
        '''
        return 0

    def type(self):
        return 0

    def state(self, value, units=PercentUnits.PERCENT):
        '''### Set the current PWM value in percent.

        #### Arguments:
            value : The new value for pwm output, -100 to +100 percent.
            units (optional) : units must be specified in PERCENT

        #### Returns:
            None

        #### Examples:
            # set pwm1 output to 50%\\
            pwm1.state(50)
        '''
        pass

# ----------------------------------------------------------

class Servo:
    '''### Servo class - create a new servo output

    The Servo class will create raw RC style pwm waveform.\\
    An output of 0 corresponds to pulse width of 1.5mS every 16mS\\
    An output of 50 degrees corresponds to pulse width of 2mS\\
    An output of -50 degrees corresponds to pulse width of 1mS

    #### Arguments:
        port : The 3wire port to use for the servo output

    #### Returns:
        An instance of the Servo class

    #### Examples:
        servo1 = Servo(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self):
        '''### Read the current raw servo pwm value.

        This is the raw internal pwm value\\
        A servo position of 0 will return 127\\
        A maximum positive servo position will return 255

        #### Arguments:
            None

        #### Returns:
            A value in the range 0 to 255.

        #### Examples:
            # get servo1 current value\\
            value = servo1.value()
        '''
        return 0

    def type(self):
        return 0

    def set_position(self, value, units: RotationPercentUnits = RotationUnits.DEG):
        '''### Set the servo position

        #### Arguments:
            value : The new value for the servo using the supplied units.
            units (optional) : The rotation units, default is PERCENT

        #### Returns:
            None

        #### Examples:
            # set servo output to 10 degrees\\
            servo1.set_position(10, DEGREES)
        '''
        pass

# ----------------------------------------------------------

class Motor29:
    '''### Motor29 class - create a new pwm motor output

    The Motor29 class will create raw RC style pwm waveform.\\
    This is primarily for use with the VEX MC29 motor controller\\
    To minimize current draw, new values sent to the motor will have slew rate control applied

    #### Arguments:
        port : The 3wire port to use for the motor controller
        reverse_flag (optional) : set reverse flag for this motor, spin commands will cause opposite rotation if set True.  default is False.

    #### Returns:
        An instance of the Motor29 class

    #### Examples:
        motor1 = Motor29(brain.three_wire_port.a)
    '''
    def __init__(self, port, reverse_flag=False):
        self._index = port
        self._velocity = 50

    def value(self):
        '''### Read the current raw motor controller pwm value.

        This is the raw internal pwm value\\
        A motor velocity of 0 will return 127\\
        A maximum positive motor velocity will return 255

        #### Arguments:
            None

        #### Returns:
            A value in the range 0 to 255.

        #### Examples:
            # get motor current pwm value\\
            value = motor1.value()
        '''
        return 0

    def type(self):
        return 0

    def set_velocity(self, value, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### Set default velocity for the motor
        This will be the velocity used for subsequent calls to spin of a velocity is not provided
        to that function.

        #### Arguments:
            value : The new velocity
            units : The units for the supplied velocity, the default is RPM

        #### Returns:
            None
        '''
        self._velocity = value

    def set_reversed(self, value):
        '''### Set the reversed flag for the motor

        #### Arguments:
            value : 1, 0, True or False

        #### Returns:
            None

        #### Examples:
            # set motor reversed flag True\\
            motor1.set_reversed(True)
        '''
        pass

    def spin(self, direction: DirectionType.DirectionType, velocity=None, units=None):
        '''### Spin the motor using the provided arguments

        The motor is assumed to have a maximum velocity of 100 rpm.

        #### Arguments:
            direction : The direction to spin the motor, FORWARD or REVERSE
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units (optional) : The units of the provided velocity, default is RPM

        #### Returns:
            None

        #### Examples:
            # spin motor forward at velocity set with set_velocity\\
            motor1.spin(FORWARD)\n
            # spin motor forward at 50 rpm\\
            motor1.spin(FORWARD, 50)\n
            # spin with negative velocity, ie. backwards\\
            motor1.spin(FORWARD, -20)\n
            # spin motor forwards with 100% velocity\\
            motor1.spin(FORWARD, 100, PERCENT)\n
            # spin motor forwards at 50 rpm\\
            motor1.spin(FORWARD, 50, RPM)\n
            # spin motor forwards at 360 dps\\
            motor1.spin(FORWARD, 360.0, VelocityUnits.DPS)
        '''
        pass

    def stop(self):
        '''### Stop the  motor, set to 0 velocity

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------

class MotorVictor:
    '''### MotorVictor class - create a new pwm motor output

    The MotorVictor class will create raw RC style pwm waveform.\\
    This is primarily for use with the VEX Victor motor controller\\

    #### Arguments:
        port : The 3wire port to use for the motor controller
        reverse_flag (optional) : set reverse flag for this motor, spin commands will cause opposite rotation if set True.  default is False.

    #### Returns:
        An instance of the MotorVictor class

    #### Examples:
        motor1 = MotorVictor(brain.three_wire_port.a)
    '''
    def __init__(self, port, reverse_flag=False):
        self._index = port
        self._velocity = 50

    def value(self):
        '''### Read the current raw motor controller pwm value.

        This is the raw internal pwm value\\
        A motor velocity of 0 will return 127\\
        A maximum positive motor velocity will return 255

        #### Arguments:
            None

        #### Returns:
            A value in the range 0 to 255.

        #### Examples:
            # get motor current pwm value\\
            value = motor1.value()
        '''
        return 0

    def type(self):
        return 0

    def set_velocity(self, value, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### Set default velocity for the motor
        This will be the velocity used for subsequent calls to spin of a velocity is not provided
        to that function.

        #### Arguments:
            value : The new velocity
            units : The units for the supplied velocity, the default is RPM

        #### Returns:
            None
        '''
        self._velocity = value

    def set_reversed(self, value):
        '''### Set the reversed flag for the motor

        #### Arguments:
            value : 1, 0, True or False

        #### Returns:
            None

        #### Examples:
            # set motor reversed flag True\\
            motor1.set_reversed(True)
        '''
        pass

    def spin(self, direction, velocity=None, units=None):
        '''### Spin the motor using the provided arguments

        The motor is assumed to have a maximum velocity of 100 rpm.

        #### Arguments:
            direction : The direction to spin the motor, FORWARD or REVERSE
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units (optional) : The units of the provided velocity, default is RPM

        #### Returns:
            None

        #### Examples:
            # spin motor forward at velocity set with set_velocity\\
            motor1.spin(FORWARD)\n
            # spin motor forward at 50 rpm\\
            motor1.spin(FORWARD, 50)\n
            # spin with negative velocity, ie. backwards\\
            motor1.spin(FORWARD, -20)\n
            # spin motor forwards with 100% velocity\\
            motor1.spin(FORWARD, 100, PERCENT)\n
            # spin motor forwards at 50 rpm\\
            motor1.spin(FORWARD, 50, RPM)\n
            # spin motor forwards at 360 dps\\
            motor1.spin(FORWARD, 360.0, VelocityUnits.DPS)
        '''
        pass

    def stop(self):
        '''### Stop the  motor, set to 0 velocity

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------

class Vision:
    '''### Vision class - a class for working with the vision sensor

    #### Arguments:
        port : The smartport this device is attached to
        brightness (optional) : set the brightness value for the vision sensor
        sigs (optional) : one or more signature objects

    #### Returns:
        An instance of the Vision class

    #### Examples:
        SIG_1 = Signature(1, 6035, 7111, 6572, -1345, -475, -910, 3.000, 0)\\
        vision1 = Vision(Ports.PORT1, 50, SIG_1)
    '''
    def __init__(self, port, *args):
        self._index = port
        self.largestObject = VisionObject()

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the vision sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def take_snapshot(self, index, count=8):
        '''### Request the vision sensor to filter latest objects to match signature or code

        #### Arguments:
            index : A signature, code or signature id.
            count (optional) : the maximum number of objects to obtain.  default is 8.

        #### Returns:
            tuple of VisionObject or None if nothing is available

        #### Examples:
            # look for and return 1 object matching SIG_1\\
            objects = vision1.take_snapshot(SIG_1)

            # look for and return a maximum of 4 objects matching SIG_1\\
            objects = vision1.take_snapshot(SIG_1, 4)
        '''
        return (VisionObject(), )

    def largest_object(self):
        return self.largestObject

    def object_count(self):
        return 0

class VisionObject:
    '''A vision object, not instantiated by user programs'''
    def __init__(self):
        self.id = 0
        self.originX = 0
        self.originY = 0
        self.centerX = 0
        self.centerY = 0
        self.width = 0
        self.height = 0
        self.exists = True
        self.angle = 0
        pass


class Signature:
    '''### Signature class - a class for holding vision sensor signatures

    #### Arguments:
        index : The signature index
        p0 : signature value p0
        p1 : signature value p1
        p2 : signature value p2
        p3 : signature value p3
        p4 : signature value p4
        p5 : signature value p5
        sigrange : signature range
        sigtype : signature type

    #### Returns:
        An instance of the Signature class

    #### Examples:
        SIG_1 = Signature(1, 6035, 7111, 6572, -1345, -475, -910, 3.000, 0)\\
        vision1 = Vision(Ports.PORT1, 50, SIG_1)
    '''
    def __init__(self, index, p0, p1, p2, p3, p4, p5, sigrange, sigtype):
        pass

    def id(self):
        '''Not used, always returns 0'''
        return 0


class Code:
    '''### Code class - a class for holding vision sensor codes

    A vision code is a collection of up to five vision signatures.
    #### Arguments:
        sig1 : A vision signature
        sig2 : A vision signature
        sig3 (optional) : A vision signature
        sig4 (optional) : A vision signature
        sig5 (optional) : A vision signature

    #### Returns:
        An instance of the Code class

    #### Examples:
        SIG_1 = Signature(1, 6035, 7111, 6572, -1345, -475, -910, 3.000, 0)\\
        SIG_2 = Signature(2, 6035, 7111, 6572, -1345, -475, -910, 3.000, 0)\\
        C1 = Code(SIG_1, SIG_2)
    '''
    def __init__(self, c1:Signature, *args):
        pass

    def id(self):
        '''Not used, always returns 0'''
        return 0

# ----------------------------------------------------------

class Colordesc:
    '''### Colordesc class - a class for holding an AI vision sensor color definition

    #### Arguments:
        index : The color description index (1 to 7)
        red : the red color value
        green : the green color value
        blue : the blue color value
        hangle : the range of allowable hue
        hdsat : the range of allowable saturation

    #### Returns:
        An instance of the Colordesc class

    #### Examples:
        COL1 = Colordesc(1,  13, 114, 227, 10.00, 0.20)\\
        COL2 = Colordesc(2, 237,  61,  74, 10.00, 0.20)\\
    '''
    def __init__(self, index, red, green, blue, hangle, hdsat):
        self.id = index
        pass

class Codedesc:
    '''### Codedesc class - a class for holding AI vision sensor codes

    A code description is a collection of up to five AI vision color descriptions.
    #### Arguments:
        index : The code description index (1 to 5)
        c1 : An AI vision Colordesc
        c1 : An AI vision Colordesc
        c3 (optional) : An AI vision Colordesc
        c4 (optional) : An AI vision Colordesc
        c5 (optional) : An AI vision Colordesc

    #### Returns:
        An instance of the Codedesc class

    #### Examples:
        COL1 = Colordesc(1,  13, 114, 227, 10.00, 0.20)\\
        COL2 = Colordesc(2, 237,  61,  74, 10.00, 0.20)\\
        C1 = Codedesc( 1, COL1, COL2 )
    '''
    def __init__(self, index, c1:Colordesc, c2:Colordesc, *args):
        self.id = index
        pass

class Tagdesc:
    '''### Tagdesc class - a class for holding AI vision sensor tag id

    A tag description holds an apriltag id
    #### Arguments:
        id : The apriltag id (positive integer, not 0)

    #### Returns:
        An instance of the Tagdesc class

    #### Examples:
        T1 = Tagdesc( 23 )
    '''
    def __init__(self, index):
        self.id = index
        pass

class AiObjdesc:
    '''### AiObjdesc class - a class for holding AI vision sensor AI object id

    A tag description holds an apriltag id
    #### Arguments:
        id : The AI Object (model) id (positive integer, not 0)

    #### Returns:
        An instance of the AiObjdesc class

    #### Examples:
        A1 = AiObjdesc( 2 )
    '''
    def __init__(self, index):
        self.id = index
        pass

class AiVision:
    '''### AI Vision class - a class for working with the AI Vision sensor

    #### Arguments:
        port : The smartport this device is attached to
        colors (optional) : one or more colordesc objects
        codes (optional) : one or more codedesc objects

    #### Returns:
        An instance of the AiVision class

    #### Examples:
        COL1 = Colordesc(1,      13,     114,     227,   10.00,    0.20)\\
        COL2 = Colordesc(2,     237,      61,      74,   10.00,    0.20)\\
        aivision1 = AiVision(Ports.PORT1, COL1, COL2)
    '''

    COLOR_OBJECT = 1
    '''Object type indicating the object is a color object'''
    CODE_OBJECT = 2
    '''Object type indicating the object is a code object'''
    AI_OBJECT = 4
    '''Object type indicating the object is a AI model object'''
    TAG_OBJECT = 8
    '''Object type indicating the object is a apriltag object'''

    ALL_TAGS = Tagdesc(0xFFFF)
    '''A tag description for take_snapshot indicating all tag objects to be returned'''
    ALL_COLORS = Colordesc(0xFFFF, 0, 0, 0, 0, 0)
    '''A tag description for take_snapshot indicating all color objects to be returned'''
    ALL_CODES = Codedesc(0xFFFF, ALL_COLORS, ALL_COLORS)
    '''A tag description for take_snapshot indicating all code objects to be returned'''
    ALL_AIOBJS = AiObjdesc(0xFFFF)
    '''A tag description for take_snapshot indicating all AI model objects to be returned'''
    ALL_OBJECTS = 0xFFFF
    '''A tag description for take_snapshot indicating all objects to be returned'''

    TAG_CIRCLE21H7 = 0
    '''Apriltag fimily circle21h7, used with set_tag_family'''
    TAG_16H5 = 1
    '''Apriltag fimily 16h5, used with set_tag_family'''
    TAG_25H9 = 2
    '''Apriltag fimily 25h9, used with set_tag_family'''
    TAG_36H11 = 3
    '''Apriltag fimily 36h11, used with set_tag_family'''

    FLG_COLORMERGE = 0x10
    '''A flag definition to enable color box merging'''
    FLG_OVLENABLE = 0x80
    '''A flag definition to enable the usb overlay'''

    def __init__(self, port, *args):
        self._index = port
        self.largestObject = AiVisionObject()

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the AI vision sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def take_snapshot(self, index, count=8):
        '''### Request the AI vision sensor to filter latest objects to match color,\\
               code, apriltag or AI object.

        #### Arguments:
            type : A color, code or other object type
            count (optional) : the maximum number of objects to obtain.  default is 8.

        #### Returns:
            tuple of AiVisionObject, this will be an empty tuple if nothing is available.

        #### Examples:
            # look for and return 1 object matching COL1\\
            objects = aivision1.take_snapshot(COL1)

            # look for and return a maximum of 4 objects matching SIG_1\\
            objects = aivision1.take_snapshot(COL1, 4)

            # return apriltag objects\\
            objects = aivision1.take_snapshot(ALL_TAGS, AIVISION_MAX_OBJECTS)
        '''
        return (AiVisionObject(), )

    def largest_object(self):
        '''### Request the largest object

        #### Arguments:
            None

        #### Returns:
            An AiVisionObject object or None if it does not exist
        '''
        return self.largestObject

    def object_count(self):
        '''### Request the number of objects found in the last take_snapshot call

        #### Arguments:
            None

        #### Returns:
            The number of objects found in the last take_snapshot call
        '''
        return 0

    def tag_detection(self, enable: bool):
        '''### Enable or disable apriltag processing

        #### Arguments:
            enable : True or False

        #### Returns:
            None
        '''
        pass

    def color_detection(self, enable: bool, merge: bool = False):
        '''### Enable or disable color and code object processing

        #### Arguments:
            enable : True or False
            merge (optional) : True to enable merging of adjacent color detections

        #### Returns:
            None
        '''
        pass

    def model_detection(self, enable: bool):
        '''### Enable or disable AI model object processing

        #### Arguments:
            enable : True or False

        #### Returns:
            None
        '''
        pass

    def debug_get(self, *args):
        '''### development use

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set_tag_family(self, f: int ):
        '''### set the apriltag family

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set(self, desc: Union[Colordesc, Codedesc] ):
        '''### set a new color or code description

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def start_awb(self):
        '''### Run auto white balance

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def status_overlay(self, enable: bool):
        '''### Enable or disable status on USB output

        #### Arguments:
            enable : True or False

        #### Returns:
            None
        '''
        pass

    def usb_overlay(self, enable: bool):
        '''### Enable or disable the USB overlay of objects

        #### Arguments:
            enable : True or False

        #### Returns:
            None
        '''
        pass

class AiVisionObject:
    '''A, AI vision object, not instantiated by user programs'''
    class Tag:
        def __init__(self):
            self.x = (0,0,0,0)
            self.y = (0,0,0,0)
            pass

    def __init__(self):
        self.id = 0
        self.type = 0
        self.originX = 0
        self.originY = 0
        self.centerX = 0
        self.centerY = 0
        self.width = 0
        self.height = 0
        self.exists = True
        self.angle = 0
        self.score = 0
        self.area = 0
        self.classname = ''
        self.color = Color.RED
        self.tag = AiVisionObject.Tag()
        pass

# ----------------------------------------------------------

class MessageLink:
    '''### MessageLink class - a class for communicating using VEXlink

    #### Arguments:
        port : The smartport the VEXlink radio is attached to
        name : The name of this link
        linktype : The type of this link, either VexlinkType.MANAGER or VexlinkType.WORKER
        wired (optional) : Set to True if this is a wired link

    #### Returns:
        An instance of the MessageLink class

    #### Examples:
        link = MessageLink(Ports.PORT1, 'james', VexlinkType.MANAGER)
    '''
    def __init__(self, port, name: str, linktype: VexlinkType.VexlinkType, wired=False):
        self._index = port

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def is_linked(self):
        '''### Return link status

        #### Arguments:
            None

        #### Returns:
            True if the link is active and connected to the paired brain.
        '''
        return True

    def send(self, message: str, *args):
        '''### Send a message with optional parameters

        #### Arguments:
            message : A string, the message to send
            index (optional) : A int such as port number
            value (optional) : A float

        #### Returns:
            length of transmitted data or None on error

        #### Examples:
            # send the message 'test' with no parameters\\
            link.send('test')

            # send the message 'test' with parameters\\
            link.send('test', 1, 3.14)
        '''
        return len(message)

    def receive(self, timeout=300000):
        '''### Receive the next message

        #### Arguments:
            timeout (optional) : An optional timeout value in mS before the function returns.

        #### Returns:
            None or received message

        #### Examples:
            message = link.receive()
        '''
        return 'test'

    def received(self, *args):
        '''### Register a function to be called when a message is received

        If the message is omitted then the callback will be called for all messages.

        #### Arguments:
            message (optional) : A message name for which the callback will be called
            callback : A function that will be called when a message is received

        #### Returns:
            None

        #### Examples:
            def cb(message, link, index, value):
                print(link, message, index, value)

            link.received('test', cb)
        '''
        pass

# ----------------------------------------------------------

class SerialLink:
    '''### SerialLink class - a class for communicating using VEXlink

    #### Arguments:
        port : The smartport the VEXlink radio is attached to
        name : The name of this link
        linktype : The type of this link, either VexlinkType.MANAGER or VexlinkType.WORKER
        wired (optional) : Set to True if this is a wired link

    #### Returns:
        An instance of the SerialLink class

    #### Examples:
        link = SerialLink(Ports.PORT1, 'james', VexlinkType.MANAGER)
    '''
    def __init__(self, port, name: str, linktype: VexlinkType.VexlinkType, wired=False):
        self._index = port

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def is_linked(self):
        '''### Return link status

        #### Arguments:
            None

        #### Returns:
            True if the link is active and connected to the paired brain.
        '''
        return True

    def send(self, buffer):
        '''### Send a buffer of length length

        #### Arguments:
            buffer : A string or bytearray, the message to send

        #### Returns:
            None

        #### Examples:
            # send the string 'test'\\
            link.send('test')

            # send the bytearray 'test' with parameters\\
            link.send('test', 1, 3.14)
        '''
        return 0

    def receive(self, length, timeout=300000):
        '''### Receive data in the serial link

        #### Arguments:
            length : maximum amount of data to wait for
            timeout (optional) : An optional timeout value in mS before the function returns.

        #### Returns:
            None or bytearray with data

        #### Examples:
            # wait for 128 bytes of data for 1000mS\\
            buffer = link.receive(128, 1000)
        '''
        return bytearray([1, 2, 3, 4])

    def received(self, callback: Callable[...,None]):
        '''### Register a function to be called when data is received

        This will receive a bytearray and a length indicating how much

        #### Arguments:
            callback : A function that will be called when data is received

        #### Returns:
            None

        #### Examples:
            def cb(buffer, length):
                print(buffer, length)

            link.received(cb)
        '''
        pass

# ----------------------------------------------------------

class Rotation:
    '''### Rotation class - a class for working with the rotation sensor

    #### Arguments:
        port : The smartport this device is attached to
        reverse (optional) : set to reverse the angle and position returned by the sensor.

    #### Returns:
        An instance of the Rotation class

    #### Examples:
        rot1 = Rotation(Ports.PORT1)\\
        rot2 = Rotation(Ports.PORT2, True)
    '''
    def __init__(self, port, reverse=False):
        self._index = port
        self._reversed = reverse

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def set_reversed(self, value):
        '''### Set the reversed flag for the sensor

        Usually this would be done in the constructor.

        #### Arguments:
            value : 1, 0, True or False

        #### Returns:
            None

        #### Examples:
            # set reversed flag True\\
            rot1.set_reversed(True)
        '''
        self._reversed = value

    def angle(self, units=RotationUnits.DEG):
        '''### The current angle of the rotation sensor

        #### Arguments:
            units (optional) : A valid RotationUnits type, the default is DEGREES

        #### Returns:
            A value in the range that is specified by the units.

        #### Examples:
            # get rotation sensor angle\
            angle = rot1.angle()
        '''
        return 0

    def reset_position(self):
        '''### Reset the rotation sensor position to 0

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set_position(self, value, units=RotationUnits.DEG):
        '''### Set the current position of the rotation sensor
        The position returned by the position() function is set to this value.

        The position is an absolute value that continues to increase or decrease as the\\
        sensor is rotated.

        #### Arguments:
            value : The new position
            units : The units for the provided position, the default is DEGREES

        #### Returns:
            None
        '''
        pass

    def position(self, units=RotationUnits.DEG):
        '''### Returns the position of the rotation sensor

        The position is an absolute value that continues to increase or decrease as the\\
        sensor is rotated.

        #### Arguments:
            units (optional) : The units for the returned position, the default is DEGREES

        #### Returns:
            The rotation sensor in provided units
        '''
        return 20

    def velocity(self, units=VelocityUnits.RPM):
        '''### Returns the velocity of the rotation sensor

        #### Arguments:
            units (optional) : The units for the returned velocity, the default is RPM

        #### Returns:
            The rotation sensor velocity in provided units
        '''
        return 20

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the value of the rotation sensor changes

        #### Arguments:
            callback : A function that will be called when the value of the rotation sensor changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("rotation changed")

            rot1.changed(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class Optical:
    '''### Optical class - a class for working with the optical sensor

    #### Arguments:
        port : The smartport this device is attached to

    #### Returns:
        An instance of the Optical class

    #### Examples:
        opt1 = Optical(Ports.PORT1)
    '''
    class Gesture:
        def __init__(self):
            self.type = GestureType.NONE
            self.udata = 0
            self.ddata = 0
            self.ldata = 0
            self.rdata = 0
            self.count = 0

    def __init__(self, port):
        self._index = port

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def hue(self):
        '''### read the hue value from the optical sensor

        #### Arguments:
            None

        #### Returns:
            hue as a float in the range 0 - 359.99 degrees

        #### Examples:
            hue = opt1.hue()
        '''
        return 0

    def brightness(self, readraw=False):
        '''### read the brightness value from the optical sensor

        #### Arguments:
            readraw (optional) : return raw brightness value if True rather than percentage.

        #### Returns:
            brightness as a float in the range 0 - 100%

        #### Examples:
            brightness = opt1.brightness()
        '''
        return 0

    def color(self):
        '''### read the color from the optical sensor

        #### Arguments:
            None

        #### Returns:
            color as an instance of the Color class

        #### Examples:
            c = opt1.color()
        '''
        return Color.RED

    def is_near_object(self):
        '''### check to see if the optical proximity sensor detects an object

        #### Arguments:
            None

        #### Returns:
            True if near an object

        #### Examples:
            if opt1.is_near_object():
                print('near object')
        '''
        return True

    def set_light(self, *args):
        '''### set optical sensor led on or of

        #### Arguments:
            value : LedStateType.ON, LedStateType.OFF or power of led, 0 to 100%

        #### Returns:
            None

        #### Examples:
            # turn on led with previous intensity\\
            opt1.set_light(LedStateType.ON)

            # turn on led with new intensity\\
            opt1.set_light(65)
        '''
        pass

    def set_light_power(self, value: vexnumber, units: PercentUnits.PercentUnits = PERCENT):
        '''### set optical sensor led to the requested power

        #### Arguments:
            value : power of led, 0 to 100%

        #### Returns:
            None

        #### Examples:
            opt1.set_light_power(50)
        '''
        pass

    def integration_time(self, value: vexnumber=-1):
        '''### set optical sensor led to the requested power

        #### Arguments:
            value (optional) : integration time in mS (5 to 700)

        #### Returns:
            The current integration time

        #### Examples:
            opt1.integration_time(50)
        '''
        pass

    def rgb(self, raw=False):
        '''### get the optical sensor rgb value

        #### Arguments:
            raw (optional) : return raw or processed values

        #### Returns:
            A tuple with red, green, blue and brightness

        #### Examples:
            value=opt1.rgb()
        '''
        return ()

    def object_detect_threshold(self, value: vexnumber):
        '''### set the threshold for object detection

        #### Arguments:
            value : Number in the range 0 to 255.  A value of 0 will just return current value.

        #### Returns:
            current value

        #### Examples:
            opt1.object_detect_threshold(100)
        '''
        return 0

    def gesture_enable(self):
        '''### Enable gesture mode

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            opt1.gesture_enable()
        '''
        pass

    def gesture_disable(self):
        '''### Disable gesture mode

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            opt1.gesture_disable()
        '''
        pass

    def get_gesture(self, newobject=False):
        '''### get gesture data

        #### Arguments:
            newobject (optional) : create a new Gesture object to return data in

        #### Returns:
            An object with the last gesture data

        #### Examples:
            opt1.gesture_disable()
        '''
        return Optical.Gesture()

    def object_detected(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when an object detected event occurs

        #### Arguments:
            callback : A function that will be called when an object detected event occurs
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("object detected")

            opt1.object_detected(foo)
        '''
        return Event(callback, arg)

    def object_lost(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when an object lost event occurs

        #### Arguments:
            callback : A function that will be called when an object lost event occurs
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("object lost")

            opt1.object_lost(foo)
        '''
        return Event(callback, arg)

    def gesture_up(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when a gesture up event is detected

        gesture must be enabled for events to fire.

        #### Arguments:
            callback : A function that will be called when a gesture up event is detected
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("up detected")

            opt1.gesture_up(foo)
        '''
        return Event(callback, arg)

    def gesture_down(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when a gesture down event is detected

        gesture must be enabled for events to fire.

        #### Arguments:
            callback : A function that will be called when a gesture down event is detected
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("down detected")

            opt1.gesture_down(foo)
        '''
        return Event(callback, arg)

    def gesture_left(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when a gesture left event is detected

        gesture must be enabled for events to fire.

        #### Arguments:
            callback : A function that will be called when a gesture left event is detected
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("left detected")

            opt1.gesture_left(foo)
        '''
        return Event(callback, arg)

    def gesture_right(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when a gesture right event is detected

        gesture must be enabled for events to fire.

        #### Arguments:
            callback : A function that will be called when a gesture right event is detected
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("right detected")

            opt1.gesture_right(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class Distance:
    '''### Distance class - a class for working with the distance sensor

    #### Arguments:
        port : The smartport this device is attached to

    #### Returns:
        An instance of the Distance class

    #### Examples:
        dist1 = Distance(Ports.PORT1)
    '''
    def __init__(self, port):
        self._index = port

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def object_distance(self, units=DistanceUnits.MM):
        '''### The current distance the sensor is reading.

        The distance will return a large positive number if no object is detected.

        #### Arguments:
            units (optional): The distance units to return the distance value in.  default is MM.

        #### Returns:
            A value for distance in the specified units.

        #### Examples:
            # get distance in mm\\
            value = dist1.object_distance()

            # get distance in inches\\
            value = dist1.object_distance(INCHES)
        '''
        return 0

    def object_size(self):
        '''### Get an estimation of the object size the sensor is detecting.

        #### Arguments:
            None

        #### Returns:
            A value for object size.\\
            The value will be of type ObjectSizeType

        #### Examples:
            # get object size\\
            size = dist1.object_size()
        '''
        return ObjectSizeType.NONE

    def object_rawsize(self):
        '''### Get the raw value of object size the sensor is detecting.

        Raw size will be a number ranging from 0 to about 400\\
        Larger and more reflective objects will return larger values.

        #### Arguments:
            None

        #### Returns:
            A value for object size that is a number.\\

        #### Examples:
            # get object raw size\\
            size = dist1.object_rawsize()
        '''
        return 0

    def object_velocity(self):
        '''### Returns the object velocity

        velocity is calculated from change of distance over time

        #### Arguments:
            None

        #### Returns:
            The velocity in m/s
        '''
        return 0

    def is_object_detected(self):
        '''### Returns if an object is detected

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def changed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the distance value changes

        #### Arguments:
            callback : A function that will be called when the distance value changes
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("distance changed")

            dist1.changed(foo)
        '''
        return Event(callback, arg)

# ----------------------------------------------------------

class Electromagnet:
    '''### Electromagnet class - a class for working with the electromagnet

    #### Arguments:
        port : The smartport this device is attached to

    #### Returns:
        An instance of the Electromagnet class

    #### Examples:
        em1 = Electromagnet(Ports.PORT1)
    '''
    def __init__(self, port):
        self._index = port
        self._power = 50

    def installed(self):
        '''### Check for device connection

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return True

    def timestamp(self):
        '''### Request the timestamp of last received message from the sensor

        #### Arguments:
            None

        #### Returns:
            timestamp of the last status packet in mS
        '''
        return 0

    def set_power(self, value):
        '''### set the default power to use for drop and pickup methods

        #### Arguments:
            value : power in range 0 to 100

        #### Returns:
            None

        #### Examples:
            # set default power to 80\\
            em1.set_power(80)
        '''
        self._power = value

    def pickup(self, duration=1000, units=MSEC, power=50):
        '''### energize the electromagnet to pickup objects

        #### Arguments:
            duration (optional) : the duration to energize the magnet for, default is 1 second
            units (optional) : the units for duration, default is MSEC
            power (optional) : the power used when energizing.

        #### Returns:
            None

        #### Examples:
            # pickup with default values\\
            em1.pickup()

            # pickup with custom values\\
            em1.pickup(250, MSEC, 90)
        '''
        pass

    def drop(self, duration=1000, units=MSEC, power=50):
        '''### energize the electromagnet to drop objects

        #### Arguments:
            duration (optional) : the duration to energize the magnet for, default is 1 second
            units (optional) : the units for duration, default is MSEC
            power (optional) : the power used when energizing.

        #### Returns:
            None

        #### Examples:
            # drop with default values\\
            em1.drop()

            # drop with custom values\\
            em1.drop(250, MSEC, 90)
        '''
        pass

    def temperature(self, *args):
        '''### Returns the temperature of the electromagnet

        #### Arguments:
            units (optional) : The units for the returned temperature, the default is CELSIUS

        #### Returns:
            The electromagnet temperature in provided units
        '''
        return 20

# ----------------------------------------------------------
AddressableLedList = Union[List[Color], List[Color.DefinedColor]]

class AddressableLed:
    '''### Addressable led class

    #### Arguments:
        port : The 3wire port to use for the addressable led strip

    #### Returns:
        An instance of the AddressableLed class

    #### Examples:
        addr1 = AddressableLed(brain.three_wire_port.a)
    '''
    def __init__(self, port):
        self._index = port

    def value(self):
        return 0

    def type(self):
        return 0

    def clear(self):
        '''### clear all addressable led to off

        #### Arguments:
            None

        #### Returns:
            None

        #### Examples:
            addr1.clear()
        '''
        pass

    def set(self, data:AddressableLedList, offset:vexnumber=0):
        '''### Set the addressable led strip to provided values

        #### Arguments:
            data : An list of Color values
            offset (optional) : index of led to start at, 0 based

        #### Returns:
            None

        #### Examples:
            addr1 = AddressableLed(brain.three_wire_port.a)\\
            pix = [Color(0x800000),Color(0x008000),Color(0x000080)]\\
            addr1.set(pix)
        '''
        pass

# ----------------------------------------------------------
# internal use only

class EventMask:
    def __init__(self, *args):
        self.value = args[0]
        if len(args) > 1:
            self.value = (self.value << 16) | args[1]
        self.name = "EVENTMASK"

# *-------------------------------------------------------------------------*/
# *                                                                         */
# *    Copyright (c) Innovation First 2019, All rights reserved.            */
# *                                                                         */
# *    Module:     motorgroup.py                                            */
# *    Author:     James Pearman                                            */
# *    Created:    30 December 2019                                         */
# *                                                                         */
# *    Revisions:                                                           */
# *                V1.00     TBD - Initial release                          */
# *                                                                         */
# *-------------------------------------------------------------------------*/

class MotorGroup:
    '''### MotorGroup class - use this to create a group of motors

    #### Arguments:
        One or more Motor class instances

    #### Returns:
        A new MotorGroup object.

    #### Examples:
        motor1 = Motor(Ports.PORT1)\\
        motor2 = Motor(Ports.PORT2)\\
        mg1 = MotorGroup(motor1, motor2)
    '''
    def __init__(self, *argv):
        self._motors = list()

        for arg in argv:
            if isinstance(arg, Motor):
                self._motors.append(arg)

        self._timeout = sys.maxsize

# ----------------------------------------------------------------------------
    def count(self):
        '''### return the number of motors in the group

        #### Arguments:
            None

        #### Returns:
            The number of motors in the group
        '''
        return len(self._motors)

# ----------------------------------------------------------------------------
    def set_velocity(self, velocity, units=None):
        '''### Set default velocity for all motors in the group
        This will be the velocity used for subsequent calls to spin if a velocity is not provided
        to that function.

        #### Arguments:
            velocity : The new velocity
            units : The units for the supplied velocity, the default is RPM

        #### Returns:
            None
        '''
        for m in self._motors:
            m.set_velocity(velocity, units)

# ----------------------------------------------------------------------------
    def set_stopping(self, mode=BrakeType.COAST):
        '''### Set the stopping mode for all motors in the group
        Setting the action for the motor when stopped.

        #### Arguments:
            mode : The stopping mode, COAST, BRAKE or HOLD

        #### Returns:
            None
        '''
        for m in self._motors:
            m.set_stopping(mode)

# ----------------------------------------------------------------------------
    def reset_position(self):
        '''### Reset the motor position to 0 for all motors in the group

        #### Arguments:
            None

        #### Returns:
            None
        '''
        for m in self._motors:
            m.reset_position()

# ----------------------------------------------------------------------------
    def set_position(self, value, units=None):
        '''### Set the current position for all motors in the group
        The position returned by the position() function is set to this value.

        #### Arguments:
            value : The new position
            units : The units for the provided position, the default is DEGREES

        #### Returns:
            None
        '''
        for m in self._motors:
            m.set_position(value, units)

# ----------------------------------------------------------------------------
    def set_timeout(self, timeout, units=TimeUnits.MSEC):
        '''### Set the timeout value used for all motors in the group
        The timeout value is used when performing spin_to_position and spin_for commands.  If timeout is
         reached and the motor has not completed moving, then the spin... function will return False.

        #### Arguments:
            timeout : The new timeout
            units : The units for the provided timeout, the default is MSEC

        #### Returns:
            None
        '''
        if units == TimeUnits.SECONDS and timeout > 0:
            if timeout > 100000:
                timeout = 100000
            self._timeout = timeout * 1000
        elif timeout <= 0:
            self._timeout = sys.maxsize
        else:
            self._timeout = timeout

        for m in self._motors:
            m.set_timeout(timeout, units)

# ----------------------------------------------------------------------------
    def spin(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### Spin all motors in the group using the provided arguments

        #### Arguments:
            direction : The direction to spin the motor, FORWARD or REVERSE
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units (optional) : The units of the provided velocity, default is RPM

        #### Returns:
            None

        #### Examples:
            # spin motors forward at velocity set with set_velocity\\
            mg1.spin(FORWARD)\n
            # spin motors forward at 50 rpm\\
            mg1.spin(FORWARD, 50)\n
            # spin with negative velocity, ie. backwards\\
            mg1.spin(FORWARD, -20)\n
            # spin motors forwards with 100% velocity\\
            mg1.spin(FORWARD, 100, PERCENT)\n
            # spin motors forwards at 50 rpm\\
            mg1.spin(FORWARD, 50, RPM)\n
            # spin motors forwards at 360 dps\\
            mg1.spin(FORWARD, 360.0, VelocityUnits.DPS)
        '''
        for m in self._motors:
            m.spin(direction, velocity, units)

# ----------------------------------------------------------------------------
    def spin_to_position(self, rotation, units=RotationUnits.DEG,
                         velocity=None, units_v:VelocityPercentUnits=VelocityUnits.RPM, wait=True):
        '''### Spin all motors in the group to an absolute position using the provided arguments
        Move the motor to the requested position.\\
        This function supports keyword arguments.

        #### Arguments:
            rotation : The position to spin the motor to
            units (optional) : The units for the provided position, the default is DEGREES
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None

        #### Examples:
            # spin to 180 degrees\\
            mg1.spin_to_position(180)\n
            # spin to 2 TURNS (revolutions)\\
            mg1.spin_to_position(2, TURNS) \n
            # spin to 180 degrees at 25 rpm\\
            mg1.spin_to_position(180, DEGREES, 25, RPM)\n
            # spin to 180 degrees and do not wait for completion\\
            mg1.spin_to_position(180, DEGREES, False)\n
            # spin to 180 degrees and do not wait for completion\\
            mg1.spin_to_position(180, DEGREES, wait=False)
        '''
        for m in self._motors:
            m.spin_to_position(rotation, units, velocity, units_v, False)

        if wait:
            return self.__waitForCompletionAll()

        return False

# ----------------------------------------------------------------------------
    def __spin_for_distance(self, direction, rotation, units,
                            velocity, units_v, wait):
        for m in self._motors:
            m.spin_for(direction, rotation, units, velocity, units_v, False)

        if wait:
            return self.__waitForCompletionAll()

        return False

# ----------------------------------------------------------------------------
    def __spin_for_time(self, direction, time, units, velocity, units_v):
        for m in self._motors:
            if m == self._motors[-1]:
                m.spin_for(direction, time, units, velocity, units_v)
            else:
                m.spin(direction, velocity, units_v)

        self.stop()

# ----------------------------------------------------------------------------
    def spin_for(self, direction, rotation, units:RotationTimeUnits=RotationUnits.DEG,
                 velocity=None, units_v:VelocityPercentUnits=VelocityUnits.RPM, wait=True):
        '''### Spin all motors in the group to a relative position using the provided arguments
        Move the motor to the requested position or for the specified amount of time.\\
        The position is relative (ie. an offset) to the current position\\
        This function supports keyword arguments.

        #### Arguments:
            direction : The direction to spin the motor, FORWARD or REVERSE
            rotation : The relative position to spin the motor to or tha amount of time to spin for
            units (optional) : The units for the provided position or time, the default is DEGREES or MSEC
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None

        #### Examples:
            # spin 180 degrees from the current position\\
            mg1.spin_for(FORWARD, 180)\n
            # spin reverse 2 TURNS (revolutions) from the current position\\
            mg1.spin_for(REVERSE, 2, TURNS)\n
            # spin 180 degrees from the current position at 25 rpm\\
            mg1.spin_for(FORWARD, 180, DEGREES, 25, RPM)\n
            # spin 180 degrees  from the current position and do not wait for completion\\
            mg1.spin_for(FORWARD, 180, DEGREES, False)\n
            # spin 180 degrees  from the current position and do not wait for completion\\
            mg1.spin_for(FORWARD, 180, DEGREES, wait=False)
        '''
        if isinstance(units, TimeUnits):
            time = rotation
            self.__spin_for_time(direction, time, units, velocity, units_v)
        else:
            self.__spin_for_distance(
                direction, rotation, units, velocity, units_v, wait)

# ----------------------------------------------------------------------------
    def is_spinning(self):
        '''### Returns the current status of the spin_to_position or spin_for command
        This function is used when False has been passed as the wait parameter to spin_to_position or spin_for\\
        It will return True if any motor is still spinning or False if they have completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current spin_to_position or spin_for status
        '''
        isAnyMotorSpinning = False
        for m in self._motors:
            isAnyMotorSpinning = isAnyMotorSpinning or m.is_spinning()
        return isAnyMotorSpinning

# ----------------------------------------------------------------------------
    def is_spinning_mode(self):
        isAnyMotorSpinningMode = False
        for m in self._motors:
            isAnyMotorSpinningMode = isAnyMotorSpinningMode or m.is_spinning_mode()
        return isAnyMotorSpinningMode

# ----------------------------------------------------------------------------
    def is_done(self):
        '''### Returns the current status of the spin_to_position or spin_for command
        This function is used when False has been passed as the wait parameter to spin_to_position or spin_for\\
        It will return False if any motor is still spinning or True if they have completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current spin_to_position or spin_for status
        '''
        return not self.is_spinning()

# ----------------------------------------------------------------------------
    def stop(self, mode=None):
        '''### Stop all motors in the group, set to 0 velocity and set current stopping_mode
        The motor will be stopped and set to COAST, BRAKE or HOLD

        #### Arguments:
            None

        #### Returns:
            None
        '''
        for m in self._motors:
            m.stop(mode)

# ----------------------------------------------------------------------------
    def set_max_torque(self, value, units:TorquePercentCurrentUnits=TorqueUnits.NM):
        '''### Set the maximum torque all motors in the group will use
        The torque can be set as torque, current or percent of maximum.

        #### Arguments:
            value : the new maximum torque to use
            units : the units that value is passed in

        #### Returns:
            None

        #### Examples:
            # set maximum torque to 2 Nm\\
            motor1.set_max_torque(2, TorqueUnits.NM)\n
            # set maximum torque to 1 Amp\\
            motor1.set_max_torque(1, CurrentUnits.AMP)\n
            # set maximum torque to 20 percent\\
            motor1.set_max_torque(20, PERCENT)
        '''
        for m in self._motors:
            m.set_max_torque(value, units)

# ----------------------------------------------------------------------------
    def direction(self):
        '''### Returns the current direction the first motor is spinning in

        #### Arguments:
            None

        #### Returns:
            The spin direction, FORWARD, REVERSE or UNDEFINED
        '''
        return self._motors[0].direction()

# ----------------------------------------------------------------------------
    def position(self, units=RotationUnits.DEG):
        '''### Returns the position of the first motor

        #### Arguments:
            units (optional) : The units for the returned position, the default is DEGREES

        #### Returns:
            The motor position in provided units
        '''
        return self._motors[0].position(units)

# ----------------------------------------------------------------------------
    def velocity(self, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### Returns the velocity of the first motor

        #### Arguments:
            units (optional) : The units for the returned velocity, the default is RPM

        #### Returns:
            The motor velocity in provided units
        '''
        return self._motors[0].velocity(units)

# ----------------------------------------------------------------------------
    def current(self, units=CurrentUnits.AMP):
        '''### Returns the total current all motors are using

        #### Arguments:
            units (optional) : The units for the returned current, the default is AMP

        #### Returns:
            The motor current in provided units
        '''
        total_current = 0
        for m in self._motors:
            total_current += m.current(units)
        return total_current

# ----------------------------------------------------------------------------
    def power(self, units=PowerUnits.WATT):
        '''### Returns the power the first motor is providing

        #### Arguments:
            units (optional) : The units for the returned power, the default is WATT

        #### Returns:
            The motor power in provided units
        '''
        return self._motors[0].power(units)

# ----------------------------------------------------------------------------
    def torque(self, units:TorquePercentCurrentUnits=TorqueUnits.NM):
        '''### Returns the torque the first motor is providing

        #### Arguments:
            units (optional) : The units for the returned torque, the default is NM

        #### Returns:
            The motor torque in provided units
        '''
        return self._motors[0].torque(units)

# ----------------------------------------------------------------------------
    def efficiency(self, units=PercentUnits.PERCENT):
        '''### Returns the efficiency of the first motor

        #### Arguments:
            units (optional) : The units for the efficiency, the only valid value is PERCENT

        #### Returns:
            The motor efficiency in percent
        '''
        return self._motors[0].efficiency(units)

# ----------------------------------------------------------------------------
    def temperature(self, units=TemperatureUnits.CELSIUS):
        '''### Returns the temperature of the first motor

        #### Arguments:
            units (optional) : The units for the returned temperature, the default is CELSIUS

        #### Returns:
            The motor temperature in provided units
        '''
        return self._motors[0].temperature(units)

# ----------------------------------------------------------------------------
    def __waitForCompletionAll(self):
        t = self._timeout
        while t > 0 and self.is_spinning():
            t -= 10
            sleep(10)

        done = self.is_done()
        if not done:
            self.stop()

        return done

# *-------------------------------------------------------------------------*/
# *                                                                         */
# *    Copyright (c) Innovation First 2019, All rights reserved.            */
# *                                                                         */
# *    Module:     drivetrain.py                                            */
# *    Author:     James Pearman                                            */
# *    Created:    30 December 2019                                         */
# *                                                                         */
# *    Revisions:                                                           */
# *                V1.00     TBD - Initial release                          */
# *                                                                         */
# *-------------------------------------------------------------------------*/

class DriveTrain:
    '''### DriveTrain class - use this to create a simple drivetrain

    #### Arguments:
        lm : Left motor or motorgroup
        rm : Right motor or motorgroup
        wheelTravel (optional) : The circumference of the driven wheels, default is 300 mm
        trackWidth (optional) : The trackwidth of the drivetrain, default is 320 mm
        wheelBase (optional) : The wheelBase of the drivetrain, default is 320 mm
        units (optional) : The units that wheelTravel, trackWidth and wheelBase are specified in, default is MM.
        externalGearRatio (optional) : An optional gear ratio used to compensate drive distances if gearing is used.

    #### Returns:
        A new DriveTrain object.

    #### Examples:
        # A simple two motor drivetrain using default values\\
        motor1 = Motor(Ports.PORT1)\\
        motor2 = Motor(Ports.PORT2, True)\\
        drive1 = DriveTrain(motor1, motor2)

        # A four motor drivetrain using custom values\\
        motor1 = Motor(Ports.PORT1)\\
        motor2 = Motor(Ports.PORT2)\\
        motor3 = Motor(Ports.PORT3, True)\\
        motor4 = Motor(Ports.PORT4, True)\\
        mgl = MotorGroup(motor1, motor3)\\
        mgr = MotorGroup(motor2, motor4)\\
        drive1 = DriveTrain(mgl, mgr, 8.6, 10, 12, INCHES)
    '''
    def __init__(self, lm, rm, wheelTravel:vexnumber=300, trackWidth:vexnumber=320, wheelBase:vexnumber=320,
                 units=DistanceUnits.MM, externalGearRatio=1.0):
        if(not (isinstance(lm, Motor) or isinstance(lm, MotorGroup)) or
           not (isinstance(rm, Motor) or isinstance(rm, MotorGroup))):
            raise TypeError('must pass two motors or motor groups')

        # motors or motor groups
        self.lm = lm
        self.rm = rm

# ----------------------------------------------------------------------------
    def set_drive_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### Set default velocity for drive commands
        This will be the velocity used for subsequent calls to drive if a velocity is not provided
        to that function.

        #### Arguments:
            velocity : The new velocity
            units : The units for the supplied velocity, the default is RPM

        #### Returns:
            None
        '''
        pass
# ----------------------------------------------------------------------------

    def set_turn_velocity(self, velocity, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### Set default velocity for turn commands
        This will be the velocity used for subsequent calls to turn if a velocity is not provided
        to that function.

        #### Arguments:
            velocity : The new velocity
            units : The units for the supplied velocity, the default is RPM

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------------------------
    def set_stopping(self, mode=BrakeType.COAST):
        '''### Set the stopping mode for all motors on the drivetrain
        Setting the action for the motors when stopped.

        #### Arguments:
            mode : The stopping mode, COAST, BRAKE or HOLD

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------------------------
    def set_timeout(self, timeout, units=TimeUnits.MSEC):
        '''### Set the timeout value used all motors on the drivetrain
        The timeout value is used when performing drive_for and turn_for commands.  If timeout is
         reached and the motor has not completed moving, then the function will return False.

        #### Arguments:
            timeout : The new timeout
            units : The units for the provided timeout, the default is MSEC

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------------------------
    def get_timeout(self):
        '''### Get the current timeout value used by the drivetrain

        #### Arguments:
            None

        #### Returns:
            Timeout value in mS
        '''
        return 1000

# ----------------------------------------------------------------------------
    def drive(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### drive the drivetrain using the provided arguments

        The drive command is similar to the motor spin command.\\
        all drive motors are commanded using the provided parameters.

        #### Arguments:
            direction : The direction to drive, FORWARD or REVERSE
            velocity (optional) : spin the motors using this velocity, the default velocity set by set_velocity will be used if not provided.
            units (optional) : The units of the provided velocity, default is RPM

        #### Returns:
            None

        #### Examples:
            # drive forward at velocity set with set_velocity\\
            drive1.drive(FORWARD)\n
            # drive forward at 50 rpm\\
            drive1.drive(FORWARD, 50)\n
            # drive with negative velocity, ie. backwards\\
            drive1.drive(FORWARD, -20)\n
            # drive forwards with 100% velocity\\
            drive1.drive(FORWARD, 100, PERCENT)\n
            # drive forwards at 50 rpm\\
            drive1.drive(FORWARD, 50, RPM)\n
            # drive forwards at 360 dps\\
            drive1.drive(FORWARD, 360.0, VelocityUnits.DPS)
        '''
        pass

# ----------------------------------------------------------------------------
    def drive_for(self, direction, distance, units=DistanceUnits.IN,
                  velocity=None, units_v:VelocityPercentUnits=VelocityUnits.RPM, wait=True):
        '''### move the drivetrain using the provided arguments

        The drive_for command is similar to the motor spin_for command,\\
        however, the drivetrain is commanded to move a distance.

        #### Arguments:
            direction : The direction to drive
            distance : The distance to drive
            units (optional) : The units for the provided distance, the default is INCHES
            velocity (optional) : drive using this velocity, the default velocity set by set_drive_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None or if wait is True then completion success or failure

        #### Examples:
            # drive forward 10 inches from the current position\\
            drive1.drive_for(FORWARD, 10, INCHES)\n
            # drive reverse 1000mm from the current position with motors at 50 rpm\\
            drive1.drive_for(REVERSE, 10000, MM, 50, RPM)
        '''
        pass

# ----------------------------------------------------------------------------
    def turn(self, direction, velocity=None, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### turn the drivetrain using the provided arguments

        The drive command is similar to the motor spin command.\\
        all drive motors are commanded using the provided parameters.

        #### Arguments:
            direction : The turn direction, LEFT or RIGHT
            velocity (optional) : spin the motors using this velocity, the default velocity set by set_turn_velocity will be used if not provided.
            units (optional) : The units of the provided velocity, default is RPM

        #### Returns:
            None

        #### Examples:
            # turn left at velocity set with set_turn_velocity\\
            drive1.turn(LEFT)\n
            # drive right at 50 rpm\\
            drive1.turn(RIGHT, 50)\n
            # turn right with 100% velocity\\
            drive1.turn(RIGHT, 100, PERCENT)\n
            # turn right at 50 rpm\\
            drive1.turn(RIGHT, 50, RPM)\n
            # turn right at 360 dps\\
            drive1.turn(RIGHT, 360.0, VelocityUnits.DPS)
        '''
        pass

# ----------------------------------------------------------------------------
    def turn_for(self, direction, angle, units=RotationUnits.DEG,
                 velocity=None, units_v:VelocityPercentUnits=VelocityUnits.RPM, wait=True):
        '''### turn the drivetrain using the provided arguments

        The turn_for command is similar to the motor spin_for command,\\
        however, the drivetrain is commanded to turn a specified angle.

        #### Arguments:
            direction : The direction to turn, LEFT or RIGHT
            angle : The angle to turn
            units (optional) : The units for the provided angle, the default is DEGREES
            velocity (optional) : drive using this velocity, the default velocity set by set_drive_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None or if wait is True then completion success or failure

        #### Examples:
            # turn right 90 degrees\\
            drive1.turn_for(RIGHT, 90, DEGREES)\n
            # turn left 180 degrees with motors at 50 rpm\\
            drive1.turn_for(LEFT, 180, DEGREES, 50, RPM)
        '''
        pass

# ----------------------------------------------------------------------------
    def is_moving(self):
        '''### Returns the current status of the drive_for or turn_for command
        This function is used when False has been passed as the wait parameter to drive_for or turn_for\\
        It will return True if the drivetrain is still moving or False if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current drive_for or turn_for status
        '''
        return False

# ----------------------------------------------------------------------------
    def is_done(self):
        '''### Returns the current status of the drive_for or turn_for command
        This function is used when False has been passed as the wait parameter to drive_for or turn_for\\
        It will return False if the drivetrain is still moving or True if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current drive_for or turn_for status
        '''
        return True

# ----------------------------------------------------------------------------

    def stop(self, mode=None):
        '''### Stop the drivetrain, set to 0 velocity and set current stopping_mode
        The motors will be stopped and set to COAST, BRAKE or HOLD

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------------------------
    def velocity(self, units:VelocityPercentUnits=VelocityUnits.RPM):
        '''### Returns average velocity of the left and right motors

        #### Arguments:
            units (optional) : The units for the returned velocity, the default is RPM

        #### Returns:
            The drivetrain velocity in provided units
        '''
        return 0

# ----------------------------------------------------------------------------
    def current(self, units=CurrentUnits.AMP):
        '''### Returns the total current all drivetrain motors are using

        #### Arguments:
            units (optional) : The units for the returned current, the default is AMP

        #### Returns:
            The drivetrain current in provided units
        '''
        return 0

# ----------------------------------------------------------------------------
    def power(self, units=PowerUnits.WATT):
        '''### Returns the total power all drivetrain motors are using

        This command only considers the first motor for left and right sides of the drive.

        #### Arguments:
            units (optional) : The units for the returned power, the default is WATT

        #### Returns:
            The drivetrain power in provided units
        '''
        return 0

# ----------------------------------------------------------------------------
    def torque(self, units=TorqueUnits.NM):
        '''### Returns the total torque all drivetrain motors are using

        This command only considers the first motor for left and right sides of the drive.

        #### Arguments:
            units (optional) : The units for the returned torque, the default is NM

        #### Returns:
            The motor torque in provided units
        '''
        return 0

# ----------------------------------------------------------------------------
    def efficiency(self, units=PercentUnits.PERCENT):
        '''### Returns the average efficiency of the left and right motors

        This command only considers the first motor for left and right sides of the drive.

        #### Arguments:
            units (optional) : The units for the efficiency, the only valid value is PERCENT

        #### Returns:
            The motor efficiency in percent
        '''
        return 0

# ----------------------------------------------------------------------------
    def temperature(self, units=TemperatureUnits.CELSIUS):
        '''### Returns the average temperature of the left and right motors

        This command only considers the first motor for left and right sides of the drive.

        #### Arguments:
            units (optional) : The units for the returned temperature, the default is CELSIUS

        #### Returns:
            The motor temperature in provided units
        '''
        return 0

# *-------------------------------------------------------------------------*/
# *                                                                         */
# *    Copyright (c) Innovation First 2019, All rights reserved.            */
# *                                                                         */
# *    Module:     smartdrive.py                                            */
# *    Author:     James Pearman                                            */
# *    Created:    30 December 2019                                         */
# *                                                                         */
# *    Revisions:                                                           */
# *                V1.00     TBD - Initial release                          */
# *                                                                         */
# *-------------------------------------------------------------------------*/

class SmartDrive(DriveTrain):
    '''### SmartDrive class - use this to create a smart drivetrain

    A smart drivetrain uses a gyro or similar sensor to turn more accurately.\\
    The smartdrive inherits all drivetrain functions.

    #### Arguments:
        lm : Left motor or motorgroup
        rm : Right motor or motorgroup
        g : The gyro, inertial sensor or gps to use for turns
        wheelTravel (optional) : The circumference of the driven wheels, default is 300 mm
        trackWidth (optional) : The trackwidth of the drivetrain, default is 320 mm
        wheelBase (optional) : The wheelBase of the drivetrain, default is 320 mm
        units (optional) : The units that wheelTravel, trackWidth and wheelBase are specified in, default is MM.
        externalGearRatio (optional) : An optional gear ratio used to compensate drive distances if gearing is used.

    #### Returns:
        A new SmartDrive object.

    #### Examples:
        # A simple two motor smart drivetrain using default values\\
        motor1 = Motor(Ports.PORT1)\\
        motor2 = Motor(Ports.PORT2, True)\\
        imu1 = Inertial(Ports.PORT9)\\
        smart1 = SmartDrive(motor1, motor2, imu1)

        # A four motor smart drivetrain using custom values\\
        motor1 = Motor(Ports.PORT1)\\
        motor2 = Motor(Ports.PORT2)\\
        motor3 = Motor(Ports.PORT3, True)\\
        motor4 = Motor(Ports.PORT4, True)\\
        imu1 = Inertial(Ports.PORT9)\\
        mgl = MotorGroup(motor1, motor3)\\
        mgr = MotorGroup(motor2, motor4)\\
        smart1 = SmartDrive(mgl, mgr, imu1, 8.6, 10, 12, INCHES)
    '''
    def __init__(self, lm, rm, g, wheelTravel:vexnumber=300, trackWidth:vexnumber=320,
                 wheelBase:vexnumber=320, units=DistanceUnits.MM,
                 externalGearRatio=1.0):

        if(not (isinstance(lm, Motor) or isinstance(lm, MotorGroup)) or
           not (isinstance(rm, Motor) or isinstance(rm, MotorGroup))):
            raise TypeError('must pass two motors or motor groups')

        if not (isinstance(g, Gyro) or isinstance(g, Inertial) or isinstance(g, Gps)):
            raise TypeError('must pass Gyro, Inertial or Gps instance')

        DriveTrain.__init__(self, lm, rm, wheelTravel,
                            trackWidth, wheelBase, units, externalGearRatio)
        self.g = g

# ----------------------------------------------------------
    def set_turn_threshold(self, value):
        '''### Set the turning threshold for the smartdrive

        This is the threshold value used to determine that turns are complete.\\
        If this is too large then turns will not be accurate, if too small then turns ma\\
        not complete.

        #### Arguments:
            value : The new turn threshold in degrees, the default for a smartdrive is 1 degree

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------
    def set_turn_constant(self, value):
        '''### Set the turning constant for the smartdrive

        The smartdrive uses a simple P controller when doing turns.\\
        This constant, generally known as kp, is the gain used in the equation that\\
        turns angular error into motor velocity.

        #### Arguments:
            value : The new turn constant in the range 0.1 - 4.0, the default is 1.0

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------
    def set_turn_direction_reverse(self, value):
        '''### Set the expected turn direction for positive heading change

        #### Arguments:
            value : True or False

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------
    def set_heading(self, value, units=RotationUnits.DEG):
        '''### set the smartdrive heading to a new value

        The new value for heading should be in the range 0 - 359.99 degrees.

        #### Arguments:
            value : The new value to use for heading.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of heading to 180 degrees\\
            smart1.set_heading(180)
        '''
        pass
# ----------------------------------------------------------

    def heading(self, units=RotationUnits.DEG):
        '''### read the current heading of the smartdrive

        heading will be returned in the range 0 - 359.99 degrees

        #### Arguments:
            units (optional) : The units to return the heading in, default is DEGREES

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current heading for the smartdrive\\
            value = smart1.heading()
        '''
        return 0

# ----------------------------------------------------------
    def set_rotation(self, value, units=RotationUnits.DEG):
        '''### set the smartdrive rotation to a new value

        #### Arguments:
            value : The new value to use for rotation.
            units (optional) : The rotation units type for value, the default is DEGREES

        #### Returns:
            None

        #### Examples:
            # set the value of rotation to 180 degrees\\
            smart1.set_rotation(180)
        '''
        pass

# ----------------------------------------------------------
    def rotation(self, units=RotationUnits.DEG):
        '''### read the current rotation of the smartdrive

        rotation is not limited, it can be both positive and negative and shows the absolute angle of the gyro.

        #### Arguments:
            units (optional) : The units to return the rotation in, default is DEGREES

        #### Returns:
            A value for heading in the range that is specified by the units.

        #### Examples:
            # get the current rotation for the smartdrive\\
            value = smart1.rotation()
        '''
        return self.g.rotation(units)

# ----------------------------------------------------------
    def turn_to_heading(self, angle, units=RotationUnits.DEG,
                        velocity=None, units_v:VelocityPercentUnits=VelocityUnits.RPM, wait=True):
        '''### turn the smartdrive to an absolute position using the provided arguments

        The turn_to_heading command is similar to the motor spin_to_position command,\\
        however, the smartdrive is commanded to turn to a specified angle.\\
        This function uses the value of heading() when turning the smartdrive\\
        This function supports keyword arguments.

        #### Arguments:
            angle : The angle to turn to
            units (optional) : The units for the provided angle, the default is DEGREES
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None

        #### Examples:
            # turn to heading 180 degrees\\
            smart1.turn_to_heading(180)\n
            # turn to heading 180 degrees at 25 rpm\\
            smart1.turn_to_heading(180, DEGREES, 25, RPM)\n
            # turn to heading 180 degrees and do not wait for completion\\
            smart1.turn_to_heading(180, DEGREES, False)\n
            # turn to heading 180 degrees and do not wait for completion\\
            smart1.turn_to_heading(180, DEGREES, wait=False)
        '''
        return True

# ----------------------------------------------------------
    def turn_to_rotation(self, angle, units=RotationUnits.DEG,
                         velocity=None, units_v:VelocityPercentUnits=VelocityUnits.RPM, wait=True):
        '''### turn the smartdrive to an absolute position using the provided arguments

        The turn_to_rotation command is similar to the motor spin_to_position command,\\
        however, the smartdrive is commanded to turn to a specified angle.\\
        This function uses the value of rotation() when turning the smartdrive\\
        This function supports keyword arguments.

        #### Arguments:
            angle : The angle to turn to
            units (optional) : The units for the provided angle, the default is DEGREES
            velocity (optional) : spin the motor using this velocity, the default velocity set by set_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None

        #### Examples:
            # turn to rotation 180 degrees\\
            smart1.turn_to_rotation(180)\n
            # turn to rotation 400 degrees at 25 rpm\\
            smart1.turn_to_rotation(400, DEGREES, 25, RPM)\n
            # turn to rotation 180 degrees and do not wait for completion\\
            smart1.turn_to_rotation(180, DEGREES, False)\n
            # turn to rotation 180 degrees and do not wait for completion\\
            smart1.turn_to_rotation(180, DEGREES, wait=False)
        '''
        return True

# ----------------------------------------------------------
    def turn_for(self, direction, angle, units=RotationUnits.DEG,
                 velocity=None, units_v:VelocityPercentUnits=VelocityUnits.RPM, wait=True):
        '''### turn the smartdrive using the provided arguments

        The turn_for command is similar to the motor spin_for command,\\
        however, the smartdrive is commanded to turn a specified angle.

        #### Arguments:
            direction : The direction to turn, LEFT or RIGHT
            angle : The angle to turn
            units (optional) : The units for the provided angle, the default is DEGREES
            velocity (optional) : drive using this velocity, the default velocity set by set_drive_velocity will be used if not provided.
            units_v (optional) : The units of the provided velocity, default is RPM
            wait (optional) : This indicates if the function should wait for the command to complete or return immediately, default is True.

        #### Returns:
            None or if wait is True then completion success or failure

        #### Examples:
            # turn right 90 degrees\\
            smart1.turn_for(RIGHT, 90, DEGREES)\n
            # turn left 180 degrees with motors at 50 rpm\\
            smart1.turn_for(LEFT, 180, DEGREES, 50, RPM)
        '''
        return True

# ----------------------------------------------------------
    def is_turning(self):
        '''### Returns the current status of the turn_to_heading, turn_to_rotation or turn_for command
        This function is used when False has been passed as the wait parameter to turn_to_heading or turn_for\\
        It will return True if the drivetrain is still moving or False if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current turn_to_heading, turn_to_rotation or turn_for status
        '''
        return False

# ----------------------------------------------------------
    def is_moving(self):
        '''### Returns the current status of the drive_for command
        This function is used when False has been passed as the wait parameter to drive_for\\
        It will return True if the drivetrain is still moving or False if it has completed the move or a timeout occurred.

        #### Arguments:
            None

        #### Returns:
            The current drive_for status
        '''
        return False

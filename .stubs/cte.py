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
# version: 20240516_1800_00
#
# ----------------------------------------------------------
'''definition of objects in the cte module'''

from typing import Union
from typing import Callable

from vex import vexnumber, vexEnum, LedStateType, Color, Event, TimeUnits

# pylint: disable=unused-argument
# pylint: disable=unnecessary-pass
# pylint: disable=missing-class-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=line-too-long

# ----------------------------------------------------------

class CylinderType:
    class CylinderType(vexEnum):
        pass
    CYLINDER1 = CylinderType(0, "CYLINDER1")
    CYLINDER2 = CylinderType(1, "CYLINDER2")
    CYLINDER3 = CylinderType(2, "CYLINDER3")
    CYLINDER4 = CylinderType(3, "CYLINDER4")
    CYLINDERALL = CylinderType(0xFF, "CYLINDERALL")

class JointType:
    class JointType(vexEnum):
        pass
    JOINT1 = JointType(0, "JOINT1")
    JOINT2 = JointType(1, "JOINT2")
    JOINT3 = JointType(2, "JOINT3")
    JOINT4 = JointType(3, "JOINT4")
    JOINT5 = JointType(4, "JOINT5")
    JOINT6 = JointType(5, "JOINT6")

class PoseType:
    class PoseType(vexEnum):
        pass
    Z_NEG = PoseType(1, "Z_NEG")
    X_POS = PoseType(2, "X_POS")
    Y_POS = PoseType(3, "Y_POS")
    Y_NEG = PoseType(4, "Y_NEG")
    X_NEG = PoseType(5, "X_NEG")
    Z_POS = PoseType(6, "Z_POS")
    PEN = PoseType(7, "PEN")

class EndEffectorType:
    class EndEffectorType(vexEnum):
        pass
    MAGNET = EndEffectorType(1, "MAGNET")
    PEN = EndEffectorType(7, "PEN")

class MoveType:
    class MoveType(vexEnum):
        pass
    SYSTEM = MoveType(0, "SYSTEM")
    LINEAR = MoveType(1, "LINEAR")
    JOINT = MoveType(2, "JOINT")

# ----------------------------------------------------------
# globals
CYLINDER1 = CylinderType.CYLINDER1
'''The first pneumatic cylinder'''
CYLINDER2 = CylinderType.CYLINDER2
'''The second pneumatic cylinder'''
CYLINDER3 = CylinderType.CYLINDER3
'''The third pneumatic cylinder'''
CYLINDER4 = CylinderType.CYLINDER4
'''The fourth pneumatic cylinder'''
CYLINDERALL = CylinderType.CYLINDERALL
'''All pneumatic cylinders'''

JOINT1 = JointType.JOINT1
'''workcell arm joint 1'''
JOINT2 = JointType.JOINT2
'''workcell arm joint 2'''
JOINT3 = JointType.JOINT3
'''workcell arm joint 3'''
JOINT4 = JointType.JOINT4
'''workcell arm joint 4'''
JOINT5 = JointType.JOINT5
'''workcell arm joint 5'''
JOINT6 = JointType.JOINT6
'''workcell arm joint 6'''

# ----------------------------------------------------------

class Pneumatic:
    '''### Pneumatic class - a class for working with the Pneumatic device

    #### Arguments:
        port : The smartport this device is attached to

    #### Returns:
        An instance of the Pneumatic class

    #### Examples:
        pnu1 = Pneumatic(Ports.PORT1)
    '''
    def __init__(self, port, compressor_enable:bool = True):
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

    def extend(self, value: CylinderType.CylinderType):
        '''### extend a cylinder

        #### Arguments:
            value : The cylinder to extend

        #### Returns:
            None

        #### Examples:
            # extend the first cylinder\\
            pnu1.extend(CylinderType.CYLINDER1)
        '''
        pass

    def retract(self, value: CylinderType.CylinderType):
        '''### retract a cylinder

        #### Arguments:
            value : The cylinder to retract

        #### Returns:
            None

        #### Examples:
            # retract the first cylinder\\
            pnu1.retract(CylinderType.CYLINDER1)
        '''
        pass

    def pump_on(self):
        '''### Turn the pneumatic air pump on

        #### Arguments:

        #### Returns:
            None

        #### Examples:
            # Turn air pump on\\
            pnu1.pump_on()
        '''
        pass

    def pump_off(self):
        '''### Turn the pneumatic air pump off

        #### Arguments:

        #### Returns:
            None

        #### Examples:
            # Turn air pump off\\
            pnu1.pump_off()
        '''
        pass

    def pump(self, state: bool):
        '''### Turn the pneumatic air pump on or off

        #### Arguments:
            state : The compressor state, on or off

        #### Returns:
            None

        #### Examples:
            # Turn air pump on\\
            pnu1.pump(True)

            # Turn air pump off\\
            pnu1.pump(False)
        '''
        pass

    def status(self):
        '''### Return raw status for the pneumatic device

        #### Arguments:
            None

        #### Returns:
            status as integer
        '''
        return 0

# ----------------------------------------------------------

class SignalTower:
    '''### SignalTower class - a class for working with the cte signal tower

    #### Arguments:
        port : The smartport this device is attached to

    #### Returns:
        An instance of the SignalTower class

    #### Examples:
        s1 = SignalTower(Ports.PORT1)
    '''

    ALL = Color(0)
    RED = Color.RED
    GREEN = Color.GREEN
    BLUE = Color.BLUE
    YELLOW = Color.YELLOW
    WHITE = Color.WHITE

    OFF = LedStateType.OFF
    ON = LedStateType.ON
    BLINK = LedStateType.BLINK

    class State:
        ''' Signal Tower State class
        '''
        OFF = LedStateType.OFF
        ON = LedStateType.ON
        BLINK = LedStateType.BLINK

    class Color:
        ''' Signal Tower Color class
        '''
        ALL = Color(0)
        RED = Color.RED
        GREEN = Color.GREEN
        BLUE = Color.BLUE
        YELLOW = Color.YELLOW
        WHITE = Color.WHITE

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

    def set_color(self, *args):
        '''### Turn one or more LED on the signal tower on or off

        #### Arguments:
            value (optional) : The color value, can be specified in various ways, see examples.

        #### Returns:
            None

        #### Examples:
            # turn on red LED\\
            s1.set_color(Color.RED, SignalTower.State.ON)

            # blink red LED\\
            s1.set_color(Color.RED, SignalTower.State.BLINK)

            # turn on red LED only\\
            s1.set_color(Color.RED)

            # turn on with a blue color set by rgb value\\
            s1.set_color(0x0000FF)
        '''
        pass

    def set_colors(self, r: LedStateType.LedStateType, y: LedStateType.LedStateType, g: LedStateType.LedStateType, b: LedStateType.LedStateType, w: LedStateType.LedStateType):
        '''### Turn all LED on the signal tower on or off

        #### Arguments:
            r : The state of the red LED
            y : The state of the yellow LED
            g : The state of the green LED
            b : The state of the blue LED
            w : The state of the white LED

        #### Returns:
            None

        #### Examples:
            # turn on all LED\\
            s1.set_color(LedStateType.ON, LedStateType.ON, LedStateType.ON, LedStateType.ON, LedStateType.ON)

            # turn on just the red LED\\
            s1.set_color(LedStateType.ON, LedStateType.OFF, LedStateType.OFF, LedStateType.OFF, LedStateType.OFF)
        '''
        pass

    def pressing(self):
        '''### Returns whether the signal tower button is currently being pressed

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        return False

    def pressed(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the signal tower button is pressed

        #### Arguments:
            callback : A function that will be called when the button is pressed
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("button pressed")

            s1.pressed(foo)
        '''
        return Event(callback, arg)

    def released(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the signal tower button button is released

        #### Arguments:
            callback : A function that will be called when the button is released
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("button released")

            s1.released(foo)
        '''
        return Event(callback, arg)


# ----------------------------------------------------------

JointParam = Union[JointType.JointType, vexnumber]
PoseParam = Union[PoseType.PoseType, vexnumber]
vexnumbool = Union[int, float, bool]

class Arm:
    '''### CTE workcell arm class - a class for working with the workcell arm

    #### Arguments:
        port : The smartport this device is attached to

    #### Returns:
        An instance of the Arm class

    #### Examples:
        arm1 = Arm(Ports.PORT1)
    '''
    def __init__(self, port):
        self._index = port

    MAGNET = EndEffectorType.MAGNET
    '''workcell end effector type magnet'''
    PEN = EndEffectorType.PEN
    '''workcell end effector type pen'''

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

    def initialize_arm(self) -> None:
        '''### Initialize the arm to a safe position and set default speed.

        #### Arguments:
            None

        #### Returns:
            True or False
        '''
        pass

    def set_timeout(self, timeout, units=TimeUnits.MSEC):
        '''### Set the timeout value used when moving the arm tip

        #### Arguments:
            timeout : The new timeout
            units : The units for the provided timeout, the default is MSEC

        #### Returns:
            None
        '''
        pass

    def get_timeout(self):
        '''### Get the current timeout value used by the arm move commands

        #### Arguments:
            None

        #### Returns:
            Timeout value in mS
        '''
        return 1000

    def get_x(self) -> float:
        '''### Request the X position of the arm tip

        #### Arguments:
            None

        #### Returns:
            X position of the arm tip in mm
        '''
        return 0.0

    def get_y(self) -> float:
        '''### Request the Y position of the arm tip

        #### Arguments:
            None

        #### Returns:
            Y position of the arm tip in mm
        '''
        return 0.0

    def get_z(self) -> float:
        '''### Request the Z position of the arm tip

        #### Arguments:
            None

        #### Returns:
            Z position of the arm tip in mm
        '''
        return 0.0

    def get_roll(self) -> float:
        '''### Request the current roll of the arm tip

        #### Arguments:
            None

        #### Returns:
            The current roll of the arm tip in degrees
        '''
        return 0.0

    def get_pitch(self) -> float:
        '''### Request the current pitch of the arm tip

        #### Arguments:
            None

        #### Returns:
            The current pitch of the arm tip in degrees
        '''
        return 0.0

    def get_yaw(self) -> float:
        '''### Request the current yaw of the arm tip

        #### Arguments:
            None

        #### Returns:
            The current yaw of the arm tip in degrees
        '''
        return 0.0

    def is_connected(self) -> bool:
        '''### Is the cte arm connected

        Compatibility function, returns the same as the installed() function

        #### Arguments:
            None

        #### Returns:
            True if the arm is connected to the brain on the associated smartport
        '''
        return True

    def set_end_effector_magnet(self, state: bool) -> None:
        '''### Set the end effector magnet to enabled or disabled

        #### Arguments:
            state : True or False

        #### Returns:
            None
        '''
        pass

    def set_end_effector_type(self, type: EndEffectorType.EndEffectorType) -> bool:
        '''### Set the end effector type to magnet or pen

        #### Arguments:
            type : MAGNET or PEN

        #### Returns:
            True if the requested type was set
        '''
        return True

    def set_pen_offset(self, zOffset :vexnumber) -> None:
        '''### Set the pen end effector z axis offset

        #### Arguments:
            zOffset : the new offset in mm, positive is up

        #### Returns:
            None
        '''
        pass

    def set_control_stop(self, state: bool=True) -> None:
        '''### Disable the arm and place joint motors in brake mode

        #### Arguments:
            state : if set to True, further linear or joint moves are disabled

        #### Returns:
            None
        '''
        pass

    def set_speed(self, speed :vexnumber) -> None:
        '''### Set the speed for moves

        #### Arguments:
            speed : movement speed

        #### Returns:
            None
        '''
        pass

    def is_done(self) -> bool:
        '''### return status of arm movement

        #### Arguments:
            None

        #### Returns:
            True if the arm is performing a movement
        '''
        return True

    def is_control_stop_enabled(self) -> bool:
        '''### return status of control stop disable

        #### Arguments:
            None

        #### Returns:
            True if the arm disabled
        '''
        return False

    def status(self) -> int:
        return 0

    def last_error(self) -> int:
        '''### Gets the last error if a move command has failed

        #### Arguments:
            None

        #### Returns:
            The last error code if a move command has failed
        '''
        return 0

    def command_complete(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the arm move command is complete

        #### Arguments:
            callback : A function that will be called when the arm move command is complete
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("move is complete")

            arm.command_complete(foo)
        '''
        return Event(callback, arg)

    def control_stopped(self, callback: Callable[...,None], arg: tuple=()):
        '''### Register a function to be called when the arm detects a joint error

        #### Arguments:
            callback : A function that will be called when the arm detects a joint error
            arg (optional) : A tuple that is used to pass arguments to the callback function.

        #### Returns:
            An instance of the Event class

        #### Examples:
            def foo():
                print("crash detected")

            arm.crash_detect(foo)
        '''
        return Event(callback, arg)

    def move_to(self, x: vexnumber, y: vexnumber, z: vexnumber, wait=True) -> bool:
        '''### Move the arm tip to the requested x, y and z absolute position

        #### Arguments:
            x : the tip x position in mm
            y : the tip y position in mm
            z : the tip z position in mm
            wait (optional) : wait for move to complete or timeout

        #### Returns:
            True if the arm has moved to the requested position when wait is True
        '''
        return True

    def move_inc(self, x: vexnumber, y: vexnumber, z: vexnumber, wait=True) -> bool:
        '''### Move the arm tip to the requested x, y and z relative position

        #### Arguments:
            x : the tip x position in mm
            y : the tip y position in mm
            z : the tip z position in mm
            wait (optional) : wait for move to complete or timeout

        #### Returns:
            True if the arm has moved to the requested position when wait is True
        '''
        return True

    def can_arm_reach_to(self, x: vexnumber, y: vexnumber, z: vexnumber) -> bool:
        '''### Can the arm tip move to the requested x, y and z absolute position

        #### Arguments:
            x : the tip x position in mm
            y : the tip y position in mm
            z : the tip z position in mm

        #### Returns:
            True if the arm can move to the requested position
        '''
        return True

    def can_arm_reach_inc(self, x: vexnumber, y: vexnumber, z: vexnumber) -> bool:
        '''### Can the arm tip move to the requested x, y and z relative position

        #### Arguments:
            x : the tip x position in mm
            y : the tip y position in mm
            z : the tip z position in mm

        #### Returns:
            True if the arm can move to the requested position
        '''
        return True

    def move_end_effector_to(self, yaw: vexnumber, roll: vexnumbool=0, pitch: vexnumbool=0, wait=True) -> bool:
        '''### Move the end effector to the requested absolute yaw, roll and pitch orientation

        #### Arguments:
            yaw : tip yaw orientation in degrees
            roll : tip roll orientation in degrees
            pitch : tip pitch orientation in degrees
            wait (optional) : wait for move to complete or timeout

        #### Returns:
            True if the end effector has moved to the requested orientation when wait is True
        '''
        return True

    def move_end_effector_inc(self, yaw: vexnumber, roll: vexnumbool=0, pitch: vexnumbool=0, wait=True) -> bool:
        '''### Move the end effector to the requested relative yaw, roll and pitch orientation

        #### Arguments:
            yaw : tip yaw orientation in degrees
            roll : tip roll orientation in degrees
            pitch : tip pitch orientation in degrees
            wait (optional) : wait for move to complete or timeout

        #### Returns:
            True if the end effector has moved to the requested orientation when wait is True
        '''
        return True

    def can_end_effector_reach_to(self, yaw: vexnumber, roll: vexnumber=0, pitch: vexnumber=0) -> bool:
        '''### Can the end effector move to the requested absolute yaw, roll and pitch orientation

        #### Arguments:
            yaw : tip yaw orientation in degrees
            roll : tip roll orientation in degrees
            pitch : tip pitch orientation in degrees

        #### Returns:
            True if the end effector can move to the requested orientation when wait is True
        '''
        return True

    def can_end_effector_reach_inc(self, yaw: vexnumber, roll: vexnumber=0, pitch: vexnumber=0) -> bool:
        '''### Can the end effector move to the requested relative yaw, roll and pitch orientation

        #### Arguments:
            yaw : tip yaw orientation in degrees
            roll : tip roll orientation in degrees
            pitch : tip pitch orientation in degrees

        #### Returns:
            True if the end effector can move to the requested orientation when wait is True
        '''
        return True

    def set_move_type(self, type: MoveType.MoveType) -> None:
        '''### Override automatic linear or joint move selection

        #### Arguments:
            type : SYSTEM (automatic selection), LINEAR or JOINT

        #### Returns:
            None
        '''
        pass

# ----------------------------------------------------------

class ArmAdvanced(Arm):
    '''### CTE workcell advanced arm class - a class for working with the workcell arm

    #### Arguments:
        port : The smartport this device is attached to

    #### Returns:
        An instance of the ArmAdv class

    #### Examples:
        arm1 = ArmAdv(Ports.PORT1)
    '''
    def __init__(self, port):
        self._index = port

    def move_to_safe_position(self, wait=True) -> bool:
        '''### Move the arm to safe position

        #### Arguments:
            wait (optional) : wait for move to complete or timeout

        #### Returns:
            True if the arm has moved to the requested position when wait is True
        '''
        return True

    def move_to_position_linear(self, x: vexnumber, y: vexnumber, z: vexnumber, yaw: vexnumber, roll: vexnumber, pitch: vexnumber, relative=False, wait=True) -> bool:
        '''### Move the arm tip to requested x, y and z position and orientation using linear movement

        #### Arguments:
            x : the tip x position in mm
            y : the tip y position in mm
            z : the tip z position in mm
            yaw : tip yaw orientation in degrees
            roll : tip roll orientation in degrees
            pitch : tip pitch orientation in degrees
            relative (optional) : move relative to the current position
            wait (optional) : wait for move to complete or timeout

        #### Returns:
            True if the arm has moved to the requested position when wait is True
        '''
        return True

    def move_to_position_joint(self, x: vexnumber, y: vexnumber, z: vexnumber, yaw: vexnumber, roll: vexnumber, pitch: vexnumber, relative=False, wait=True) -> bool:
        '''### Move the arm tip to requested x, y and z position and orientation using joint movements

        #### Arguments:
            x : the tip x position in mm
            y : the tip y position in mm
            z : the tip z position in mm
            yaw : tip yaw orientation in degrees
            roll : tip roll orientation in degrees
            pitch : tip pitch orientation in degrees
            relative (optional) : move relative to the current position
            wait (optional) : wait for move to complete or timeout

        #### Returns:
            True if the arm has moved to the requested position when wait is True
        '''
        return True

    def get_joint_position(self, joint: JointParam) -> float:
        '''### Request joint position

        #### Arguments:
            joint : The requested joint

        #### Returns:
            position of the joint in degrees
        '''
        return 0.0

    def get_joint_velocity(self, joint: JointParam) -> float:
        '''### Request joint velocity

        #### Arguments:
            joint : The requested joint

        #### Returns:
            velocity of the joint in degrees/second
        '''
        return 0.0

    def get_joint_current(self, joint: JointParam) -> float:
        '''### Request joint current

        #### Arguments:
            joint : The requested joint

        #### Returns:
            current of the joint motor in amps
        '''
        return 0.0

    def get_joint_voltage(self, joint: JointParam) -> float:
        '''### Request joint voltage (not implemented)

        Compatibility function, not implemented in this version

        #### Arguments:
            joint : The requested joint

        #### Returns:
            voltage of the joint motor in volts (not implemented)
        '''
        return 0.0

    def get_end_effector_angle(self) -> float:
        '''### Request the angle of the end effector

        #### Arguments:
            None

        #### Returns:
            end effector angle in degrees
        '''
        return 0.0

    def spin_end_effector_to(self, angle: vexnumber, speed: vexnumber=45, wait=True) -> None:
        '''### Spin the end effector to the requested angle

        #### Arguments:
            angle : new angle in degrees
            speed : the speed to move the end effector

        #### Returns:
            None
        '''
        pass

    def spin_end_effector_for(self, angle: vexnumber, speed: vexnumber=45, wait=True) -> None:
        '''### Spin the end effector by the requested angle

        #### Arguments:
            angle : the angle to move in degrees
            speed : the speed to move the end effector

        #### Returns:
            None
        '''
        pass

    def set_end_effector_angle(self, angle :vexnumber) -> None:
        '''### Set the end effector to the given angle

        The end effector does not move but assumes the new angle

        #### Arguments:
            angle : the new angle for the end effector

        #### Returns:
            None
        '''
        pass

    def enable_manual_movement(self) -> None:
        '''### Disable the arm control loops

        This allow the arm to be moved manually

        #### Arguments:
            None

        #### Returns:
            None
        '''
        pass

    def set_linear_move_speed(self, speed :vexnumber) -> None:
        '''### Set the speed for linear moves

        #### Arguments:
            speed : movement speed

        #### Returns:
            None
        '''
        pass

    def set_joint_move_speed(self, speed :vexnumber) -> None:
        '''### Set the speed for joint moves

        #### Arguments:
            speed : movement speed

        #### Returns:
            None
        '''
        pass

    def is_crashed(self) -> bool:
        '''### return status of arm movement error detection

        #### Arguments:
            None

        #### Returns:
            True if the arm is has detected an error during a move
        '''
        return True

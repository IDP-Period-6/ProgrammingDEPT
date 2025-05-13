# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       201131602                                                    #
# 	Created:      4/10/2025, 3:20:43 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #


# press the region declarations to collapse it
#region declarations

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

#controller defintion
controller = Controller()

# Robot configuration code
controller = Controller(PRIMARY)
leftMotor1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
leftMotor2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
leftDrive = MotorGroup(leftMotor1, leftMotor2)
rightMotor1 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
rightMotor2 = Motor(Ports.PORT13, GearSetting.RATIO_18_1, True)
rightDrive = MotorGroup(rightMotor1, rightMotor2)
drivetrainInertial = Inertial(Ports.PORT20)
drivetrain = SmartDrive(leftDrive, rightDrive, drivetrainInertial, 319.19, 320, 40, MM, 1)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    # Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    brain.screen.print("Calibrating")
    brain.screen.next_row()
    brain.screen.print("Inertial")
    drivetrainInertial.calibrate()
    while drivetrainInertial.is_calibrating():
        sleep(25, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)


# Calibrate the Drivetrain
calibrate_drivetrain()



# define variables used for controlling motors based on controller inputs
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            # stop the motors if the brain is calibrating
            if drivetrainInertial.is_calibrating():
                leftDrive.stop()
                rightDrive.stop()
                while drivetrainInertial.is_calibrating():
                    sleep(25, MSEC)
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis2 + axis4
            # right = axis2 - axis4
            drivetrain_left_side_speed = controller.axis2.position() + controller.axis4.position()
            drivetrain_right_side_speed = controller.axis2.position() - controller.axis4.position()
            
            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor
                    leftDrive.stop()
                    # tell the code that the left motor has been stopped
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drivetrain_l_needs_to_be_stopped_controller_1 = True
            # check if the value is inside of the deadband range
            if drivetrain_right_side_speed < 5 and drivetrain_right_side_speed > -5:
                # check if the right motor has already been stopped
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    # stop the right drive motor
                    rightDrive.stop()
                    # tell the code that the right motor has been stopped
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drivetrain_r_needs_to_be_stopped_controller_1 = True
            
            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_l_needs_to_be_stopped_controller_1:
                leftDrive.set_velocity(drivetrain_left_side_speed, PERCENT)
                leftDrive.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                rightDrive.set_velocity(drivetrain_right_side_speed, PERCENT)
                rightDrive.spin(FORWARD)
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)




# ai vision sensor code
purple = Colordesc(1, 214, 72, 219, 10, 0.2)
aivisionsensor = AiVision(Ports.PORT12, AiVision.ALL_TAGS, purple)
snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)




# distance sensor code
distanceSensor = Distance(Ports.PORT7)
value = distanceSensor.object_distance(INCHES)



clawHeight = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
clawControl = Motor(Ports.PORT3, GearSetting.RATIO_18_1)



#endregion 



#global variables for running
vialChecker = False
distanceChecker = False
tubeChecker = True



 #’open claw’

start = False
# 1 checks when to start the entire autonomous routine
while start == False:
    value = distanceSensor.object_distance(INCHES)
    if (value <= 3): 
        print("Don't Start Yet")
        wait(1, SECONDS)
    elif(value > 3):
        print("Start Program!")
        start = True		


forwardClear = False
# 2 define the function called is the forward clear           
def forwardIsClear():
    global forwardClear
    value = distanceSensor.object_distance(INCHES)
    if(value > 7):
        forwardClear = True
    elif(value < 7):
        forwardClear = False

RobotFinished = False
while(RobotFinished == False and start == True):
    # drivetrain drive forward for one y coordinate 
    forwardIsClear()
    if controller.buttonUp.pressed:
        RobotFinished = True
    elif(forwardClear == False):
        print("A Obstacle Spotted")
        wait(1, SECONDS)
        #drivetrain turn left 
        print("turned left")
    elif(forwardClear == True):
        print("No Obstacle Spotted")
        wait(1, SECONDS)
        forwardClear = True
        
        
if controller.buttonA.pressed:
    aivisionsensor.tag_detection(True)
    snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
    for obj in snapshot:
        print("Tag detected: ", obj.id)
        wait(0.5, SECONDS)
        if obj.id == 8:
            print("This is the correct vial")

robotPart2 = False
while(robotPart2 == False and start == True):
    # drivetrain drive forward for one y coordinate 
    forwardIsClear()
    if controller.buttonUp.pressed:
        robotPart2 = True
    elif(forwardClear == False):
        print("A Obstacle Spotted")
        wait(1, SECONDS)
        #drivetrain turn left 
        print("turned left")
    elif(forwardClear == True):
        print("No Obstacle Spotted")
        wait(1, SECONDS)
        forwardClear = True


#Should work needs testing
leverID = True
while (leverID == True):
    aivisionsensor.tag_detection(True)
    snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
    for obj in snapshot:
        brain.screen.set_cursor(1, 1)
        brain.screen.print("Tag detected: ", obj.id)
        wait(0.5, SECONDS)
        brain.screen.clear_screen()
        if obj.id == 5:
            clawControl.set_velocity(15, PERCENT)
            clawControl.spin_for(REVERSE, 90, DEGREES)




def dropOff():
    while tubeChecker == True:
        purple = Colordesc(1, 214, 72, 219, 10, 0.2)
        tubeColor = aivisionsensor.take_snapshot(purple)
        if len(tubeColor) >= 1:
            clawHeight.spin(FORWARD) #’bring the arm down’
            clawControl.spin_for(REVERSE, 90, DEGREES) #’open claw’
            clawControl.spin_for(FORWARD, 90, DEGREES) #’close claw’
            clawHeight.spin(REVERSE) #’bring claw back up’
            #waitForLever()
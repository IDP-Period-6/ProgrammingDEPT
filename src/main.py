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


# define variables used for controlling motors based on controller inputs
controller1 = Controller(PRIMARY)
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

# define a task that will handle monitoring inputs from controller_1

# inertial sensor code
# Allows us to get headings and turn accurately
inertialSensor = Inertial(Ports.PORT4)
inertialSensor.set_heading(0, DEGREES)
inertialSensor.calibrate()
while inertialSensor.is_calibrating():
    wait(100, MSEC)
# drivetrain code
leftMotor1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1,)
leftMotor2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, )

leftMotor1.set_reversed(True)
leftMotor2.set_reversed(True)


rightMotor1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1,  )
rightMotor2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1,  )

#claw heights controls are inversed
clawHeight = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
clawControl = Motor(Ports.PORT15, GearSetting.RATIO_18_1)

leftMotors = MotorGroup(leftMotor1, leftMotor2, )
rightMotors = MotorGroup(rightMotor1, rightMotor2, )

drivetrain = SmartDrive(leftMotors, rightMotors, inertialSensor)

# define variable for remote controller enable/disable
remote_control_code_enabled = True
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            # stop the motors if the brain is calibrating
            if inertialSensor.is_calibrating():
                leftMotors.stop()
                rightMotors.stop()
                while inertialSensor.is_calibrating():
                    sleep(25, MSEC)
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_side_speed = controller1.axis3.position() + controller1.axis1.position()
            drivetrain_right_side_speed = controller1.axis3.position() - controller1.axis1.position()
            
            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor
                    leftMotors.stop()
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
                    rightMotors.stop()
                    # tell the code that the right motor has been stopped
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drivetrain_r_needs_to_be_stopped_controller_1 = True
            
            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_l_needs_to_be_stopped_controller_1:
                leftMotors.set_velocity(drivetrain_left_side_speed, PERCENT)
                leftMotors.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                rightMotors.set_velocity(drivetrain_right_side_speed, PERCENT)
                rightMotors.spin(FORWARD)
        # wait before repeating the process
        wait(20, MSEC)
rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

# ai vision sensor code
# Allows us to get the purple color signature 
#needed to insert the vial into the disemination chamber
purple = Colordesc(1, 214, 72, 219, 10, 0.2)
aivisionsensor = AiVision(Ports.PORT14, AiVision.ALL_TAGS, purple)
snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)





# distance sensor code
# Allows us to get the distance an object is from
#the distance sensor this is useful for when we have to position
#ourselves accurately such as when we pick up the vial and drop it off
distanceSensor = Distance(Ports.PORT5)
value = distanceSensor.object_distance(INCHES)

rightDistance = Distance(Ports.PORT3)
rightValue = rightDistance.object_distance(INCHES)

leftDistance = Distance(Ports.PORT13)
leftValue = leftDistance.object_distance(INCHES)

#endregion 

        
#global variables for running
vialChecker = False
distanceChecker = False
tubeChecker = False
start = False 

# actually used variables
forwardClear = False
leftClear = False
rightClear = False
RobotFinished = False
leftOrRight = 1
start = False

#robot position coordinates
xCord = 6 
yCord = 0

# This variable is set to true if space infront of the robot is clear
# This variable allows us and the robot to know whether there
#is enough safe space infront of the robot for it to move forward safely
def forwardIsClear():
    global forwardClear
    value = distanceSensor.object_distance(INCHES)
    if(value > 10):
        forwardClear = True
    elif(value < 10):
        forwardClear = False
# This variable allows us and the robot to know whether there
#is enough safe space to the lefto the left of the robot for it to move left safely
def leftIsClear():
    global leftClear
    leftValue = leftDistance.object_distance(INCHES)
    if (leftValue > 10):
        leftClear = True
    elif(leftValue < 10):
        leftClear = False
# This variable allows us and the robot to know whether there
#is enough safe space to the left of the robot for it to move right safely
def rightIsClear():
    global rightClear
    rightValue = rightDistance.object_distance(INCHES)
    if (rightValue > 10):
        rightClear = True
    elif(rightValue < 10):
        rightClear = False

#keeps track of robot's current position on field using coordinates (x and y)
def coordinate_tracker():
    global xCord
    global yCord
    #robot moved up the field 1 in the y axis
    if inertialSensor.heading() <= 10 and inertialSensor.heading() >= 350: 
        yCord += 1
    #robot moved to the left side of the field 1 in the x axis
    elif inertialSensor.heading() <= 80 and inertialSensor.heading() >= 100:
        xCord += 1
    #robot moved down the field, -1 in the y axis
    # 1 in the y axis
    elif inertialSensor.heading() <= 170 and inertialSensor.heading() >= 190:
        yCord -= 1
    #robot moved to the right side of the field, -1 in 
    elif inertialSensor.heading() <= 260 and inertialSensor.heading() >= 280:
        xCord -= 1
    print(xCord)
    print(yCord)

drivetrain.set_drive_velocity(10, PERCENT)
def vialDetection():
    # This makes sure that when we are checking which vial to grab
    #we take a picture of all of the availabe tags and sort through them
    #in the next statement
    while vialChecker == True:
        aivisionsensor.tag_detection(True)
        snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
        value = distanceSensor.object_distance(MM)
        print("hello")
        if(value < 477):
            #This is a for loo used to check every single april tag
            #that was taken inside the picture for the vials
            for obj in snapshot:
                #This will print to the scree
                print("Tag detected: ", obj.id)
                if obj.id == 9:
                    drivetrain.drive_for(FORWARD, 2.5, INCHES)
                    wait(1, SECONDS)
                    clawControl.spin_for(FORWARD, 70, DEGREES)
                    wait(1,SECONDS)
                    clawHeight.spin_for(FORWARD, 100, DEGREES)
                    wait(1, SECONDS)
                    drivetrain.drive_for(REVERSE, 5, INCHES)
                    break
            break
        elif(value > 477):
            print("not ready to check")
            drivetrain.drive_for(FORWARD, 1, MM)
        
        
# set this as the height of the claw to stay at
#clawHeight.spin_for(FORWARD, 255, DEGREES)
# this is the vial wideness
#wait(1, SECONDS)
#vialDetection()




# 1 checks when to start the entire autonomous routine
while start == False:
    value = distanceSensor.object_distance(INCHES)
    clawHeight.spin_for(FORWARD, 240, DEGREES)
    if (value < 10): 
        print("Don't Drive Forward")
    elif(value > 10):
        print("Drive Forward")
        start = True		


while(RobotFinished == False and start == True):
    # drivetrain drive forward for one y coordinate 
    forwardIsClear()
    drivetrain.set_drive_velocity(20, PERCENT)
    if (yCord == 20 and xCord == 4):
        # if the coordinates of the robot are perfectly in front of the vials
        # then we can stop the robot
        drivetrain.stop(BRAKE)
        RobotFinished = True
        start = False
        break
    elif(forwardClear == False):
            # the front of the robot is not clear 
            # we must check the left side first then the right side
            print("forward is not clear")
            leftIsClear()
            # called our function to check if left side is clear
            if (leftClear == False):
                # left side is not clear we must check the right side
                print("left is not clear")
                rightIsClear()
                if(rightClear == False):
                    # the right side is not clear so we just have to back up though this is very RARE!!!
                    print("not possible")
                    drivetrain.drive_for(REVERSE, 6, INCHES)
                    yCord -= 1
                    drivetrain.stop(BRAKE)
                elif(rightClear == True):
                    # the right side is clear so we turn to the right side
                    print("right side is clear")
                    drivetrain.turn_to_heading(90, DEGREES)
                    drivetrain.drive_for(FORWARD, 6, INCHES)
                    coordinate_tracker()
                    leftIsClear()
                    if(leftClear == True):
                        # this means that the left AKA FRONT is clear
                        drivetrain.turn_to_heading(0, DEGREES)
                        drivetrain.drive_for(FORWARD, 6, INCHES)
                        coordinate_tracker()
                    elif(leftClear == False):
                        # the front is not clear
                        forwardIsClear()
                        if(forwardClear == True):
                            drivetrain.drive_for(FORWARD, 6, INCHES)
                            coordinate_tracker()
                        elif(forwardClear == False):
                            # all sides are not clear
                            print("not possible")
            elif(leftClear == True):
                # the left side is clear so we can turn left
                print("left side is clear")
                drivetrain.turn_to_heading(-90, DEGREES)
                drivetrain.drive_for(FORWARD, 6, INCHES)
                coordinate_tracker()
                rightIsClear()
                # we are checking to see if the right side is clear aka the FRONT SIDE
                # if the front side is clear, we are able to face back forward
                if(rightClear == False):
                    # the right side is not clear
                    print("right side aka FRONT is not clear")
                    # now we have to check if the front is clear to continue driving forward
                    forwardIsClear()
                    if (forwardClear == True):
                        # if the front is clear we can drive forward again
                        print("front side is clear AKA LEFT")
                        drivetrain.drive_for(FORWARD, 6, INCHES)                        
                        coordinate_tracker()
                    elif (forwardClear == False):
                        print("the LEFT side is NOT clear")
                        # the forward is not clear
                        # so we have to back up
                        print("we are stuck")
                        drivetrain.stop(BRAKE)
                elif(rightClear == True):
                    # the right side is clear so we can turn back to the front
                    print("the FRONT side is clear so we can start again")
                    drivetrain.turn_to_heading(0, DEGREES)
                    drivetrain.drive_for(FORWARD, 6, INCHES)
                    coordinate_tracker()
    elif(forwardClear == True):
        # the front of the robot is clear so we will continue driving forward
        print("clear")
        wait(1, SECONDS)
        forwardClear = True
        drivetrain.drive_for(FORWARD, 6, INCHES)
        coordinate_tracker()
        


#Should work needs testing
def waitForLever():
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



#function for dropping the vial in the
def dropOff():
    # we use the start_awb in order to clear the whiteness in the ai vision sensor 
    # this balances the color and prevents extra yellowing
    aivisionsensor.start_awb()
    # this entire while loop inside of the drop off function
    # only starts when the variable is set true so it doesn't function
    # until the variable is set true
    while tubeChecker == True:
        aivisionsensor.color_detection(True)
        tubeColor = aivisionsensor.take_snapshot(purple)
        value = distanceSensor.object_distance(MM)
        print(value)
        if (value < 600 and value > 590):
            drivetrain.stop(BRAKE)
            if len(tubeColor) >= 1:
                print("tube detected")
                clawHeight.spin_for(FORWARD, 160, DEGREES)
                drivetrain.drive_for(FORWARD, 6.25, INCHES)
                wait(1, SECONDS)
                clawHeight.set_velocity(10, PERCENT)
                clawHeight.spin_for(REVERSE, 60, DEGREES)
                wait(1, SECONDS)
                clawControl.spin_for(REVERSE, 60, DEGREES)
                drivetrain.drive_for(REVERSE, 3, INCHES)
        elif(value > 600):
            drivetrain.drive(FORWARD)


'''''
clawControl.spin_for(FORWARD, 75, DEGREES)
clawHeight.spin_for(FORWARD, 240, DEGREES)
drivetrain.set_heading(0, DEGREES)
dropOff()
'''
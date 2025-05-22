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


# define variables used for controlling drivetrain motors based on controller inputs
# automaticed generated code for controller drivetrain
controller1 = Controller(PRIMARY)
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

# define a task that will handle monitoring inputs from controller_1

# inertial sensor code
# Allows us to get headings and turn accurately
inertialSensor = Inertial(Ports.PORT4)
inertialSensor.set_heading(0, DEGREES)
inertialSensor.calibrate()
# stops robot until inertial sensor is done callibrating
while inertialSensor.is_calibrating():
    wait(100, MSEC)

# drivetrain code
# This defines the ports to plug the motors into
# so the brain can read the code involving the left motors
leftMotor1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1,)
leftMotor2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, )

# This sets the left motors to be reversed
leftMotor1.set_reversed(True)
leftMotor2.set_reversed(True)

# This defines the ports to plug the motors into
rightMotor1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1,  )
rightMotor2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1,  )

# claw heights controls are inversed
# This defines the ports to plug the arm motor into
# so the brain can read the code involving the correct motors
clawHeight1 = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
clawHeight1.set_velocity(100, PERCENT)
clawHeight1.set_stopping(BRAKE)

# This sets the claw height motor to be reversed as well as
# making its speed as fast as it can
clawHeight2 = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
clawHeight2.set_velocity(100, PERCENT)
clawHeight2.set_stopping(BRAKE)

#This tells the robot which port the motor is plugged into
# and the gear ratio used to control the claw
clawControl = Motor(Ports.PORT15, GearSetting.RATIO_18_1)
#This makes the claw move as fast as it can   
clawControl.set_velocity(100, PERCENT)
# This sets the claw control motor to be in brake mode
# everytime there is no input for the claw
clawControl.set_stopping(BRAKE)

# This groups all of the left motors into one group
# and the right motors into one group to power the wheels
leftMotors = MotorGroup(leftMotor1, leftMotor2, )
rightMotors = MotorGroup(rightMotor1, rightMotor2, )

#This tells the brain/robot what components are being added to the robot
drivetrain = SmartDrive(leftMotors, rightMotors, inertialSensor)

#This sets the robots speed to half of its max speed
drivetrain.set_drive_velocity(50, PERCENT)

# define variables used for controlling motors based on controller inputs
controller_1_left_shoulder_control_motors_stopped = True
controller_1_right_shoulder_control_motors_stopped = True
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, controller_1_left_shoulder_control_motors_stopped, controller_1_right_shoulder_control_motors_stopped, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        # check if the remote control code is enabled
        if remote_control_code_enabled:
            # stop the motors if the brain is calibrating
            if inertialSensor.is_calibrating():
                leftMotors.stop()
                rightMotors.stop()
                # This stops the code while the inertial sensor
                # is calibrating for 25 miliseconds
                while inertialSensor.is_calibrating():
                    sleep(25, MSEC)
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_side_speed = (controller1.axis3.position() + controller1.axis1.position())/3
            drivetrain_right_side_speed = (controller1.axis3.position() - controller1.axis1.position())/3
            
            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor if it needs to be stopped
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
                    # stop the right drive motor if it needs to be stopped
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
            # check the buttonL1/buttonL2 status
            # to control clawHeight
            # Holding left bumper will raise the arm 
            if controller1.buttonL1.pressing():
                clawHeight1.spin(FORWARD)
                clawHeight2.spin(FORWARD)
                controller_1_left_shoulder_control_motors_stopped = False
            # Holding the left trigger will lower the arm
            elif controller1.buttonL2.pressing():
                clawHeight1.spin(REVERSE)
                clawHeight2.spin(REVERSE)
                controller_1_left_shoulder_control_motors_stopped = False
            # check if the left motor has already been stopped
            elif not controller_1_left_shoulder_control_motors_stopped:
                clawHeight1.stop(BRAKE)
                clawHeight2.stop(BRAKE)
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_left_shoulder_control_motors_stopped = True
            # check the buttonR1/buttonR2 status
            # to control clawControl
            # Holding the right bumper will open the claw
            if controller1.buttonR1.pressing():
                clawControl.spin(FORWARD)
                controller_1_right_shoulder_control_motors_stopped = False
            # Holding the right trigger will close the claw
            elif controller1.buttonR2.pressing():
                clawControl.spin(REVERSE)
                controller_1_right_shoulder_control_motors_stopped = False
            # check if the right motor has already been stopped
            elif not controller_1_right_shoulder_control_motors_stopped:
                clawControl.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_right_shoulder_control_motors_stopped = True
        # wait 20 miliseconds before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)


# ai vision sensor code
# Allows us to get the purple color signature 
# needed to insert the vial into the tadisemination chamber
purple = Colordesc(1, 214, 72, 219, 10, 0.2)
aivisionsensor = AiVision(Ports.PORT20, AiVision.ALL_TAGS, purple)
snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)





# distance sensor code
# Allows us to get the distance an object is from
#the distance sensor this is useful for when we have to position
#ourselves accurately such as when we pick up the vial and drop it off
distanceSensor = Distance(Ports.PORT5)
value = distanceSensor.object_distance(INCHES)

#This sets the right distance sensor to port 3 on the brain
# this allows the robot to read the inputs on port 3
rightDistance = Distance(Ports.PORT3)
rightValue = rightDistance.object_distance(INCHES)

# This sets the left distance sensor to port 13 on the brain
# this allows the robot to read inputs on port 13
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
RobotFinished = True
manual = False
leftOrRight = 1

#robot position coordinates
xCord = 6 
yCord = 0

# This variable is set to true if space infront of the robot is clear
# This variable allows us and the robot to know whether there
#is enough safe space infront of the robot for it to move forward safely
def forwardIsClear():
    global forwardClear
    value = distanceSensor.object_distance(INCHES)
    # this lets the robot use the distance sensor to check if the front is clear
    if(value > 18):
        forwardClear = True
    # this lets the robot know that the front is not clear
    elif(value < 18):
        forwardClear = False
# This variable allows us and the robot to know whether there
#is enough safe space to the lefto the left of the robot for it to move left safely
def leftIsClear():
    global leftClear
    # This allows us to store the distance an object is detected from the 
    # left distance sensor in inches into a variable to call for later
    leftValue = leftDistance.object_distance(INCHES)
    #If the distance between the right distance sensor and
    # an obstacle is more than 12 inches then it sets the
    # right is clear to true to notify the robot it is
    # safe to go there
    if (leftValue > 12):
        leftClear = True
    #If the distance between the left distance sensor and
    # an obstacle is less than 12 inches than it sets
    # the left is clear to false to notify the robot it
    # is not safe to go there
    elif (leftValue < 12):
        leftClear = False

# This variable allows us and the robot to know whether there
#is enough safe space to the left of the robot for it to move right safely
def rightIsClear():
    global rightClear
    # This allows us to store the distance an object is detected from the
    # right distance sensor in inches into a variable to call for later
    rightValue = rightDistance.object_distance(INCHES)
    # If the distance between the right distance sensor and an obstacle
    # is more than 12 inches than it sets the right is clear to true to
    # notify the robot that it is safe to move there
    if (rightValue > 12):
        rightClear = True
    # If the distance between the right distance sensor and an obstacle
    # is less than 12 inches then it sets the right is clear to false to
    # notify the robot that it is not safe to move there
    elif(rightValue < 1):
        rightClear = False

#keeps track of robot's current position on field using coordinates (x and y)
def coordinate_tracker():
    global xCord
    global yCord
    robotAngle = inertialSensor.heading()
    #robot moved up the field 1 in the y axis
    if robotAngle <= 10 or robotAngle >= 350: 
        yCord += 1
        print("y up by one")
    #robot moved to the left side of the field 1 in the x axis
    elif robotAngle <= 80 or robotAngle >= 100:
        xCord += 1
        print("1 was added to the x-cord")
    #robot moved down the field, -1 in the y axis
    # 1 in the y axis
    elif robotAngle <= 170 or robotAngle >= 190:
        print("y down by one")
        yCord -= 1
    #robot moved to the right side of the field, -1 in 
    elif robotAngle <= 260 or robotAngle >= 280:
        xCord -= 1
        print("1 was subtracted from the x-cord")
    #The robot hasnt changed coordinates
    else:
        print("nothing ran")
        print("robot angle: ", robotAngle)
    #This is useed so we can see what coordinate the
    #robot thinks it is on mostly used for testing
    print(xCord)
    print(yCord)



        
# set this as the height of the claw to stay at
#clawHeight.spin_for(FORWARD, 20, DEGREES)
# this is the vial wideness
#wait(1, SECONDS)
#vialDetection()
# 
# 
# # this is the claw control that we must start the entire autonomous routine with             
clawHeight1.spin_for(FORWARD, 240, DEGREES)
clawHeight2.spin_for(FORWARD, 240, DEGREES)
clawControl.spin_for(REVERSE, 90, DEGREES)

# 1 checks when to start the entire autonomous routine
while (start == False):
    value = distanceSensor.object_distance(MM)
    print(value)
    wait(1, SECONDS)
    # this is the distance sensor value on the front side of the robot
    # this lets the robot use the distance sensor to check if the front is clear
    if (value < 440): 
        controller1.screen.print("Don't Drive Forward")
        wait(1, SECONDS)
        controller1.screen.clear_screen()
    # this lets the robot know that the front is not clear
    elif(value > 440):
        start = True
        RobotFinished = False
        controller1.screen.print("Drive Forward")
        wait(1, SECONDS)
        controller1.screen.clear_screen()
        drivetrain.drive_for(FORWARD, 6, INCHES)

# This makes sure that the robot is allowed to start
# the autonomous routine but has not finished yet
while(RobotFinished == False and start == True):
    # drivetrain drive forward for one y coordinate 
    forwardIsClear()
    drivetrain.set_drive_velocity(20, PERCENT)
    controller1.screen.print("started the next section")
    wait(1, SECONDS)
    controller1.screen.clear_screen()
    # This is checking if the up button is pressed
    if controller1.buttonUp.pressing():
        controller1.screen.print("manual code started")
        manual = True
        # this is the manual code that allows the robot to be controlled
        break
    # This checks if the robot is in the finishing position
    # this will stop the robot from moving
    elif (yCord == 20 and xCord == 4):
        # if the coordinates of the robot are perfectly in front of the vials
        # then we can stop the robot
        print("robot is in the right position")
        drivetrain.stop(BRAKE)
        RobotFinished = True
        start = False
        vialChecker = True
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
                # called our function to check if right side is clear
                if(rightClear == False):
                    # the right side is not clear so we just have to back up though this is very RARE!!!
                    print("not possible")
                    drivetrain.drive_for(REVERSE, 8, INCHES)
                    yCord -= 1
                    drivetrain.stop(BRAKE)
                # this means that the right side is clear
                # so we can turn to the right side
                elif(rightClear == True):
                    # the right side is clear so we turn to the right side
                    print("right side is clear")
                    drivetrain.turn_to_heading(90, DEGREES)
                    drivetrain.drive_for(FORWARD, 8, INCHES)
                    coordinate_tracker()
                    leftIsClear()
                    # we are checking to see if the left side is clear
                    # if the left side is clear, we are able to turn left and drive forward
                    if(leftClear == True):
                        # this means that the left AKA FRONT is clear
                        drivetrain.turn_to_heading(0, DEGREES)
                        drivetrain.drive_for(FORWARD, 8, INCHES)
                        coordinate_tracker()
                    # this means that the left side is not clear
                    # so we have to check if the front is clear
                    elif(leftClear == False):
                        # the front is not clear
                        forwardIsClear()
                        # we are checking to see if the front side is clear
                        if(forwardClear == True):
                            drivetrain.drive_for(FORWARD, 8, INCHES)
                            coordinate_tracker()
                        # this means that all the sides are not clear
                        elif(forwardClear == False):
                            # all sides are not clear
                            print("not possible")
            # this means that the left side is clear
            # so we can turn to the left side
            elif(leftClear == True):
                # the left side is clear so we can turn left
                print("left side is clear")
                drivetrain.turn_to_heading(-90, DEGREES)
                drivetrain.drive_for(FORWARD, 8, INCHES)
                coordinate_tracker()
                rightIsClear()
                # we are checking to see if the right side is clear
                if(rightClear == False):
                    # the right side is not clear
                    print("right side is not clear")
                    # now we have to check if the front is clear to continue driving forward
                    forwardIsClear()
                    if (forwardClear == True):
                        # if the front is clear we can drive forward again
                        print("front side is clear")
                        drivetrain.drive_for(FORWARD, 8, INCHES)                        
                        coordinate_tracker()
                    # this means that the front side is not clear
                    elif (forwardClear == False):
                        print("the FRONT side is NOT clear")
                        # the forward is not clear
                        # so we have to back up
                        print("we are stuck")
                        drivetrain.stop(BRAKE)
                # this means that the right side is clear
                elif(rightClear == True):
                    # the right side is clear so we can turn back to the front
                    print("the RIGHT side is clear so we can start again")
                    drivetrain.turn_to_heading(0, DEGREES)
                    drivetrain.drive_for(FORWARD, 8, INCHES)
                    coordinate_tracker()
    # this means that the front side is clear                
    elif(forwardClear == True):
        # the front of the robot is clear so we will continue driving forward
        print("clear")
        wait(1, SECONDS)
        forwardClear = True
        drivetrain.drive_for(FORWARD, 8, INCHES)
        coordinate_tracker()
    break

def vialDetection():
    # This makes sure that when we are checking which vial to grab
    #we take a picture of all of the availabe tags and sort through them
    #in the next statement
    while vialChecker == True:
        aivisionsensor.tag_detection(True)
        snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
        value = distanceSensor.object_distance(MM)
        drivetrain.set_drive_velocity(10, PERCENT)
        if(value < 477):
            #This is a for loop used to check every single april tag
            #that was taken inside the picture for the vials
            for obj in snapshot:                
                #This will print to the screen the tag id of the tag that was detectedn
                print("Tag detected: ", obj.id)
                #This will check if the tag is id is 9 and then run
                # the code to pick up the vial
                if obj.id == 9:
                    controller1.screen.print("correct vial detected with tag:", obj.id)
                    print("correct vial detected with tag:", obj.id)
                    drivetrain.drive_for(FORWARD, 0.5, INCHES)
                    clawControl.spin_for(FORWARD, 70, DEGREES)
                    wait(1,SECONDS)
                    clawHeight1.spin_for(FORWARD, 100, DEGREES)
                    clawHeight2.spin_for(FORWARD, 100, DEGREES)
                    wait(1, SECONDS)
                    drivetrain.drive_for(REVERSE, 5, INCHES)
                    break
                break
            break
        # This is the else statement that will run if the distance sensor is not close enough to the vials
        elif(value > 477):
            print("not ready to check")
            drivetrain.drive_for(FORWARD, 2, INCHES)

#Should work needs testing
#function for dropping the vial in the
def dropOff():
    # we use the start_awb in order to cancel out the yellowness with whiteness in the ai vision sensor 
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
        # this is the distance sensor value on the front side of the robot
        # this lets the robot use the distance sensor to check if it is close enough
        # to the tube
        if (value < 600 and value > 590):
            #the robot's distance sensor will detect the distance between it and the wall and will stop drivetrain.stop(BRAKE)
            if len(tubeColor) >= 1:
            
                #the if statement will run if purple color is detected by the AI vision sensor    print("tube detected")
                controller1.screen.print("tube detected")
                clawHeight1.spin_for(FORWARD, 160, DEGREES)
                clawHeight2.spin_for(FORWARD, 160, DEGREES)
                drivetrain.drive_for(FORWARD, 6.25, INCHES)
                wait(1, SECONDS)
                clawHeight1.set_velocity(10, PERCENT)
                clawHeight2.set_velocity(10, PERCENT)
                clawHeight1.spin_for(REVERSE, 60, DEGREES)
                clawHeight2.spin_for(REVERSE, 60, DEGREES)
                wait(1, SECONDS)
                clawControl.spin_for(REVERSE, 60, DEGREES)
                drivetrain.drive_for(REVERSE, 3, INCHES)
        elif(value > 600):
            break
            #if the distance sensor doesn't detect a wall in 600 mm then the robot will continue drivingdrivetrain.drive(FORWARD)



#this is code that will run if the vial is there and we need to drop it off in the dissemination chamber
#clawControl.sp#in_for(FORWARD, 75, DEGREES)
#clawHeight.spi#n_for(FORWARD, 240, DEGREES)
#drivetrain.set#drivetrain.set_heading(0, DEGREES
#dropOff()
while manual == True:
    if controller1.buttonA.pressing():
        # this is the manual code that allows the robot to be controlled
        print("manaual vial detection")
        # This makes sure that when we are checking which vial to grab
        aivisionsensor.tag_detection(True)
        snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
        for obj in snapshot:                
            #This will print to the screen the tag id of the tag that was detectedn
            controller1.screen.print("Tag detected: ", obj.id)
            #This will check if the tag is id is 9 and then run
            # the code to pick up the vial
            if obj.id == 9:
                controller1.screen.print("correct vial detected with tag:", obj.id)
    elif controller1.buttonB.pressing():
        # this is the manual code that allows the robot to be controlled
        print("manaual tube detection")
        # This makes sure that when we are checking which vial to grab
        aivisionsensor.color_detection(True)
        tubeColor = aivisionsensor.take_snapshot(purple)
        if len(tubeColor) >= 1:
            #if statement will run the AI vision sensor detects the color purple   
            print("tube detected and it is purple!")
            controller1.screen.print("tube detected and it is purple!")
    elif controller1.buttonX.pressing():
        print("manual lever april tag detection")
        aivisionsensor.tag_detection(True)
        snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
        for obj in snapshot:
            controller1.screen.print("Tag detected: ", obj.id)
            # This will check if the tag is id is 5 and then run
            if obj.id == 5:
                print("correct lever signal detected with tag:", obj.id)
                controller1.screen.print("correct lever signal with tag:", obj.id)
                
                    
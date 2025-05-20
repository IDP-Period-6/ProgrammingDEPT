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

# ai vision sensor code
purple = Colordesc(1, 214, 72, 219, 10, 0.2)
aivisionsensor = AiVision(Ports.PORT14, AiVision.ALL_TAGS, purple)
snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)

# inertial sensor code
inertialSensor = Inertial(Ports.PORT4)
inertialSensor.set_heading(0, DEGREES)
inertialSensor.calibrate()
while inertialSensor.is_calibrating():
    wait(100, MSEC)



# distance sensor code
distanceSensor = Distance(Ports.PORT5)
value = distanceSensor.object_distance(INCHES)

rightDistance = Distance(Ports.PORT3)
rightValue = rightDistance.object_distance(INCHES)

leftDistance = Distance(Ports.PORT13)
leftValue = leftDistance.object_distance(INCHES)


# drivetrain code
leftMotor1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1,)
leftMotor2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, )

leftMotor1.set_reversed(True)
leftMotor2.set_reversed(True)


rightMotor1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1,  )
rightMotor2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1,  )


clawHeight = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
clawControl = Motor(Ports.PORT15, GearSetting.RATIO_18_1)

leftMotors = MotorGroup(leftMotor1, leftMotor2, )
rightMotors = MotorGroup(rightMotor1, rightMotor2, )

drivetrain = SmartDrive(leftMotors, rightMotors, inertialSensor)
#endregion 

#global variables for running
vialChecker = False
distanceChecker = False
tubeChecker = True
start = False 

# actually used variables
forwardClear = False
leftClear = False
rightClear = False
RobotFinished = True
leftOrRight = 1
start = True

xCord = 6 
yCord = 0

def forwardIsClear():
    global forwardClear
    value = distanceSensor.object_distance(INCHES)
    if(value > 13.5):
        forwardClear = True
    elif(value < 13.5):
        forwardClear = False

def leftIsClear():
    global leftClear
    leftValue = leftDistance.object_distance(INCHES)
    if (leftValue > 7):
        leftClear = True
    elif(leftValue < 7):
        leftClear = False

def rightIsClear():
    global rightClear
    rightValue = rightDistance.object_distance(INCHES)
    if (rightValue > 7):
        rightClear = True
    elif(rightValue < 7):
        rightClear = False

def coordinate_tracker():
    global xCord
    global yCord
    if inertialSensor.heading() <= 10 and inertialSensor.heading() >= 350: 
        yCord += 1
    elif inertialSensor.heading() <= 80 and inertialSensor.heading() >= 100:
        xCord += 1
    elif inertialSensor.heading() <= 170 and inertialSensor.heading() >= 190:
        yCord -= 1
    elif inertialSensor.heading() <= 260 and inertialSensor.heading() >= 280:
        xCord -= 1
    print(xCord)
    print(yCord)

drivetrain.set_drive_velocity(10, PERCENT)
def vialDetection():
    while vialChecker == True:
        aivisionsensor.tag_detection(True)
        snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
        value = distanceSensor.object_distance(MM)
        print("hello")
        if(value < 477):
            for obj in snapshot:
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
clawControl.spin_for(FORWARD, 60, DEGREES)

clawHeight.spin_for(FORWARD, 255, DEGREES)
# this is the vial wideness
# clawControl.spin_for(REVERSE, 55, DEGREES)
# wait(1, SECONDS)
# vialDetection()




# 1 checks when to start the entire autonomous routine
while start == False:
    value = distanceSensor.object_distance(INCHES)
    if (value < 12): 
        print("Don't Drive Forward")
    elif(value > 12):
        print("Drive Forward")
        drivetrain.drive_for(84, INCHES)
        drivetrain.turn_to_heading(-90, DEGREES)
        start = True		


while(RobotFinished == False and start == True):
    # drivetrain drive forward for one y coordinate 
    forwardIsClear()
    if (yCord == 6 and xCord == 8):
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
        drivetrain.drive_for(6, INCHES)
        coordinate_tracker()
        

                



while distanceChecker == True:
    value = distanceSensor.object_distance(INCHES)
    if value <= 3:
        brain.screen.set_cursor(1,2)
        print("DO NOT START")
        wait(1, SECONDS)
        brain.screen.clear_screen()
    elif value > 3:
        brain.screen.set_cursor(20,1)
        print("START NOW!")
        wait(1, SECONDS)
        brain.screen.clear_screen()

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




def dropOff():
    while tubeChecker == True:
        tubeColor = aivisionsensor.take_snapshot(purple)
        value = distanceSensor.object_distance(MM)
        if (value < 427):
            if len(tubeColor) >= 1:
                print("tube detected")
                clawHeight.spin_for(FORWARD, 140, DEGREES)
                drivetrain.drive_for(FORWARD, 4, INCHES)
                wait(1, SECONDS)
                clawHeight.set_velocity(10, PERCENT)
                clawHeight.spin_for(REVERSE, 60, DEGREES)
                wait(1, SECONDS)
                clawControl.spin_for(REVERSE, 60, DEGREES)
                drivetrain.drive_for(REVERSE, 3, INCHES)
        elif(value > 428):
            drivetrain.drive_for(FORWARD, 1, MM)
dropOff()
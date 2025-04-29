# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       201131602                                                    #
# 	Created:      4/10/2025, 3:20:43 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

# ai vision sensor code
aivisionsensor = AiVision(Ports.PORT20, AiVision.ALL_TAGS)
snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)

# inertial sensor code
# inertialSensor = Inertial(Ports.PORT10)
# inertialSensor.calibrate()
# while inertialSensor.is_calibrating():
#     wait(100, MSEC)

# distance sensor code
distanceSensor = Distance(Ports.PORT19)
value = distanceSensor.object_distance(INCHES)

# drivetrain code
leftMotor1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1)
leftMotor2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1)
rightMotor1 = Motor(Ports.PORT4, GearSetting.RATIO_18_1)
rightMotor2 = Motor(Ports.PORT5, GearSetting.RATIO_18_1)
leftMotors = MotorGroup(leftMotor1, leftMotor2)
rightMotors = MotorGroup(rightMotor1, rightMotor2)
# drivetrain = SmartDrive(leftMotors, rightMotors, Inertial, 319.19, 295, 40, MM) # change this

#global variables for running
aiChecker = True
distanceChecker = False

# actually used variables
fowardCheck = False
RobotFinished = True
leftOrRight = 1
startOrStop = False
xCord = 6
yCord = 0



while startOrStop == True:
    value = distanceSensor.object_distance(INCHES)
    if (value <= 3): 
        print("Drive Forward")
    elif(value > 3):
        print("Don't Drive Forward")
        startOrStop = True		
          
def forwardIsClear():
    if(value > 5):
        forwardCheck = True
    elif(value < 5):
        forwardCheck = False

while (RobotFinished == False and startOrStop == True):
    forwardCheck = False
    forwardIsClear()
    if(forwardCheck == False):
	    yCord += 1

while aiChecker == True:
    snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
    for obj in snapshot:
        brain.screen.set_cursor(1, 1)
        brain.screen.print("Tag detected: ", obj.id)
        wait(0.5, SECONDS)
        brain.screen.clear_screen()

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
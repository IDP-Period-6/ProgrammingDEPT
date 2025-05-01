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
aivisionsensor = AiVision(Ports.PORT20, AiVision.ALL_TAGS)
purple = Colordesc(1, 214, 72, 219, 10, 0.2)
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
leftMotor1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
# leftMotor2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1)
rightMotor1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
# rightMotor2 = Motor(Ports.PORT5, GearSetting.RATIO_18_1)

clawHeight = Motor(Ports.PORT4, GearSetting.RATIO_18_1)
clawControl = Motor(Ports.PORT3, GearSetting.RATIO_18_1)

leftMotors = MotorGroup(leftMotor1)
rightMotors = MotorGroup(rightMotor1)

drivetrain = DriveTrain(leftMotors, rightMotors)
# drivetrain = SmartDrive(leftMotors, rightMotors, Inertial, 319.19, 295, 40, MM) # change this
#endregion 

drivetrain.set_drive_velocity(5, PERCENT)
drivetrain.set_turn_velocity(5, PERCENT)

# drivetrain.drive(FORWARD)
# wait(1, SECONDS)
# drivetrain.turn_for(RIGHT, 90, DEGREES)

#global variables for running
vialChecker = True
distanceChecker = False
tubeChecker = False

# actually used variables
fowardCheck = False
RobotFinished = True
leftOrRight = 1
start = False
xCord = 6
yCord = 0


 #’open claw’


# checks when to start the entire autonomous routine
while start == True:
    value = distanceSensor.object_distance(INCHES)
    if (value <= 3): 
        print("Drive Forward")
    elif(value > 3):
        print("Don't Drive Forward")
        start = True		
          
def forwardIsClear():
    if(value > 5):
        forwardCheck = True
    elif(value < 5):
        forwardCheck = False

while(RobotFinished == False and start == True):
    forwardCheck = False
    forwardIsClear()
    if(forwardCheck == False):
	    yCord += 1

while vialChecker == True:
    aivisionsensor.tag_detection(True)
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
            clawControl.spin_for(REVERSE, 90, DEGREES)        # add the claw movements here to make the lever clos



def dropOff():
    while tubeChecker == True:
        aivisionsensor.color_detection(True)
        tubeColor = aivisionsensor.take_snapshot(purple)
        if len(tubeColor) >= 1:
            clawHeight.spin(FORWARD) #’bring the arm down’
            clawControl.spin_for(REVERSE, 90, DEGREES) #’open claw’
            clawControl.spin_for(FORWARD, 90, DEGREES) #’close claw’
            clawHeight.spin(REVERSE) #’bring claw back up’
            #waitForLever()

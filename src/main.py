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
aivisionsensor = AiVision(Ports.PORT1, AiVision.ALL_TAGS)
snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)

def imgthread():
    # function will run in a separate thread so it can continue running in the background
    while True:
        global snapshot
        snapshot = aivisionsensor.take_snapshot(AiVision.ALL_TAGS)
        wait(50)
thread = Thread(imgthread)

while True:
    for obj in snapshot:
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("Tag detected: ", obj.id)
        wait(0.5, SECONDS)
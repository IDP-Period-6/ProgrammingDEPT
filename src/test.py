# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       test.py                                                      #
# 	Author:       201131602                                                    #
# 	Created:      4/10/2025, 3:20:43 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Code will be uploaded to a MicroSD card to insert into the brain
# Do not upload this code to the brain directly
def testcode():
    brain = Brain()
    brain.screen.print("This is the test function being called")

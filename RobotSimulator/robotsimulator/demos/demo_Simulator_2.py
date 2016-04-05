from math import *

from robotsimulator import Robot
from robotsimulator import World
from robotsimulator.worlds import officeWorld


# Roboter in Office-World positionieren:
myWorld = officeWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 2, 5.5, pi/2)

# CursorController definieren:
myCursorController = myWorld.getCursorController()


# Bewege Roboter mit Cursor-Tasten:
while True:
    motion = myCursorController.getSpeed()
    if motion == None:
        break
    print("v = ", motion[0], "omega = ", motion[1]*180/pi)
    print("sense = ", myRobot.sense())
    print("boxes = ", myRobot.senseBoxes())
    myRobot.move(motion)

# Simulation schliessen:
myWorld.close(False)
from math import *

from numpy import math

from robotsimulator import Robot
from robotsimulator.World import World


# Roboter in einer Welt positionieren:
myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 5, 12, 0)
myRobot.deactivateMotionNoise()

v = 1
v2 = 1
myRobot.driveCircle(v, 3, 1)

myRobot.straightDrive(v, 3)
myRobot.driveChangeLane(v, 1, math.pi / 4, 1)
myRobot.straightDrive(v, 3)
myRobot.driveChangeLane(v, 1, -math.pi / 4, 1)
myRobot.straightDrive(v, 1)

myRobot.driveRectangle(v, 2, 2)



# Simulation schliessen:
myWorld.close()

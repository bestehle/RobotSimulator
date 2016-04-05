from math import *

from robotsimulator import Robot
from robotsimulator import World
from robotsimulator.worlds import emptyWorld


# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myRobot.deactivateMotionNoise()
myWorld.setRobot(myRobot, 5, 8, pi / 4)

myRobot.straightDrive(1, 10)


# Simulation schliessen:
myWorld.close()
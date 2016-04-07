from math import *

from numpy import math

from robotsimulator import Robot
from robotsimulator.World import World


# Roboter in einer Welt positionieren:
myWorld = World(20, 20)
myRobot = Robot.Robot()
# myRobot.deactivateMotionNoise()
myWorld.setRobot(myRobot, 1, 1, math.pi / 4)

v = 1
v2 = 1

# myRobot.straightDrive(v, 15)
myRobot.straightDriveTruePose(v, 15)



# myRobot.driveChangeLane(v, 1, math.pi / 4, 1)
# myRobot.straightDrive(v, 3)
# myRobot.driveChangeLane(v, 1, -math.pi / 4, 1)
# myRobot.straightDrive(v, 3)
# 
# myRobot.curveDrive(v, 1, -math.pi)
# myRobot.straightDrive(v, 3)
# 
# myRobot.driveRectangle(v, 5, 2)
# myRobot.driveChangeLane(v, 1, math.pi / 4, 1)
# myRobot.driveRectangle(v, 5, 2)
# myRobot.driveChangeLane(v, 1, math.pi / 4, 1)
# myRobot.driveRectangle(v, 5, 2)
# myRobot.driveChangeLane(v, 1, math.pi / 4, 1)
# 
# myRobot.curveDrive(v, 1, -math.pi / 2)
# myRobot.straightDrive(v, 6)


# Simulation schliessen:
myWorld.close()

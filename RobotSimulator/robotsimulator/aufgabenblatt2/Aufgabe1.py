from math import *

from numpy import math

from robotsimulator import Robot
from robotsimulator.World import World


# Roboter in einer Welt positionieren:
myWorld = World(20, 20)
myRobot = Robot.Robot()
# myWorld.addLine(0, 17, 3, 20)
# myWorld.addLine(17, 20, 20, 17)
# myWorld.addLine(0, 3, 3, 0)
# myWorld.addLine(17, 0, 20, 3)

myWorld.addLine(10, 0, 13, 3)
myWorld.addLine(10, 6, 13, 3)
myWorld.addLine(10, 17, 13, 20)
myWorld.addLine(10, 17, 13, 14)
myWorld.addLine(12, 8, 8, 12)
myWorld.addLine(8, 8, 12, 12)

myWorld.setRobot(myRobot, 6, 10, 0 * math.pi)
# myRobot.deactivateMotionNoise()

v = 0.5
v2 = 1
myRobot.braitenberg(v2, 6, 2)



# Simulation schliessen:
myWorld.close()

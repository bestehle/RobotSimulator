
import math

from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


# Roboter in einer Welt positionieren:
myWorld = World(20, 20)
# myWorld.addLine(0, 0, 20, 20)

myRobot = Robot.Robot()
# myRobot.deactivateMotionNoise()
myWorld.setRobot(myRobot, 6, 2, math.pi / 4)

v = 1

myRobot.followLineP(v, 0.2, Point(0, 0), Point(20, 20))




# Simulation schliessen:
myWorld.close()

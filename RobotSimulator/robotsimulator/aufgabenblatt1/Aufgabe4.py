
import math

from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


# Roboter in einer Welt positionieren:
myWorld = World(20, 20)
# myWorld.addLine(0, 0, 20, 20)

myRobot = Robot.Robot()
# myRobot.deactivateMotionNoise()
myWorld.addBox(5, 5)
myWorld.setRobot(myRobot, 6, 12, math.pi / 4)

v = 1
p = Point(5, 5)
tol = 0.2

# myRobot.goto(v, p, tol)
myRobot.goto(v, p, tol)

# Simulation schliessen:
myWorld.close()

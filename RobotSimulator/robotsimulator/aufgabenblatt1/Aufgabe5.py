
import math

from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


# Roboter in einer Welt positionieren:
myWorld = World(30, 30)
# myWorld.addLine(0, 0, 20, 20)

myRobot = Robot.Robot()
# myRobot.deactivateMotionNoise()
myWorld.setRobot(myRobot, 6, 2, math.pi / 4)

v = 1
poly = [Point(2, 10), Point(20, 10), Point(20, 5), Point(25, 5)]
for x in poly :
    myWorld.addBox(x.getX(), x.getY())

myRobot.followPolyline(v, poly)



# Simulation schliessen:
myWorld.close()

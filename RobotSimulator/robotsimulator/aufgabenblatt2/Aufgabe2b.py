
from numpy import math

from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


# Roboter in einer Welt positionieren:
myWorld = World(30, 30)
myRobot = Robot.Robot()

myWorld.addLine(10, 0, 13, 3)
myWorld.addLine(10, 6, 13, 3)
myWorld.addLine(10, 17, 13, 20)
myWorld.addLine(10, 17, 13, 14)
myWorld.addLine(12, 8, 8, 12)
# myWorld.addLine(8, 8, 12, 12)
myWorld.addLine(23, 0, 23, 12)
myWorld.addLine(23, 12, 24, 12)
myWorld.addLine(24, 0, 24, 12)

poly = [Point(2, 10), Point(20, 10), Point(20, 5), Point(25, 7), Point(2, 25)]
polyline = [[15, 12], [2, 10], [20, 10], [20, 5], [25, 7], [2, 25]]
for x in poly :
    myWorld.addBox(x.getX(), x.getY())
    
myWorld.drawPolyline(polyline)

myWorld.setRobot(myRobot, 15, 12, math.pi)
# myRobot.deactivateMotionNoise()

v = 0.5
v2 = 1
myRobot.followPolylineWithObstacle(v2, poly, 6, 2)



# Simulation schliessen:
myWorld.close()

from numpy import math
from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


myWorld = World(30, 30)
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 6, 2, math.pi / 4)

v = 1
polyline = [Point(2, 10), Point(20, 10), Point(20, 5), Point(25, 5)]
for x in polyline:
    myWorld.addBox(x.getX(), x.getY())

myRobot.followPolylineWithObstacleAvoidance(v, polyline)

myWorld.close()

from numpy import math
from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


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

polyline = [Point(2, 10), Point(20, 10), Point(20, 5), Point(25, 7), Point(2, 25)]
for x in polyline :
    myWorld.addBox(x.getX(), x.getY())
    
myWorld.drawPolyline(polyline)
myWorld.setRobot(myRobot, 15, 12, math.pi)
# myRobot.deactivateMotionNoise()

v = 0.5
v2 = 1
myRobot.followPolylineWithObstacleAvoidance(v2, polyline, 6, 2)

myWorld.close()

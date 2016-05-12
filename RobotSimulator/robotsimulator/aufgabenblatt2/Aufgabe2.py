<<<<<<< Upstream, based on origin/master
from robotsimulator.worlds import obstacleWorld2 as obstacleWorld
from robotsimulator import Robot

myWorld = obstacleWorld.buildWorld()
polyline = []
for i in range(1, 18):
    polyline.append([i, 5])

myWorld.drawPolyline(polyline)

myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 5, 0)

myRobot.followPolylineAvoidCollision(1, polyline)

=======

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

poly = [Point(2, 10), Point(20, 10), Point(20, 5), Point(25, 5)]
for x in poly :
    myWorld.addBox(x.getX(), x.getY())

myWorld.setRobot(myRobot, 6, 10, 0 * math.pi)
# myRobot.deactivateMotionNoise()

v = 0.5
v2 = 1
myRobot.followPolylineWithObstacle(v, poly, 6, 2)



# Simulation schliessen:
myWorld.close()
>>>>>>> 3df00cd 2.2) fixed sensorData adn add followPolylineWithObstacle

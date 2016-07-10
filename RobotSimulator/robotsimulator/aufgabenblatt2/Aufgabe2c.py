from robotsimulator import Robot
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import obstacleWorld3 as obstacleWorld


myWorld = obstacleWorld.buildWorld()
polyline = [Point(1, 6), Point(10, 6), Point(10.5, 3)]
myWorld.drawPolyline(polyline)

myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 6, 0)

myRobot.followPolylineWithObstacleAvoidance(1, polyline, 6, 1)

myWorld.close()
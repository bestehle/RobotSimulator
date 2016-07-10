from robotsimulator import Robot
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import obstacleWorld2 as obstacleWorld


myWorld = obstacleWorld.buildWorld()
polyline = [Point(1, 5), Point(18, 5)]
myWorld.drawPolyline(polyline)

myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 5, 0)

myRobot.followPolylineWithObstacleAvoidance(1, polyline, 6, 1)

myWorld.close()
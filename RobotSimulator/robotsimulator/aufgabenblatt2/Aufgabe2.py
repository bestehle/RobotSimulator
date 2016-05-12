from robotsimulator import Robot
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import obstacleWorld2 as obstacleWorld


myWorld = obstacleWorld.buildWorld()
# polyline = []
# poly = []
# for i in range(1, 18):
#     poly.append(Point(i, 5))
#     polyline.append([i, 5])

polyline = [[1, 5], [18, 5]]
poly = [Point(1, 5), Point(18, 5)]
# for i in range(1, 18):
#     poly.append(Point(i, 5))
#     polyline.append([i, 5])

myWorld.drawPolyline(polyline)

myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 5, 0)

myRobot.followPolylineWithObstacle(1, poly, 6, 1)


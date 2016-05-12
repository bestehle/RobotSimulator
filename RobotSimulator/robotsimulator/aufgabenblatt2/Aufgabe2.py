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


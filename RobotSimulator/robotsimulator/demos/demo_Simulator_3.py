from math import pi
from robotsimulator import Robot
from robotsimulator.worlds import obstacleWorld3
from robotsimulator.graphics.graphics import Point


# Roboter in obstacleWorld3 positionieren:
myWorld = obstacleWorld3.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 1, 6, 0)

# Polygonzug, der abgefahren werden soll, einzeichnen:
polyline = [Point(1,6), Point(9.5,6), Point(10.5,3)]
myWorld.drawPolyline(polyline)


# CursorController definieren:
myCursorController = myWorld.getCursorController()


# Bewege Roboter mit Cursor-Tasten:
while True:
    motion = myCursorController.getSpeed()
    if motion == None:
        break
    print("v = ", motion[0], "omega = ", motion[1]*180/pi)
    myRobot.move(motion)

# Simulation schliessen:
myWorld.close(False)
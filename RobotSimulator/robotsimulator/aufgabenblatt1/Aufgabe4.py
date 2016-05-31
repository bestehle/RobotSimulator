from numpy import math
from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld.addBox(5, 5)
myWorld.setRobot(myRobot, 6, 12, math.pi / 4)

v = 1
p = Point(5, 5)
tol = 0.2

myRobot.goto(v, p, tol)
myWorld.close()
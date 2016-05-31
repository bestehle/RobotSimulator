from numpy import math
from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


myWorld = World(100, 100)

myRobot = Robot.Robot()
myRobot.deactivateMotionNoise()
myWorld.setRobot(myRobot, 1, 2, math.pi * 0)

v = 2

myRobot.followLineP(v, 0.3, Point(0, 0), Point(20, 20))
# myRobot.followLinePD(v, 0.2, Point(0, 0), Point(20, 20))

myWorld.close()

from numpy import math

from robotsimulator import Robot
from robotsimulator.Localisation import Localistaion
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import officeWorld


myWorld = World(20, 20)
myRobot = Robot.Robot()

myWorld.setRobot(myRobot, 6, 7, 0 * math.pi)
myWorld.addLine(4, 8, 4, 20)
myWorld.addLine(4, 4, 8, 4)

loc = Localistaion(myRobot, myWorld)
loc.localizeObstacles(myRobot.getTrueRobotPose())

particles = loc.generateParticles(myRobot.getTrueRobotPose(), 50)

for particle in particles:
    myWorld.addBox(particle[0], particle[1])
    myWorld._udateWindow()
    loc.localizeObstacles([particle[0], particle[1], particle[2]])

myWorld.close()

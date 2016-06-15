from numpy import math

from robotsimulator import Robot
from robotsimulator.Localisation import Localisation
from robotsimulator.worlds import officeWorld

myWorld = officeWorld.buildWorld()
myRobot = Robot.Robot()

myWorld.setRobot(myRobot, 6, 7, 0 * math.pi)

localisation = Localisation(myRobot, myWorld)
myRobot.onMove(localisation.check)

myRobot.braitenberg(0.5, 6, 3)

myWorld.close()

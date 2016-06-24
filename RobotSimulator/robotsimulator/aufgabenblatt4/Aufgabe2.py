from numpy import math

from robotsimulator import Robot
from robotsimulator.Localisation import Localisation
from robotsimulator.worlds import officeWorld

myWorld = officeWorld.buildWorld()
myRobot = Robot.Robot()

myWorld.setRobot(myRobot, 6, 7, 0 * math.pi)

localisation = Localisation(myRobot, myWorld)
localisation._grid.addSafetyDistance(myRobot, 0.1)
localisation._grid.brushfire()

localisation.addLandmark(1, 1)
localisation.addLandmark(18, 13)
#localisation.addLandmark(1, 13)
#localisation.addLandmark(18, 1)
myWorld.addBox(1, 1)
myWorld.addBox(18, 13)
myWorld.addBox(18, 1)
myWorld.addBox(1, 13)

myRobot.onMove(localisation.check)

myRobot.braitenberg(0.5, 6, 3)

myWorld.close()

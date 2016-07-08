from numpy import math

from robotsimulator import Robot
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator.World import World
from robotsimulator.worlds import officeWorldWithGlobalGrid as officeWorld
from robotsimulator.LocalisationGlobal import LocalisationGlobal as Localisation


myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld = officeWorld.buildWorld()

myWorld.setRobot(myRobot, 4, 6.5, 0 * math.pi)

pathPlanning = PathPlanning(myRobot, myWorld)
pathPlanning._grid.addSafetyDistance(myRobot, 0.1)
pathPlanning._grid.brushfire()

localisation = Localisation(myRobot, myWorld, numberOfParticles=1000)

myRobot.onMove(localisation.check)

#myRobot.useApproximatePosition(localisation.getPosition)

myRobot.braitenberg(0.6, 9, 2, 0.5)

myWorld.close()


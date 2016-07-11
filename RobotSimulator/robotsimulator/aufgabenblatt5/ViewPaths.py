from numpy import math

from robotsimulator import Robot
from robotsimulator.DrawHelper import DrawHelper
from robotsimulator.LocalisationLikelihood import LocalisationLikelihood as Localisation
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator.PlanByPermutations import PlanByPermutations as Planner
from robotsimulator.World import World
from robotsimulator.worlds import officeWorldWithDynObstacles as officeWorld


# from robotsimulator.LocalisationLandmarks import LocalisationLandmarks as Localisation
myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld = officeWorld.buildWorld()

myWorld.setRobot(myRobot, 2, 7, 0 * math.pi)

draw = DrawHelper(myWorld)

pathPlanning = PathPlanning(myRobot, myWorld)
pathPlanning._grid.numberOfNeighbors = 8
pathPlanning._grid.addSafetyDistance(myRobot, 0.09)
pathPlanning._grid.brushfire()

localisation = Localisation(myRobot, myWorld)
localisation.drawParticles = True
localisation.printFault = True
localisation.printPosition = True
localisation.addLandmark(1, 1)
localisation.addLandmark(18, 13)
localisation.addLandmark(1, 13)

myRobot.onMove(localisation.check)
myRobot.useApproximatePosition(localisation.getPosition)

planner = Planner(myWorld)

paths = planner.getAllPaths()
for path in paths:
    draw.permanentPolyline(path)
    
path = planner.printablePath()
draw.permanentPolyline(path, 'red')

myWorld.close()

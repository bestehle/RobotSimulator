from numpy import math

from robotsimulator import GeometryHelper
from robotsimulator import Robot
from robotsimulator.BoxDetection import BoxDetection
from robotsimulator.DrawHelper import DrawHelper
from robotsimulator.LocalisationLikelihood import LocalisationLikelihood as Localisation
# from robotsimulator.LocalisationLandmarks import LocalisationLandmarks as Localisation
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator.PlanByPermutations import PlanByPermutations as Planner
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point
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
# pathPlanning._grid.drawGrid()

localisation = Localisation(myRobot, myWorld)
localisation.drawParticles = True
localisation.printFault = True
localisation.printPosition = True
localisation.addLandmark(1, 1)
localisation.addLandmark(5, 7)
localisation.addLandmark(18, 13)
localisation.addLandmark(1, 13)

boxDetector = BoxDetection(myWorld, myRobot)

myRobot.onMove(localisation.check)
myRobot.useApproximatePosition(localisation.getPosition)

planner = Planner(myWorld)

# settings for polyline follow
v = 1
sensorsToUse = 9
sensorMaxDistance = 1
avoidDistance = 0.6
tol = 0.5

# visit all rooms
while (not planner.roomsVisited()):
    (startX, startY, _) = localisation.getPosition()
    (room, roomX, roomY) = planner.nextRoom()
    start = (startX, startY)
    goal = (roomX, roomY)
    # plan shortest path
    path = pathPlanning.shortestPath(start, goal)
    path = pathPlanning.rdp(path, 0.1)
    distance = GeometryHelper.pathDistance(path)
    draw.polyline(path)
    polyline = list(map(lambda point : Point(point[0], point[1]), path))

    myRobot.followPolylineWithObstacleAvoidance(v, polyline, sensorsToUse, sensorMaxDistance, avoidDistance, tol)   
    boxDetector.findBoxes()

myWorld.close()

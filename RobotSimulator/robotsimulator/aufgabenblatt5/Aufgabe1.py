from sys import maxsize

from numpy import math

from robotsimulator import GeometryHelper
from robotsimulator import Robot
#from robotsimulator.LocalisationLandmarks import LocalisationLandmarks as Localisation
from robotsimulator.LocalisationLikelihood import LocalisationLikelihood as Localisation
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import officeWorldWithDynObstacles as officeWorld


myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld = officeWorld.buildWorld()

myWorld.setRobot(myRobot, 2, 7, 0 * math.pi)

pathPlanning = PathPlanning(myRobot, myWorld)
pathPlanning._grid.numberOfNeighbors = 8
pathPlanning._grid.addSafetyDistance(myRobot, 0.1)
pathPlanning._grid.brushfire()

localisation = Localisation(myRobot, myWorld)
localisation.addLandmark(1, 1)
localisation.addLandmark(18, 13)
localisation.addLandmark(1, 13)

myRobot.onMove(localisation.check)
myRobot.useApproximatePosition(localisation.getPosition)

rooms = myWorld.getRooms()

# visit all rooms
while (len(rooms) > 0):
    maxDistance = maxsize
    # get the distance to each room
    for (room, x, y) in rooms:
        (posX, posY, _) = localisation.getPosition()
        start = (posX, posY)
        goal = (x, y)
        # print ("check the distance from " , start , " to ", goal)
        path = pathPlanning.shortestPath(start, goal)
        if (None == path): continue;
        path = pathPlanning.rdp(path, 0.1)
        distance = GeometryHelper.pathDistance(path)
        # check if distance to this room is next
        if (distance < maxDistance):
            maxDistance = distance
            nextRoom = [room, x, y]
            nextPath = path
            nextDistance = distance
    # visit the next room
    rooms.remove(nextRoom)
    (room, x, y) = nextRoom
    print ("Visit : " , room, " : (", x, ",", y, ")")
    
    polyline = []
    for (x, y) in nextPath:
        polyline.append(Point(x, y))
    myWorld.drawPolyline(polyline, color='green')

    myRobot.followPolylineWithObstacle(1, polyline, 9, 0.6, 0.5)
    
    myRobot.findBoxes()

myWorld.close()

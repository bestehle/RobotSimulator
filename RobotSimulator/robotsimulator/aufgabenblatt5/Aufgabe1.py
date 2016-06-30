from numpy import math

from sys import maxsize
from robotsimulator import Robot
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator import GeometryHelper
from robotsimulator.Localisation import Localisation
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import officeWorldWithDynObstacles as officeWorld


myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld = officeWorld.buildWorld()

myWorld.setRobot(myRobot, 11, 11.5, 1 * math.pi)
start = (11, 11.5)
goal = (15, 6)

pathPlanning = PathPlanning(myRobot, myWorld)
pathPlanning._grid.addSafetyDistance(myRobot, 0.1)
pathPlanning._grid.brushfire()

localisation = Localisation(myRobot, myWorld)
localisation.addLandmark(1, 1)
localisation.addLandmark(18, 13)
localisation.addLandmark(1, 13)
#localisation.addLandmark(18, 1)

myRobot.onMove(localisation.check)
myRobot.useApproximatePosition(localisation.getPosition)

rooms = myWorld.getRooms()
print (rooms)
# visit all rooms
while (len(rooms) > 0):
    maxDistance = maxsize
    # get the distance to each room
    for (room, x, y) in rooms:
        print (room)
        (posX, posY, _) = localisation.getPosition()
        start = (posX, posY)
        goal = (x, y)
        #print ("check the distance from " , start , " to ", goal)
        path = pathPlanning.shortestPath(start, goal)
        if (None == path): continue;
        path = pathPlanning.rdp(path, 0.1)
        distance = GeometryHelper.pathDistance(path)
        # check if distance to this room is next
        if (room == "Room 03"):
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

    myRobot.followPolylineWithObstacle(0.2, polyline, 6, 0.8, 0.5)

myWorld.close()

from numpy import math

from sys import maxsize
from robotsimulator import Robot
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator import GeometryHelper
from robotsimulator.Localisation import Localisation
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import officeWorld


myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld = officeWorld.buildWorld()

myWorld.setRobot(myRobot, 2, 7, 0 * math.pi)
start = (2, 7)
goal = (15, 6)

pathPlanning = PathPlanning(myRobot, myWorld)
pathPlanning._grid.addSafetyDistance(myRobot, 0.1)
pathPlanning._grid.brushfire()

localisation = Localisation(myRobot, myWorld)

myRobot.onMove(localisation.check)

rooms = myWorld.getRooms()
print (rooms)
# visit all rooms
while (len(rooms) > 0):
    maxDistance = maxsize
    # get the distance to each room
    for (room, x, y) in rooms:
        start = localisation.getPosition()
        goal = (x, y)
        #print ("check the distance from " , start , " to ", goal)
        path = pathPlanning.shortestPath(start, goal)
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

    myRobot.followPolylineWithObstacle(0.2, polyline, 6, 0.1, 0.5)

myWorld.close()

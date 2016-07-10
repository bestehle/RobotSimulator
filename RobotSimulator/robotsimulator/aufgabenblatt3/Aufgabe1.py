from numpy import math

from robotsimulator import Robot
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import officeWorld
from robotsimulator.DrawHelper import DrawHelper


myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld = officeWorld.buildWorld()

draw = DrawHelper(myWorld)

myWorld.setRobot(myRobot, 2, 7, 0 * math.pi)
start = (2, 7)
goal = (15, 6)

pathPlanning = PathPlanning(myRobot, myWorld)
pathPlanning._grid.numberOfNeighbors = 8
pathPlanning._grid.addSafetyDistance(myRobot, 0.1)
pathPlanning._grid.brushfire()
pathPlanning._grid.drawGrid()

path = pathPlanning.shortestPath(start, goal)
draw.permanentPolyline(path, color='red')
path = pathPlanning.rdp(path, 0.25)
draw.permanentPolyline(path, color='green')
polyline = []
for point in path:
    polyline.append(Point(point[0], point[1]))


myRobot.followPolylineWithObstacle(v=1, poly=polyline, sensorsToUse=9, sensorMaxDistance=10, avoidDistance=0.5, tol=1)

myWorld.close()

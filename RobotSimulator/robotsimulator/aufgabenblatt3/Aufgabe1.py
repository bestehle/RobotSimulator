from numpy import math
from robotsimulator.graphics.graphics import Point
from robotsimulator.worlds import officeWorld
from robotsimulator import Robot
from robotsimulator.World import World
from robotsimulator.PathPlanning import PathPlanning

myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld = officeWorld.buildWorld()

myWorld.setRobot(myRobot, 2, 7, 0 * math.pi)
start = (2, 7)
goal = (15, 6)

pathPlanning = PathPlanning(myRobot, myWorld)
#pathPlanning.brushfire()
path = pathPlanning.shortestPath(start, goal)
polyline = []
for point in path:
    polyline.append(Point(point[0], point[1]))
myWorld.drawPolyline(polyline, color='green')

v = 0.2
myRobot.followPolylineWithObstacle(v, polyline, 6, 0.1, 0.1)

myWorld.close()

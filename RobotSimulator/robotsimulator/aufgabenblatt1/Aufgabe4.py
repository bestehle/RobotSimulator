from numpy import math
from robotsimulator import Robot, GeometryHelper
from robotsimulator.World import World
from robotsimulator.graphics.graphics import Point


myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld.addBox(5, 5)
myWorld.setRobot(myRobot, 6, 12, math.pi / 4)

v = 1
p = Point(5, 5)
tol = 0.2

myRobot.goto(v, p, tol)
myWorld.close()


def gotoTurnFirst(robot, v, p, tol):
    # get the actual position of robot
    [x, y, theta] = robot.getTrueRobotPose();
    # calculate the distance between robot and target point
    distance = math.sqrt(((x - p.getX()) ** 2) + ((y - p.getY()) ** 2))
    delta_theta = GeometryHelper.diffDegree(math.atan2(p.getY() - y, p.getX() - x), theta)
    # point not reached?
    if(distance > tol):
        # drive missing distance
        robot.curveDriveTruePose(0.5, 0, delta_theta)
        robot.straightDriveTruePose(v, distance);
        # call goto again.
        robot.goto(v, p, tol)
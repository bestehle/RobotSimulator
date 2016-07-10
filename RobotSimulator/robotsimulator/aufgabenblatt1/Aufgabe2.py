from numpy import math
from robotsimulator import Robot, GeometryHelper
from robotsimulator.World import World

myWorld = World(40, 20)
myRobot = Robot.Robot()
# myRobot.deactivateMotionNoise()
myWorld.setRobot(myRobot, 8, 5, 0)


def straightDriveTruePose(robot, v, l):
    if v == 0:
        return
    truePoseBefore = robot.getTrueRobotPose()
    trueL = 0
    while trueL < (l - v * robot._T):
        robot.move([v, 0])
        truePoseAfter = robot.getTrueRobotPose()
        trueL = math.sqrt((truePoseAfter[0] - truePoseBefore[0]) ** 2 + (truePoseAfter[1] - truePoseBefore[1]) ** 2)
    v = (l - trueL) / robot._T
    robot.move([v, 0])

def curveDriveTruePose(robot, v, r, delta_theta):
    if v == 0 and r != 0:
        return
    if r == 0:
        omega = robot._maxOmega
    else:
        omega = (v / r)
    sign = -1 if delta_theta < 0 else 1
    endTheta = GeometryHelper.addDegree(robot.getTrueRobotPose()[2], delta_theta)
    while abs(GeometryHelper.diffDegree(robot.getTrueRobotPose()[2], endTheta)) >= (omega * robot._T) :
        robot.move([v, omega * sign])
    robot.move([v, GeometryHelper.diffDegree(robot.getTrueRobotPose()[2], endTheta) / robot._T * sign])

v = 1
v2 = 1

myRobot.straightDrive(v, 1.5)
myRobot.curveDrive(v, 2, math.pi / 2)
myRobot.straightDrive(v, 3)
myRobot.curveDrive(v, 2, math.pi / 2)
myRobot.straightDrive(v, 3)
myRobot.curveDrive(v, 2, math.pi / 2)
myRobot.straightDrive(v, 3)
myRobot.curveDrive(v, 2, math.pi / 2)
myRobot.straightDrive(v, 1.5)

myWorld.setRobot(myRobot, 20, 5, 0)

straightDriveTruePose(myRobot, v, 1.5)
curveDriveTruePose(myRobot, v, 2, math.pi / 2)
straightDriveTruePose(myRobot, v, 3)
curveDriveTruePose(myRobot, v, 2, math.pi / 2)
straightDriveTruePose(myRobot, v, 3)
curveDriveTruePose(myRobot, v, 2, math.pi / 2)
straightDriveTruePose(myRobot, v, 3)
curveDriveTruePose(myRobot, v, 2, math.pi / 2)
straightDriveTruePose(myRobot, v, 1.5)

# Simulation schliessen:
myWorld.close()


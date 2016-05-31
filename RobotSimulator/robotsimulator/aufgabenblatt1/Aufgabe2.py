from numpy import math
from robotsimulator import Robot
from robotsimulator.World import World

myWorld = World(40, 20)
myRobot = Robot.Robot()
# myRobot.deactivateMotionNoise()
myWorld.setRobot(myRobot, 8, 5, 0)

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

myRobot.straightDriveTruePose(v, 1.5)
myRobot.curveDriveTruePose(v, 2, math.pi / 2)
myRobot.straightDriveTruePose(v, 3)
myRobot.curveDriveTruePose(v, 2, math.pi / 2)
myRobot.straightDriveTruePose(v, 3)
myRobot.curveDriveTruePose(v, 2, math.pi / 2)
myRobot.straightDriveTruePose(v, 3)
myRobot.curveDriveTruePose(v, 2, math.pi / 2)
myRobot.straightDriveTruePose(v, 1.5)

# Simulation schliessen:
myWorld.close()
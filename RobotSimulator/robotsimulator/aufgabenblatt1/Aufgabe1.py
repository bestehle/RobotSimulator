from numpy import math
from robotsimulator import Robot
from robotsimulator.World import World

def driveCircle(robot, v, r, direction):
    robot.curveDrive(v, r, math.pi)   
    robot.curveDrive(v, r, math.pi)   
    
def driveRectangle(robot, v, a, b):
    robot.straightDrive(v, a)
    robot.curveDrive(v, 0, math.pi / 2)   
    robot.straightDrive(v, b)
    robot.curveDrive(v, 0, math.pi / 2)   
    robot.straightDrive(v, a)
    robot.curveDrive(v, 0, math.pi / 2)   
    robot.straightDrive(v, b)
    robot.curveDrive(v, 0, math.pi / 2)   
    
def driveChangeLane(robot, v, r, alfa, length):
    robot.curveDrive(v, r, alfa)
    robot.straightDrive(v, length)
    robot.curveDrive(v, r, -alfa)

myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, 5, 12, 0)
myRobot.deactivateMotionNoise()

v = 1
v2 = 1
driveCircle(myRobot, v, 3, 1)

myRobot.straightDrive(v, 3)
driveChangeLane(myRobot, v, 1, math.pi / 4, 1)
myRobot.straightDrive(v, 3)
driveChangeLane(myRobot, v, 1, -math.pi / 4, 1)
myRobot.straightDrive(v, 1)

driveRectangle(myRobot, v, 2, 2)

myWorld.close()
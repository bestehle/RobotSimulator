# class Robot.
#
# This class define methods to move a robot and to sense the world.
# The robot knows only its pose estimated by odometry.
#
# O. Bittel
# V 2.0; 9.3.2016


from math import *
import random
from numpy import math


class Robot:
    # --------
    # init: creates robot.
    #
    def __init__(self):
        self._size = 0.4  # robot diameter
        self._T = 0.1  # time step
        self._world = None  # robot's world; is set by setWorld()

        # Motion parameter:
        self._k_d = 0.02 * 0.02  # velocity noise parameter = 0.02m*0.02m / 1m
        self._k_theta = (2.0 * 2.0 / 360.0) * (pi / 180.0)  # turning rate noise parameter = 2deg*2deg/360deg * (1rad/1deg)
        self._k_drift = (2.0 * 2.0) / 1.0 * (pi / 180.0) ** 2  # drift noise parameter = 2deg*2deg / 1m
        self._motionNoise = True
        self._maxSpeed = 2  # maximum speed
        self._maxOmega = pi  # maximum rotational speed

        # Sensor parameter (x-axis is in forward direction):
        self._numberOfSensors = 36
        dTheta = 360.0 / self._numberOfSensors
        self._sensorDirections = [(-90.0 + dTheta * i) * (pi / 180.0) for i in range(self._numberOfSensors)]
        self._maxSenseValue = 5.0  # Maximum sensor value for each sensor beam
        self._sensorNoise = 0.01  # standard deviation of distance measurement for 1m

        # Odometry Pose
        self._odoX = 0.0
        self._odoY = 0.0
        self._odoTheta = pi / 2
        
    def activateMotionNoise(self):
        self._motionNoise = True

    def deactivateMotionNoise(self):
        self._motionNoise = False

    def getTimeStep(self):
        return self._T

    def setTimeStep(self, T):
        self._T = T

    # --------
    # returns the diameter of the robot
    #
    def getSize(self):
        return self._size

    # --------
    # returns the direction of the sensors
    #
    def getSensorDirections(self):
        return self._sensorDirections

    # --------
    # returns the maximal possible sensor value
    #
    def getMaxSenseValue(self):
        return self._maxSenseValue


    # --------
    # Set the odometry pose
    #
    def setOdoPose(self, x, y, theta):
        self._odoX = x
        self._odoY = y
        self._odoTheta = theta

    # --------
    # get the odometry pose
    #
    def getOdoPose(self):
        return [self._odoX, self._odoY, self._odoTheta]

    # --------
    # get the true robot pose (x,y,theta).
    #
    def getTrueRobotPose(self):
        return self._world.getTrueRobotPose()

    # --------
    # move the robot for the next time step T by the
    # command motion = [v,omega].
    # Returns False if robot is stalled.
    #
    def move(self, motion):
        v = motion[0]
        omega = motion[1]

        # translational and rotational speed is limited:
        if omega > self._maxOmega:
            omega = self._maxOmega
        if omega < -self._maxOmega:
            omega = -self._maxOmega
        if v > self._maxSpeed:
            v = self._maxSpeed
        if v < -self._maxSpeed:
            v = -self._maxSpeed

        print ("motion ", v, omega * 180 / pi)

        # Odometry pose update (based on the motion command):
        d = v * self._T
        dTheta = omega * self._T
        self._odoX += d * cos(self._odoTheta + 0.5 * dTheta)
        self._odoY += d * sin(self._odoTheta + 0.5 * dTheta)
        self._odoTheta = (self._odoTheta + dTheta) % (2 * pi)

        # Add noise to v:
        if not self._motionNoise:
            return self._world.moveRobot(d, dTheta, self._T)    
        
        sigma_v_2 = (self._k_d / self._T) * abs(v)
        v_noisy = v + random.gauss(0.0, sqrt(sigma_v_2))

        # Add noise to omega:
        sigma_omega_tr_2 = (self._k_theta / self._T) * abs(omega)  # turning rate noise
        sigma_omega_drift_2 = (self._k_drift / self._T) * abs(v)  # drift noise
        omega_noisy = omega + random.gauss(0.0, sqrt(sigma_omega_tr_2))
        omega_noisy += random.gauss(0.0, sqrt(sigma_omega_drift_2))

        # Move robot in the world (with noise):
        d_noisy = v_noisy * self._T
        dTheta_noisy = omega_noisy * self._T
        return self._world.moveRobot(d_noisy, dTheta_noisy, self._T)

    # dirves length l with speed v 
    def straightDrive(self, v, l):
        if v == 0:
            return
        t = int((l / v) / self._T)
        for i in range(0, t):
            self.move([v, 0])
            
    def curveDrive(self, v, r, delta_theta):
        if v == 0 and r != 0:
            return
        if r == 0:
            omega = self._maxOmega
        else:
            omega = (v / r)
        sign = -1 if delta_theta < 0 else 1
        tau = int((((delta_theta * sign) % (math.pi * 2)) / omega) / self._T)
        for i in range(0, tau):
            self.move([v, omega * sign])

    def driveCircle(self, v, r, direction):
        self.curveDrive(v, r, math.pi)   
        self.curveDrive(v, r, math.pi)   
        
    def driveRectangle(self, v, a, b):
        self.straightDrive(v, a)
        self.curveDrive(v, 0, math.pi / 2)   
        self.straightDrive(v, b)
        self.curveDrive(v, 0, math.pi / 2)   
        self.straightDrive(v, a)
        self.curveDrive(v, 0, math.pi / 2)   
        self.straightDrive(v, b)
        self.curveDrive(v, 0, math.pi / 2)   
        
    def driveChangeLane(self, v, r, alfa, length):
        self.curveDrive(v, r, alfa)
        self.straightDrive(v, length)
        self.curveDrive(v, r, -alfa)
           

    # --------
    # sense and returns distance measurements for each sensor beam.
    # If a sensor beams senses no obstacle distance value is set to None.
    #
    def sense(self):
        sensorDistNoisy = []
        sensorDist = self._world.sense()
        for d in sensorDist:
            if d is not None:
                # print "d: ", d
                sigma2 = self._sensorNoise ** 2 * d
                d += random.gauss(0.0, sqrt(sigma2))
            sensorDistNoisy.append(d)
        return sensorDistNoisy

    # --------
    # Sense boxes.
    # Return [distances, angles] for all sensed boxes.
    # Return None, if no boxes are visible.
    #
    def senseBoxes(self):
        distAngles = self._world.senseBox()
        if distAngles is None or distAngles[0] == []:
            return None
        else:
            return distAngles

    # --------
    # set world
    #
    def setWorld(self, world):
        self._world = world





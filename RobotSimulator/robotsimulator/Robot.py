# class Robot.
#
# This class define methods to move a robot and to sense the world.
# The robot knows only its pose estimated by odometry.
#
# O. Bittel
# V 2.0; 9.3.2016

import random

from numpy import math

from robotsimulator import GeometryHelper
from robotsimulator.EventEmitter import EventEmitter


class Robot:
    # --------
    # init: creates robot.
    #
    def __init__(self):
        self._size = 0.4  # robot diameter
        self._T = 0.1  # time step
        self._world = None  # robot's world; is set by setWorld()
        self.FRONT_SENSOR_INDEX = 10

        # Motion parameter:
        self._k_d = 0.02 * 0.02  # velocity noise parameter = 0.02m*0.02m / 1m
        self._k_theta = (2.0 * 2.0 / 360.0) * (math.pi / 180.0)  # turning rate noise parameter = 2deg*2deg/360deg * (1rad/1deg)
        self._k_drift = (2.0 * 2.0) / 1.0 * (math.pi / 180.0) ** 2  # drift noise parameter = 2deg*2deg / 1m
        self._motionNoise = True
        self._maxSpeed = 2  # maximum speed
        self._maxOmega = math.pi  # maximum rotational speed

        # Sensor parameter (x-axis is in forward direction):
        self._numberOfSensors = 36
        dTheta = 360.0 / self._numberOfSensors
        self._sensorDirections = [(-90.0 + dTheta * i) * (math.pi / 180.0) for i in range(self._numberOfSensors)]
        self._maxSenseValue = 5.0  # Maximum sensor value for each sensor beam
        self._sensorNoise = 0.01  # standard deviation of distance measurement for 1m

        # Odometry Pose
        self._odoX = 0.0
        self._odoY = 0.0
        self._odoTheta = math.pi / 2
        
        # initialize event emitter which will be called after each move cycle.
        self._moveListener = EventEmitter()
        
        # method to be called if we use approximate position
        self._approximatePosition = None
        
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
    def getRobotPose(self):
        if (self._approximatePosition):
            return self._approximatePosition()
        else:
            return self._world.getTrueRobotPose()

    # --------
    # Add a method to be called to ask for the position of the robot.
    #
    def useApproximatePosition(self, method):
        self._approximatePosition = method

    # --------
    # add a move listener
    #
    def onMove(self, method):
        self._moveListener += method
        
    def clearOnMoveListener(self):
        self._moveListener.clear()

    # --------
    # move the robot for the next time step T by the
    # command motion = [v,omega].
    # Returns False if robot is stalled.
    #
    def move(self, motion):
        v = motion[0]
        omega = motion[1]
        self._currentV = v
        self._currentOmega = omega

        # translational and rotational speed is limited:
        if omega > self._maxOmega:
            omega = self._maxOmega
        if omega < -self._maxOmega:
            omega = -self._maxOmega
        if v > self._maxSpeed:
            v = self._maxSpeed
        if v < -self._maxSpeed:
            v = -self._maxSpeed

        # print ("motion ", v, omega * 180 / math.pi)

        # Odometry pose update (based on the motion command):
        d = v * self._T
        dTheta = omega * self._T
        self._odoX += d * math.cos(self._odoTheta + 0.5 * dTheta)
        self._odoY += d * math.sin(self._odoTheta + 0.5 * dTheta)
        self._odoTheta = (self._odoTheta + dTheta) % (2 * math.pi)
        
        # Add noise to v:
        if not self._motionNoise:
            return self._world.moveRobot(d, dTheta, self._T)    
        
        sigma_v_2 = (self._k_d / self._T) * abs(v)
        v_noisy = v + random.gauss(0.0, math.sqrt(sigma_v_2))

        # Add noise to omega:
        sigma_omega_tr_2 = (self._k_theta / self._T) * abs(omega)  # turning rate noise
        sigma_omega_drift_2 = (self._k_drift / self._T) * abs(v)  # drift noise
        omega_noisy = omega + random.gauss(0.0, math.sqrt(sigma_omega_tr_2))
        omega_noisy += random.gauss(0.0, math.sqrt(sigma_omega_drift_2))

        # Move robot in the world (with noise):
        d_noisy = v_noisy * self._T
        dTheta_noisy = omega_noisy * self._T
        
        # call the move listeners
        self._moveListener.emit()
        
        return self._world.moveRobot(d_noisy, dTheta_noisy, self._T)
    
    def moveRandom(self, v, x, scaledAngle):
        if x % 30 == 0:
            for _ in range(1, 4):
                self.move([v, (random.random() - 0.5) * 2 * math.pi])
        else:
            self.move([v, scaledAngle])

    # dirves length l with speed v 
    def straightDrive(self, v, l):
        if v == 0:
            return
        t = int((l / v) / self._T)
        for _ in range(0, t):
            self.move([v, 0])
            
    def curveDrive(self, v, r, delta_theta, tolerance=0.01):
        if v == 0 and r != 0:
            return
        if r == 0:
            theta = self.getOdoPose()[2] + delta_theta
            diff = GeometryHelper.diffDegree(theta, self.getOdoPose()[2])
            while abs(diff) > tolerance :
                self.move([0, diff])
                diff = GeometryHelper.diffDegree(theta, self.getOdoPose()[2])
            return
        else:
            omega = (v / r)
        sign = -1 if delta_theta < 0 else 1
        tau = round((((delta_theta * sign) % (math.pi * 2)) / omega) / self._T)
        for _ in range(0, tau):
            self.move([v, omega * sign])
            
    def rotate(self, delta, tolerance=0.1):
        self.curveDrive(0, 0, delta, tolerance)
    
    def distanceFromRobotToLine(self, p1, p2):
        return GeometryHelper.perpendicularDistance((p1.x, p1.y), (p2.x, p2.y), self.getRobotPose())

    def followLineP(self, v, kp, p1, p2, kd=1):
        e = self.distanceFromRobotToLine(p1, p2)
        while e != 0:
            e = self.distanceFromRobotToLine(p1, p2)
            if not self.move([v, -kp * e]):
                return

    def followLinePD(self, v, kp, p1, p2, kd=1):
        e = self.distanceFromRobotToLine(p1, p2)
        while e != 0:
            de = (self.distanceFromRobotToLine(p1, p2) - e) / self._T
            e = self.distanceFromRobotToLine(p1, p2)
            if not self.move([v, (-kp * e) - (kd * de)]) :
                return

    def goto(self, v, p, tol):
        while True:
            [x, y, theta] = self.getRobotPose();
            distance = math.sqrt(((x - p.getX()) ** 2) + ((y - p.getY()) ** 2))
            delta_theta = GeometryHelper.diffDegree(math.atan2(p.getY() - y, p.getX() - x), theta)
            if (distance < tol):
                return True
            self.move([v, delta_theta])
            
            
    def gotoWithObstacleAvoidance(self, v, p, tol, sensorsToUse=9, distanceTol=1, minSpeed=0.2):
        while True:
            [x, y, theta] = self.getRobotPose();
            distance = math.sqrt(((x - p.getX()) ** 2) + ((y - p.getY()) ** 2))
            delta_theta = GeometryHelper.diffDegree(math.atan2(p.getY() - y, p.getX() - x), theta)
            if (distance < tol):
                return True
            if self.isObstacleInWay(sensorsToUse, distanceTol):
                return False
            if not self.move([ v * max(minSpeed, (1 - abs(delta_theta * 5) / math.pi)), delta_theta]):
                for _ in range(1, 5):
                    self.move([-1, 0])
               
    def followPolylineWithObstacleAvoidance(self, v, poly, sensorsToUse=3,
                                             sensorMaxDistance=5, avoidDistance=1, tol=1):
        for p in poly:
            while True:
                if not self.gotoWithObstacleAvoidance(v, p, tol, sensorsToUse, avoidDistance):
                    self.avoidObstacle(v, sensorsToUse, sensorMaxDistance, avoidDistance)
                else:
                    break
            
    def isObstacleInWay(self, sensorsToUse, distance):
        left, right, front = self.getWeightedSensorData(sensorsToUse, distance)
        return left != 0 or right != 0 or front != 0
    
    def avoidObstacle(self, v, sensorsToUse=3, sensorMaxDistance=5, avoidDistance=1, minSpeed=0.05):
        x = 0
        scale = math.pi / (sensorsToUse * (sensorsToUse + 1) / 2 * 5 + 5)
        self._world.ROBOT_WAY_COLOR = 'yellow'
        while self.isObstacleInWay(sensorsToUse, avoidDistance):
            x = x + 1
            left, right, front = self.getWeightedSensorData(sensorsToUse, sensorMaxDistance)  
            
            if self.isInDeadEnd(avoidDistance, scale, left, right, front):
                self.move([minSpeed, -self._maxOmega])
                continue
            
            if right > left:
                angle = right + front
            else:
                angle = -(left + front)
            scaledAngle = angle * scale
            
            # move the robot and check if the robot stalled.
            # if the robot stalled, rotate with 90 degree to left or right
            if not self.move([max(minSpeed, v * (1 - abs(scaledAngle) / math.pi)), scaledAngle]):
                if right > left:
                    self.rotate(-(math.pi / 2))
                else:
                    self.rotate((math.pi / 2))
        self._world.ROBOT_WAY_COLOR = 'red'

    def braitenberg(self, v, sensorsToUse=3, distance=5, minSpeed=0.05):
        x = 0
        scale = math.pi / (sensorsToUse * (sensorsToUse + 1) / 2 * 5 + 5)
        while True:
            x = x + 1
            left, right, front = self.getWeightedSensorData(sensorsToUse, distance)  
            
            if self.isInDeadEnd(distance, scale, left, right, front):
                self.move([minSpeed, -self._maxOmega])
                continue
            
            if right > left:
                angle = right + front
            else:
                angle = -(left + front)
            scaledAngle = angle * scale
            
            if angle != 0:
                self.move([max(minSpeed, v * (1 - abs(scaledAngle) / math.pi)), scaledAngle])
            else:
                self.moveRandom(v, x, scaledAngle)

    def getWeightedSensorData(self, sensorsToUse, distance):
        sensors = self.sense()
        right = 0
        left = 0
        front = 0
        for i in range(1, sensorsToUse + 1):
            sensorRight = self.FRONT_SENSOR_INDEX - 1 - sensorsToUse + i
            if sensors[sensorRight] != None and sensors[sensorRight] <= distance:
                right += i * (5 - sensors[sensorRight])
            sensorLeft = self.FRONT_SENSOR_INDEX + 1 + sensorsToUse - i
            if sensors[sensorLeft] != None and sensors[sensorLeft] <= distance:
                left += i * (5 - sensors[sensorLeft])
        
        if sensors[self.FRONT_SENSOR_INDEX] != None and sensors[self.FRONT_SENSOR_INDEX] <= distance:
            front = 5 - sensors[self.FRONT_SENSOR_INDEX]
        return left, right, front


    def isInDeadEnd(self, distance, scale, left, right, front):
        return (left + right + front) * scale / 2 > 2 and front > distance

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
                d += random.gauss(0.0, math.sqrt(sigma2))
            sensorDistNoisy.append(d)
        return sensorDistNoisy

    # --------
    # Sense Landmarks
    # Returns distance measurements of the landmarks from the current position
    # of the robot
    #
    def senseLandmarks(self, landmarks):
        angles = []
        distances = []
        (posX, posY, _) = self._world.getTrueRobotPose()
        for (landX, landY) in landmarks:
            distance = math.sqrt((landX - posX) ** 2 + (landY - posY) ** 2)
            distance += random.gauss(0, self._sensorNoise)
            angle = math.atan2(posY - landY, posX - landX)
            #angle += random.gauss(0, self._sensorNoise)
            angles.append(angle)
            distances.append(distance)
        return (distances, angles)

    # --------
    # set world
    #
    def setWorld(self, world):
        self._world = world


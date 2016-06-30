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
        self.FRONT = 10

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
    def getTrueRobotPose(self):
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

    # dirves length l with speed v 
    def straightDrive(self, v, l):
        if v == 0:
            return
        t = int((l / v) / self._T)
        for i in range(0, t):
            self.move([v, 0])

    # drives length l with speed v 
    def straightDriveTruePose(self, v, l):
        if v == 0:
            return
        truePoseBefore = self.getTrueRobotPose()
        trueL = 0
        while trueL < (l - v * self._T):
            self.move([v, 0])
            truePoseAfter = self.getTrueRobotPose()
            trueL = math.sqrt((truePoseAfter[0] - truePoseBefore[0]) ** 2 + (truePoseAfter[1] - truePoseBefore[1]) ** 2)
        v = (l - trueL) / self._T
        self.move([v, 0])
            
    def curveDrive(self, v, r, delta_theta):
        if v == 0 and r != 0:
            return
        if r == 0:
            theta = self.getOdoPose()[2] + delta_theta
            diff = GeometryHelper.diffDegree(theta, self.getOdoPose()[2])
            while abs(diff) > 0.01 :
                self.move([0, diff])
                diff = GeometryHelper.diffDegree(theta, self.getOdoPose()[2])
            return
        else:
            omega = (v / r)
        sign = -1 if delta_theta < 0 else 1
        tau = round((((delta_theta * sign) % (math.pi * 2)) / omega) / self._T)
        for i in range(0, tau):
            self.move([v, omega * sign])
            
            
    def curveDriveTruePose(self, v, r, delta_theta):
        if v == 0 and r != 0:
            return
        if r == 0:
            omega = self._maxOmega
        else:
            omega = (v / r)
        sign = -1 if delta_theta < 0 else 1
        endTheta = GeometryHelper.addDegree(self.getTrueRobotPose()[2], delta_theta)
        while abs(GeometryHelper.diffDegree(self.getTrueRobotPose()[2], endTheta)) >= (omega * self._T) :
            self.move([v, omega * sign])
        self.move([v, self.diffDegree(self.getTrueRobotPose()[2], endTheta) / self._T * sign])

    
    def distance(self, p1, p2):
        return GeometryHelper.perpendicularDistance((p1.x, p1.y), (p2.x, p2.y), self.getTrueRobotPose())

    def followLineP(self, v, kp, p1, p2, kd=1):
        e = self.distance(p1, p2)
        while e != 0:
            e = self.distance(p1, p2)
            if not self.move([v, -kp * e]):
                return

    def followLinePD(self, v, kp, p1, p2, kd=1):
        e = self.distance(p1, p2)
        while e != 0:
            de = (self.distance(p1, p2) - e) / self._T
            e = self.distance(p1, p2)
            if not self.move([v, (-kp * e) - (kd * de)]) :
                return

    def gotoTurnFirst(self, v, p, tol):
        # get the actual position of robot
        [x, y, theta] = self.getTrueRobotPose();
        # calculate the distance between robot and target point
        distance = math.sqrt(((x - p.getX()) ** 2) + ((y - p.getY()) ** 2))
        delta_theta = GeometryHelper.diffDegree(math.atan2(p.getY() - y, p.getX() - x), theta)
        # point not reached?
        if(distance > tol):
            # drive missing distance
            self.curveDriveTruePose(0.5, 0, delta_theta)
            self.straightDriveTruePose(v, distance);
            # call goto again.
            self.goto(v, p, tol)

    def goto(self, v, p, tol):
        while True:
            [x, y, theta] = self.getTrueRobotPose();
            distance = math.sqrt(((x - p.getX()) ** 2) + ((y - p.getY()) ** 2))
            delta_theta = GeometryHelper.diffDegree(math.atan2(p.getY() - y, p.getX() - x), theta)
            if (distance < tol):
                return True
            self.move([v, delta_theta])
            
            
    def gotoWithObstacle(self, v, p, tol, sensorsToUse, distanceTol, minSpeed=0.2):
        while True:
            [x, y, theta] = self.getTrueRobotPose();
            distance = math.sqrt(((x - p.getX()) ** 2) + ((y - p.getY()) ** 2))
            delta_theta = GeometryHelper.diffDegree(math.atan2(p.getY() - y, p.getX() - x), theta)
            if (distance < tol):
                return True
            if self.obstacleInWay(sensorsToUse, distanceTol):
                return False
            self.move([ v * max(minSpeed, (1 - abs(delta_theta * 2) / math.pi)), delta_theta])
        
    def followPolylineTurnFirst(self, v, poly, tol=1):
        for p in poly:
            self.gotoTurnFirst(v, p, tol);
        
    def followPolyline(self, v, poly, tol=1):
        for p in poly:
            self.goto(v, p, tol);     
               
    def followPolylineWithObstacle(self, v, poly, sensorsToUse=3, distance=5, tol=1):
        i = 0
        for p in poly:
            i += 1
            # print("Polyline : ", i)
            while True:
                if not self.gotoWithObstacle(v, p, tol, sensorsToUse, distance):
                    self.avoidObstacle(v, sensorsToUse, distance)
                else:
                    break
            
            
    def obstacleInWay(self, sensorsToUse, distance):
        left, right, front = self.getWeightedSensorData(sensorsToUse, distance)
        return left != 0 or right != 0 or front != 0
    
    def avoidObstacle(self, v, sensorsToUse=3, distance=5, minSpeed=0.05):
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
            
            # print(left, right, front, scaledAngle)
            
            if angle != 0:
                self.move([max(minSpeed, v * (1 - abs(scaledAngle) / math.pi)), scaledAngle])
            else:
                return

    def braitenberg(self, v, sensorsToUse=3, distance=5):
        x = 0
        scale = math.pi / (sensorsToUse * (sensorsToUse + 1) / 2 * 5 + 5)
        while True:
            x = x + 1
            left, right, front = self.getWeightedSensorData(sensorsToUse, distance)  
            
            if self.isInDeadEnd(distance, scale, left, right, front):
                self.move([0, -self._maxOmega])
                continue
            
            if right > left:
                angle = right + front
            else:
                angle = -(left + front)
            scaledAngle = angle * scale
            
            # print(left, right, front, scaledAngle)
            
            if angle != 0:
                self.move([v, scaledAngle])
            else:
                self.moveRandom(v, x, scaledAngle)

    def getWeightedSensorData(self, sensorsToUse, distance):
        sensors = self.sense()
        right = 0
        left = 0
        front = 0
        for i in range(1, sensorsToUse + 1):
            sensorRight = self.FRONT - 1 - sensorsToUse + i
            if sensors[sensorRight] != None and sensors[sensorRight] <= distance:
                right += i * (5 - sensors[sensorRight])
            sensorLeft = self.FRONT + 1 + sensorsToUse - i
            if sensors[sensorLeft] != None and sensors[sensorLeft] <= distance:
                left += i * (5 - sensors[sensorLeft])
        
        if sensors[self.FRONT] != None and sensors[self.FRONT] <= distance:
            front = 5 - sensors[self.FRONT]
        return left, right, front


    def isInDeadEnd(self, distance, scale, left, right, front):
        return (left + right + front) * scale / 2 > 2 and front > distance


    def moveRandom(self, v, x, scaledAngle):
        if x % 30 == 0:
            for _ in range(1, 4):
                self.move([v, (random.random() - 0.5) * 2 * math.pi])
        else:
            self.move([v, scaledAngle])


    # --------
    # rotate the robot with the given delta
    #     
    def rotate(self, delta):
        (_, _, theta) = self.getTrueRobotPose();
        self._world.moveRobot(0, delta, self._T);
        self._odoTheta = theta + delta;

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
    # Use senseBoxes() to check for near boxes.
    #
    def findBoxes(self):
        print ("find boxes")
        boxes = self.senseBoxes()
        print (boxes)

    # --------
    # Sense Landmarks
    # Returns distance measurements of the landmarks from the current position
    # of the robot
    #
    def senseLandmarks(self, landmarks):
        ret = []
        (posX, posY, _) = self.getTrueRobotPose()
        for (landX, landY) in landmarks:
            distance = math.sqrt((landX - posX) ** 2 + (landY - posY) ** 2)
            distance += random.gauss(0, self._sensorNoise)
            ret.append(distance)
        return ret

    # --------
    # set world
    #
    def setWorld(self, world):
        self._world = world


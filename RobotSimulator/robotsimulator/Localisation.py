from statistics import median
from numpy import math, random

from robotsimulator import GeometryHelper
from robotsimulator import Stats
from robotsimulator.graphics import graphics
from robotsimulator.graphics.graphics import Line, Point
from robotsimulator.EventEmitter import EventEmitter


class Localisation:
    def __init__(self, robot, world, numberOfParticles=300):
        # constants to access items in list
        self.X = 0
        self.Y = 1
        self.THETA = 2
        self.WEIGHT = 3
        self.SUM = 4
        
        self.xFault = 0
        self.yFault = 0
        self.thetaFault = 0
        self.FaultCount = 0
        
        # the variance for generated particles.
        self.PARTICLE_VARIANZ = 1
        self.drawWayOfParticle = False
        self.drawParticles = False
        self.drawWayOfLocalisation = False
        self.printFault = False
        self.printPosition = False
    
        self._robot = robot
        self._world = world
        self._position = self._robot.getTrueRobotPose()
        self._grid = self._world.getOccupancyGrid()
        self._numberOfParticles = numberOfParticles
        self._particles = self._generateParticles()
        self._landmarks = []
        self._positionFoundListener = EventEmitter()
    
    # --------
    # add a method to call after the position was found.
    #
    def onPositionFound(self, method):
        self._positionFoundListener += method
       
    # --------
    # clear all listeners for the position found event.
    # 
    def clearOnPositionFoundListener(self):
        self._positionFoundListener.clear()
        
    # --------
    # add a landmark which will be used for the measurement model
    #        
    def addLandmark(self, x, y):
        self._landmarks.append((x, y))

    # --------
    # return the coordinates (x, y, theta) of the approximate position
    #
    def getPosition(self):
        return self._position

    # --------
    # check: call this method cyclic to run the localisation algorithm
    # while driving
    #
    def check(self):
        self._sampleMotionModel()
        self._measurementModel()
        if self.drawParticles:
            self._world.drawParticles(self._particles)
        self._particles = self._resampling()
        # calculate the approximate position
        x = median(map(lambda l : l[self.X], self._particles))
        y = median(map(lambda l : l[self.Y], self._particles))
        theta = median(map(lambda l : l[self.THETA], self._particles))
        if self.drawWayOfLocalisation:
                pathLine = Line(Point(self._position[self.X], self._position[self.Y]), Point(x, y))
                color = min(255, int(abs(self._position[self.THETA] - theta) * 1000))
                pathLine.setFill(graphics.color_rgb(color, 255 - color, 0))
                pathLine.setWidth(2)
                pathLine.draw(self._world._win)
        self._position = (x, y, theta)
        self._world.drawApproximatePosition(self._position)
        if self.printFault:
            self.printLocalistationFault()
        if self.printPosition:
            self.printRobotPosition()

    def _generateParticles(self):
        [xPos, yPos, theta] = self._position
        xValues = map(lambda x: (x - 0.5) * self.PARTICLE_VARIANZ + xPos, random.random(self._numberOfParticles))
        yValues = map(lambda x: (x - 0.5) * self.PARTICLE_VARIANZ + yPos, random.random(self._numberOfParticles))
        thetaValues = random.normal(theta, 0.2, self._numberOfParticles)
        weightValues = [0] * self._numberOfParticles
        sumValues = [0] * self._numberOfParticles
        
        return [list(a) for a in zip(xValues, yValues, thetaValues, weightValues, sumValues)]
   
    # --------
    # Pull particles depending on the weight of the particles with the roulette technique
    # and return a new list with the pulled particles.
    #
    def _resampling(self):
        pulledParticles = []
        maximum = sum(p[self.WEIGHT] for p in self._particles)

        for _ in range(self._numberOfParticles):
            item = random.uniform(0, maximum)
            # print ("random : ", item)
            li = 0
            re = self._numberOfParticles - 1
            # binary search
            while li <= re:
                m = int((li + re) / 2)

                if item < self._particles[m][self.SUM]:
                    re = m - 1
                elif item > self._particles[m][self.SUM]:
                    li = m + 1
                else:
                    break

            x = self._particles[m][self.X]
            y = self._particles[m][self.Y]
            theta = self._particles[m][self.THETA]
            pulledParticles.append([x, y, theta, 0, 0])

        return pulledParticles
    
    # --------
    # Add noisy sample motion of robot to all particles
    #
    def _sampleMotionModel(self, noiseFaktor=30, noiseReposition=0.05):  # TODO
        for i in range(self._numberOfParticles):
            v = max(self._robot._currentV, 0.01)
            omega = self._robot._currentOmega
            
            sigma_v_2 = (self._robot._k_d * noiseFaktor / self._robot._T) * abs(v)
            v_noisy = v + random.normal(0.05, math.sqrt(sigma_v_2))
    
            # Add noise to omega:
            sigma_omega_tr_2 = (self._robot._k_theta / self._robot._T) * abs(omega + 0.0001)  # turning rate noise
            sigma_omega_drift_2 = (self._robot._k_drift / self._robot._T) * abs(v)  # drift noise
            omega_noisy = omega + random.normal(0.0, math.sqrt(sigma_omega_tr_2))
            omega_noisy += random.normal(0.0, math.sqrt(sigma_omega_drift_2))
            
            omega = omega_noisy
            v = v_noisy
    
            # translational and rotational speed is limited:
            if omega > self._robot._maxOmega:
                omega = self._robot._maxOmega
            if omega < -self._robot._maxOmega:
                omega = -self._robot._maxOmega
            if v > self._robot._maxSpeed:
                v = self._robot._maxSpeed
            if v < -self._robot._maxSpeed:
                v = -self._robot._maxSpeed
    
            d = v * self._robot._T
            dTheta = omega * self._robot._T
            
            x = self._particles[i][self.X]
            y = self._particles[i][self.Y]
            theta = self._particles[i][self.THETA]
    
            x = (x + d * math.cos(theta + 0.5 * dTheta))
            y = (y + d * math.sin(theta + 0.5 * dTheta))
            theta = (theta + dTheta) % (2 * math.pi)
            
            if self.drawWayOfParticle == True:
                pathLine = Line(Point(self._particles[i][self.X], self._particles[i][self.Y]), Point(x, y))
                pathLine.setFill('green')
                pathLine.setWidth(1)
                pathLine.draw(self._world._win)
        
            self._particles[i][self.X] = x + random.normal(0.0, noiseReposition)
            self._particles[i][self.Y] = y + random.normal(0.0, noiseReposition)
            self._particles[i][self.THETA] = theta  # + random.normal(0.0, noiseReposition * 2)
            self._particles[i][self.WEIGHT] = 0
            self._particles[i][self.SUM] = 0

    def printRobotPosition(self):
        truePos = self._world.getTrueRobotPose()
        approX = round(self._position[self.X], 3)
        approY = round(self._position[self.Y], 3)
        approTheta = round(self._position[self.THETA], 3)
        truePosX = round(truePos[self.X], 3)
        truePosY = round(truePos[self.Y], 3)
        truePosTheta = round(truePos[self.THETA], 3)
        
        
        Stats.robotPositions(approX, '\t', approY, '\t', approTheta, '\t',
                             truePosX, '\t', truePosY, '\t', truePosTheta)

    def printLocalistationFault(self):
        currentXFault = abs(round(self._position[self.X] - self._world.getTrueRobotPose()[self.X], 3))
        currentYFault = abs(round(self._position[self.Y] - self._world.getTrueRobotPose()[self.Y], 3))
        currentThetaFault = abs(round(GeometryHelper.diffDegree(self._position[self.THETA], self._world.getTrueRobotPose()[self.THETA]), 3))
        self.xFault = abs(round((self.xFault * self.FaultCount + currentXFault) / (self.FaultCount + 1), 3))
        self.yFault = abs(round((self.yFault * self.FaultCount + currentYFault) / (self.FaultCount + 1), 3))
        self.thetaFault = abs(round((self.thetaFault * self.FaultCount + currentThetaFault) / (self.FaultCount + 1), 3))
        self.FaultCount += 1
        Stats.localisationFault(currentXFault, "\t", currentYFault, "\t", currentThetaFault, "\t",
             self.xFault, "\t", self.yFault, "\t", self.thetaFault, "\t", self.FaultCount)

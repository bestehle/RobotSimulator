
from statistics import median

from numpy import math, random

from robotsimulator import GeometryHelper
from robotsimulator import Stats
from robotsimulator.graphics import graphics
from robotsimulator.graphics.graphics import Line, Point


class GlobalLocalisation:
    # --------
    # init: Sets the robot and the world
    #
    def __init__(self, robot, world, numberOfParticles=500):
        # const
        self.X = 0
        self.Y = 1
        self.THETA = 2
        self.WEIGHT = 3
        self.SUM = 4
        
        self.xFault = 0
        self.yFault = 0
        self.thetaFault = 0
        self.FaultCount = 0
        
        self.PARTICLE_VARIANZ = 1
        self._drawWayOfParticle = False
        self._drawParticles = True
        self._drawWayOfLocalisation = False
        self._printFault = False
        self._printPosition = False
    
        self._robot = robot
        self._world = world
        self._position = (0, 0, 0)
        self._grid = self._world.getOccupancyGrid()
        self._numberOfParticles = numberOfParticles
        self._particles = self._generateParticles()

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
        self._measurementModelGlobal()
        if self._drawParticles:
            self._world.drawParticles(self._particles)
        self._particles = self._resampling()
        # calculate the approximate position
        x = median(map(lambda l : l[self.X], self._particles))
        y = median(map(lambda l : l[self.Y], self._particles))
        theta = median(map(lambda l : l[self.THETA], self._particles))
        if self._drawWayOfLocalisation:
                pathLine = Line(Point(self._position[self.X], self._position[self.Y]), Point(x, y))
                color = min(255, int(abs(self._position[self.THETA] - theta) * 1000))
                pathLine.setFill(graphics.color_rgb(color, 255 - color, 0))
                pathLine.setWidth(2)
                pathLine.draw(self._world._win)
        self._position = (x, y, theta)
        self._world.drawApproximatePosition(self._position)
        if self._printFault:
            self.printLocalistationFault()
        if self._printPosition:
            self.printPosition()
        
    def _generateParticles(self):
        xValues = list(map(lambda x: x * self._world._width, random.random(self._numberOfParticles)))
        yValues = list(map(lambda y: y * self._world._height, random.random(self._numberOfParticles)))
        thetaValues = list(map(lambda x: x * math.pi * 2, random.random(self._numberOfParticles)))

        self._numberOfParticles = len(thetaValues)
        weightValues = [0] * self._numberOfParticles
        sumValues = [0] * self._numberOfParticles
        
        return [list(a) for a in zip(xValues, yValues, thetaValues, weightValues, sumValues)]
    
    # --------
    # Pull particles depending on the weight of the particles with the roulette technique
    # and return the 
    #
    def _resampling(self):
        pulledParticles = []
        maximum = sum(p[self.WEIGHT] for p in self._particles)
        pulled = {}
        # print ("maximum is : " , maximum)
        # pull particles
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
            if (m in pulled):
                pulled[m] += 1
            else:
                pulled[m] = 1
        
#         print ("pulled particles by resampling (" , len(pulled.keys()) , ") : " , pulled)

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
            sigma_omega_tr_2 = (self._robot._k_theta / self._robot._T) * abs(omega)  # turning rate noise
            sigma_omega_drift_2 = (self._robot._k_drift / self._robot._T) * abs(v)  # drift noise
            omega_noisy = omega + random.normal(0.0, math.sqrt(sigma_omega_tr_2) + 0.1)
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
            
            if self._drawWayOfParticle == True:
                pathLine = Line(Point(self._particles[i][self.X], self._particles[i][self.Y]), Point(x, y))
                pathLine.setFill('green')
                pathLine.setWidth(1)
                pathLine.draw(self._world._win)
        
            self._particles[i][self.X] = x + random.normal(0.0, noiseReposition)
            self._particles[i][self.Y] = y + random.normal(0.0, noiseReposition)
            self._particles[i][self.THETA] = theta  # + random.normal(0.0, noiseReposition * 2)
            self._particles[i][self.WEIGHT] = 0
            self._particles[i][self.SUM] = 0

    # --------
    # Check how good the particles are and set the weight of each particle with the distance to landmarks
    #        
    def _measurementModelGlobal(self):
        # get the distance for each landMark to the current approximate robot position
        robotSense = self._world.globalSense(self._world.getTrueRobotPose())

        weightSum = 0
        # check each particle
        for i in range(self._numberOfParticles):
            self._particles[i][self.SUM] = weightSum
            self._particles[i][self.WEIGHT] = 1
            # get the sense for the current particle
            p = (self._particles[i][self.X], self._particles[i][self.Y], self._particles[i][self.THETA])
            pSense = self._world.globalSense(p)
            # check each landmark
            for j in range(len(robotSense)):
                if (robotSense[j] == pSense[j]):
                    self._particles[i][self.WEIGHT] *= 2
            
            # update the weight sum
            weightSum += self._particles[i][self.WEIGHT]

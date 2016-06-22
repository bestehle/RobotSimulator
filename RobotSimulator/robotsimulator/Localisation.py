
from numpy import math, random
from robotsimulator.graphics.graphics import Line, Point

class Localisation:
    # --------
    # init: Sets the robot and the world
    #
    def __init__(self, robot, world, numberOfParticles=200):
        # const
        self.X = 0
        self.Y = 1
        self.THETA = 2
        self.WEIGHT = 3
        self.SUM = 4
        
        self.PARTICLE_VARIANZ = 3
        self._drawWayOfParticle = True
        
        self._robot = robot
        self._world = world
        self._grid = self._world.getOccupancyGrid()
        self._numberOfParticles = numberOfParticles
        self._particles = self._generateParticles()
        self._landmarks = []

    # --------
    # return the coordinates (x, y) of the approximate position
    #
    def getPosition(self):
        # TODO calculate position
        (x, y, _) = self._world.getTrueRobotPose()
        return (x, y)
        

    def addLandmark(self, x, y):
        self._landmarks.append((x, y))

    # --------
    # check: call this method cyclic to run the localisation algorithm
    # while driving
    #
    def check(self):
        self._sampleMotionModel()
#         self._measurementModelSensors()
        self._measurementModelLandMarks()
        self._world.drawParticles(self._particles)
        self._particles = self._resampling()
        
        
    def _generateParticles(self):
        [xPos, yPos, theta] = self._robot.getTrueRobotPose()
        xValues = map(lambda x: (x - 0.5) * self.PARTICLE_VARIANZ + xPos, random.random(self._numberOfParticles))
        yValues = map(lambda x: (x - 0.5) * self.PARTICLE_VARIANZ + yPos, random.random(self._numberOfParticles))
#         thetaValues = map(lambda x: x * math.pi * 2, random.random(self._numberOfParticles))
        thetaValues = random.laplace(theta, 1, self._numberOfParticles)
        weightValues = [0] * self._numberOfParticles
        sumValues = [0] * self._numberOfParticles
        
        return [list(a) for a in zip(xValues, yValues, thetaValues, weightValues, sumValues)]
    
        
    def matchParticle(self, particle):
        weight = 0
        sensorData = zip(self._robot.getSensorDirections(), self._robot.sense())
        for direction, distance in sensorData:
            if distance is not None:
                sensorPosition = self._calculatePosition(direction, distance, particle)
                weight += self._grid.getValueWeight(sensorPosition[0], sensorPosition[1])
            else:
                sensorPosition = self._calculatePosition(direction, 5, particle)
                currentWeight = self._grid.getValueWeight(sensorPosition[0], sensorPosition[1])
                if currentWeight == 1:
                    weight += 1 
                else: 
                    weight += 0
        return weight  #  1000 / (weight + 0.001) TODO
                
        
        
    def _calculatePosition(self, direction, distance, position):
        absDirection = position[2] + direction
        x = math.cos(absDirection) * distance
        y = math.sin(absDirection) * distance
        return position[0] + x, position[1] + y
    
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
    def _sampleMotionModel(self, noiseFaktor=300): # TODO
        for i in range(self._numberOfParticles):
            v = self._robot._currentV
            omega = self._robot._currentOmega
            
            sigma_v_2 = (self._robot._k_d * noiseFaktor / self._robot._T) * abs(v)
            v_noisy = v + random.laplace(0.0, math.sqrt(sigma_v_2))
    
            # Add noise to omega:
            sigma_omega_tr_2 = (self._robot._k_theta * noiseFaktor / self._robot._T) * abs(omega)  # turning rate noise
            sigma_omega_drift_2 = (self._robot._k_drift * noiseFaktor / self._robot._T) * abs(v)  # drift noise
            omega_noisy = omega + random.laplace(0.0, math.sqrt(sigma_omega_tr_2))
            omega_noisy += random.laplace(0.0, math.sqrt(sigma_omega_drift_2))
            
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
        
            self._particles[i][self.X] = x
            self._particles[i][self.Y] = y
            self._particles[i][self.THETA] = theta
            self._particles[i][self.WEIGHT] = 0
            self._particles[i][self.SUM] = 0
        
    # --------
    # Check how good the particles are and set the weight of each particle with the Likelihood-Field
    #
    def _measurementModelSensors(self):
        weightSum = 0
        for i in range(len(self._particles)):
            self._particles[i][self.SUM] = weightSum
            weight = self.matchParticle(self._particles[i])
            # print ("weight", weight)
            weightSum += weight
            self._particles[i][self.WEIGHT] = weight
    
    # --------
    # Check how good the particles are and set the weight of each particle with the distance to landmarks
    #        
    def _measurementModelLandMarks(self):
        # get the distance for each landMark to the current approximate robot position
        robotDistance = self._robot.senseLandmarks(self._landmarks)

        weightSum = 0
        # check each particle
        for i in range(self._numberOfParticles):
            self._particles[i][self.SUM] = weightSum
            self._particles[i][self.WEIGHT] = 1
            # check each landmark
            for mark in range(len(self._landmarks)):
                # coordinates of the landmark
                (landmarkX, landmarkY) = self._landmarks[mark]
                # distance of the particle to the landmark
                particleDistance = math.sqrt((self._particles[i][self.X] - landmarkX) ** 2 + (self._particles[i][self.Y] - landmarkY) ** 2)
                # set (distance of robot to landmark) in relation to (distance of particle to landmark)
                distance = abs(robotDistance[mark] - particleDistance)
                # multiply the distance to the weight
                self._particles[i][self.WEIGHT] *= distance
                
            # invert weight to get high value for a good particle
            self._particles[i][self.WEIGHT] = 1 / self._particles[i][self.WEIGHT]
            
            # update the weight sum
            weightSum += self._particles[i][self.WEIGHT]
      #  print ("-----------------------------------------------")
      #  for particle in (self._particles):
      #      print ("sum : " , particle[self.SUM], "   weight : ", particle[self.WEIGHT], " coord : ", particle[self.X], "," , particle[self.Y])
      #  print ("-----------------------------------------------")
        #assert (1 == 2)
from numpy import math
from numpy import random


class Localisation:
    # --------
    # init: Sets the robot and the world
    #
    def __init__(self, robot, world, numberOfParticles=100):
        # const
        self.X = 0
        self.Y = 1
        self.THETA = 2
        self.WEIGHT = 3
        self.SUM = 4
        
        self._robot = robot
        self._world = world
        self._grid = self._world.getOccupancyGrid()
        self._numberOfParticles = numberOfParticles
        self._particles = self._generateParticles()

    # --------
    # check: call this method cyclic to run the localisation algorithm
    # while driving
    #
    def check(self):
        xk1 = []

        # Bewegungsmodell: Hier wird auf Partikel i ein zuf�llig generierter Steuerbefehl angewendet. 
#         xk1.append(self._sampleMotionModel(i))
        # Messmodell: Hier wird gepr�ft, wie gut die gemessenen Sensorwerte z_k+1 zur Position x_k1[i] in einer
        # Umgebungskarte (map) m passen. 
        self._measurementModel()
        
        self._particles = self._resampling()
        self._world.drawParticles(self._particles)
        
        
    def _generateParticles(self):
        [xPos, yPos, _] = self._robot.getTrueRobotPose()
        xValues = map(lambda x: (x - 0.5) * 3 + xPos, random.random(self._numberOfParticles))
        yValues = map(lambda x: (x - 0.5) * 3 + yPos, random.random(self._numberOfParticles))
        thetaValues = map(lambda x: x * math.pi * 2, random.random(self._numberOfParticles))
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
        return weight
                
        
        
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
    
        # pull particles
        for _ in range(self._numberOfParticles):
            item = random.uniform(0, maximum)
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
        
            pulledParticles.append(self._particles[m])
        
        return pulledParticles
    
    # --------
    # Add random sample motion to the particle i where 0 <= i < self._particles.length
    #
    def _sampleMotionModel(self, i):
        # TODO
        return 0
        
    # --------
    # Check with Likelihood-Field
    #
    def _measurementModel(self):
        sum = 0
        for i in range(len(self._particles)):
            self._particles[i][self.SUM] = sum
            weight = self.matchParticle(self._particles[i])
            sum += weight
            self._particles[i][self.WEIGHT] = weight
        return 0

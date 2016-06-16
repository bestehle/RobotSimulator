from numpy import random
from numpy import math

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
        self._world.drawParticles(self._particles)
        self._resampling()
        
        
    def _generateParticles(self):
        [xPos, yPos, _] = self._robot.getTrueRobotPose()
        xValues = map(lambda x: (x - 0.5) * 3 + xPos, random.random(self._numberOfParticles))
        yValues = map(lambda x: (x - 0.5) * 3 + yPos, random.random(self._numberOfParticles))
        thetaValues = map(lambda x: x * math.pi * 2, random.random(self._numberOfParticles))
        weightValues = [0] * self._numberOfParticles
        sumValues = [0] * self._numberOfParticles
        
        return list(zip(xValues, yValues, thetaValues, weightValues, sumValues))
        
    def localizeObstacles(self, position):
        obstacles = map(self._calculatePosition, self._robot.getSensorDirections(),
                        self._robot.sense(), [position] * len(self._robot.sense()))
        match = 0
        for obstacle in obstacles:
            if obstacle != None:
                match += max(1, abs(self._world._grid.getValueCell(int(obstacle[0] * 10), int(obstacle[1] * 10)))) - 1
            
        print(match)
        
        
    def _calculatePosition(self, direction, distance, position):
        if distance is None:
            return None
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
        maximum  = sum(p[self.WEIGHT] for p in self._particles)
    
        # pull particles
        for _ in range(self._numberOfParticles):
            item = random.uniform(0, maximum)
            li = 0
            re = self._numberOfParticles - 1
            # binary search
            while li <= re:
                m = int((li + re)/2)

                if item < self._particles[m][self.SUM]:
                    re = m-1
                elif item > self._particles[m][self.SUM]:
                    li = m+1
                else:
                    break
        
            pulledParticles.append(self._particles[m])
        
        return pulledParticles

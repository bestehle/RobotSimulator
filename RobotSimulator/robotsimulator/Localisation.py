'''
Created on 09.06.2016

@author: Benjamin Stehle
'''
import math

import numpy
from robotsimulator.PathPlanning import PathPlanning


class Localisation:
    # --------
    # init: Sets the robot and the world
    #
    def __init__(self, robot, world, particles=100):
        self._robot = robot
        self._world = world
        self._particles = particles
        
        self._pathPlaning = PathPlanning(robot, world)
        self._pathPlaning.brushfire()
#         self._pathPlaning._grid.drawGrid()
#         self._pathPlaning._grid.printGrid()
        
    # --------
    # check: call this method cyclic to run the localisation algorithm
    # while driving
    #
    def check(self):
        print("start localisation")
        particles = self.generateParticles()#
        self._world.drawParticles(particles)
        
        
    def generateParticles(self):
        [xPos, yPos, _] = self._robot.getTrueRobotPose()
        xValues = map(lambda x: (x - 0.5) * 3 + xPos, numpy.random.random(self._particles))
        yValues = map(lambda x: (x - 0.5) * 3 + yPos, numpy.random.random(self._particles))
        thetaValues = map(lambda x: x * math.pi * 2, numpy.random.random(self._particles))
        weightValues = [0] * self._particles
        
        return list(zip(xValues, yValues, thetaValues, weightValues))
        
    def localizeObstacles(self, position):
        obstacles = map(self._calculatePosition, self._robot.getSensorDirections(),
                        self._robot.sense(), [position] * len(self._robot.sense()))
        match = 0
        for obstacle in obstacles:
            if obstacle != None:
                match += max(1, abs(self._pathPlaning._grid.getValueCell(int(obstacle[0] * 10), int(obstacle[1] * 10)))) - 1
            
        print(match)
        
        
    def _calculatePosition(self, direction, distance, position):
        if distance is None:
            return None
        absDirection = position[2] + direction
        x = math.cos(absDirection) * distance
        y = math.sin(absDirection) * distance
        
        return position[0] + x, position[1] + y

'''
Created on 09.06.2016

@author: Benjamin Stehle
'''
import math

import numpy
from robotsimulator.PathPlanning import PathPlanning


class Localistaion(object):
    '''
    classdocs
    '''


    # --------
    # init: Sets the robot and the world
    #
    def __init__(self, robot, world):
        self._robot = robot
        self._world = world
        
        self._pathPlaning = PathPlanning(robot, world)
        self._pathPlaning.brushfire()
#         self._pathPlaning._grid.drawGrid()
#         self._pathPlaning._grid.printGrid()
        
        
    def generateParticles(self, position, particles=100):
        xValues = map(lambda x: (x - 0.5) * 3 + position[0], numpy.random.random(particles))
        yValues = map(lambda x: (x - 0.5) * 3 + position[1], numpy.random.random(particles))
        thetaValues = map(lambda x: x * math.pi * 2, numpy.random.random(particles))
        weightValues = [0] * particles
        
        return zip(xValues, yValues, thetaValues, weightValues)
        
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

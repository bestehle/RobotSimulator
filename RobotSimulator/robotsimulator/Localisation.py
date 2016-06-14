'''
Created on 09.06.2016

@author: Benjamin Stehle
'''
import math

import numpy


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
        
    def generateParticles(self, particles=100):
        xValues = numpy.random.laplace(0, 3, particles)
        yValues = numpy.random.laplace(0, 3, particles)
        thetaValues = map(lambda x: x * math.pi * 2, numpy.random.random(particles))
        weightValues = [0] * particles
        
        return zip(xValues, yValues, thetaValues, weightValues)
        
    def localizeObstacles(self, position):
        positions = map(self.calculatePosition, self._robot.getSensorDirections(),
                        self._robot.sense(), [position] * len(self._robot.sense()))
        
        
        
    def calculatePosition(self, direction, distance, position):
        if distance == None:
            return None
        absDirection = position[2] + direction
        x = math.cos(absDirection) * distance
        y = math.sin(absDirection) * distance
        
        return position[0] + x, position[1] + y

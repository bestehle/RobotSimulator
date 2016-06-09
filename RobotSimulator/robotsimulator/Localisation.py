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
        thetaValues = map(lambda x: x * math.pi * 2, numpy.random.random(20))
        weightValues = [0] * particles
        
        return zip(xValues, yValues, thetaValues, weightValues)
        
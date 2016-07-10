
from numpy import math, random, var

from robotsimulator import Stats
from robotsimulator.Localisation import Localisation


class LocalisationGlobal(Localisation):
    def __init__(self, *args, **keywargs):
        super().__init__(*args, **keywargs)
        self.WEIGHT_LIMIT = 10000000000
        self.VARX_LIMIT = 0.03
        self.VARY_LIMIT = 0.03
        self.isLocalisedCount = 0
        self.IS_LOCALISED_LMIT = 50
        
    def _generateParticles(self):
        xValues = list(map(lambda x: x * self._world._width, random.random(self._numberOfParticles)))
        yValues = list(map(lambda y: y * self._world._height, random.random(self._numberOfParticles)))
        thetaValues = list(map(lambda x: x * math.pi * 2, random.random(self._numberOfParticles)))

        weightValues = [0] * self._numberOfParticles
        sumValues = [0] * self._numberOfParticles
        
        return [list(a) for a in zip(xValues, yValues, thetaValues, weightValues, sumValues)]

    # --------
    # Check how good the particles are with the sensors of the room
    #        
    def _measurementModel(self):
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
            # check each global sense value
            for j in range(len(robotSense)):
                if (None != robotSense[j] and robotSense[j] == pSense[j]):
                    self._particles[i][self.WEIGHT] *= 2
            
            # update the weight sum
            weightSum += self._particles[i][self.WEIGHT]

        # unpack vals
        xVals = list(map(lambda p: p[self.X], self._particles))
        yVals = list(map(lambda p: p[self.Y], self._particles))
        # get the variance
        xVar = var(xVals)
        yVar = var(yVals)

        Stats.globalLocalisationWeightSum(weightSum, '\t', xVar, '\t', yVar, '\t', self.WEIGHT_LIMIT, '\t', self.VARX_LIMIT, '\t', self.VARY_LIMIT)

        if (weightSum == self._numberOfParticles):
            self._particles = self._generateParticles()
            
            
        if (weightSum > self.WEIGHT_LIMIT and xVar < self.VARX_LIMIT and yVar < self.VARY_LIMIT):
            self.isLocalisedCount += 1
        else:
            self.isLocalisedCount = 0
        if (self.isLocalisedCount > 0 and self.isLocalisedCount <= self.IS_LOCALISED_LMIT):
            print ('Localization complete : ', (self.isLocalisedCount / self.IS_LOCALISED_LMIT * 100), '%')
        if (self.isLocalisedCount == self.IS_LOCALISED_LMIT):
            self._localisedListener.emit()
            
            
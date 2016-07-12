import math
import random

from robotsimulator.Localisation import Localisation


class LocalisationLandmarks(Localisation):
    def __init__(self, *args, **keywargs):
        super().__init__(*args, **keywargs)

    # --------
    # Check how good the particles are and set the weight of each particle with the distance to landmarks
    #        
    def _measurementModel(self):
        # get the distance for each landMark to the current approximate robot position
        (robotDistance, robotAngle) = self._robot.senseLandmarks(self._landmarks)

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
                # get the particle angle
                particleAngle = math.atan2(self._particles[i][self.Y] - landmarkY, self._particles[i][self.X] - landmarkX)
                # set (distance of robot to landmark) in relation to (distance of particle to landmark)
                distance = abs(robotDistance[mark] - particleDistance)
                angle = abs((robotAngle[mark] - particleAngle - math.pi) % 2 * math.pi - math.pi)
                # multiply the distance to the weight
                self._particles[i][self.WEIGHT] *= distance * angle
                
            # invert weight to get high value for a good particle
            self._particles[i][self.WEIGHT] = 1 / self._particles[i][self.WEIGHT]
            
            # update the weight sum
            weightSum += self._particles[i][self.WEIGHT]
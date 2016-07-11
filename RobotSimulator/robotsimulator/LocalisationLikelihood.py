from robotsimulator import GeometryHelper
from robotsimulator.Localisation import Localisation


class LocalisationLikelihood(Localisation):
    def __init__(self, *args, **keywargs):
        super().__init__(*args, **keywargs)

    # --------
    # Check how good the particles are and set the weight of each particle with the Likelihood-Field
    #
    def _measurementModel(self):
        weightSum = 0
        for i in range(len(self._particles)):
            self._particles[i][self.SUM] = weightSum
            weight = self._matchParticle(self._particles[i])
            weightSum += weight
            self._particles[i][self.WEIGHT] = weight

    # --------
    # Check how good a single particles is
    #
    def _matchParticle(self, particle):
        weight = 0
        sensorData = zip(self._robot.getSensorDirections(), self._robot.sense())
        for direction, distance in sensorData:
            # robot sensor measured a wall.
            if distance is not None:
                sensorPosition = GeometryHelper.calculatePosition(direction, distance, particle)
                # get the weight of the particle. if the particle
                weight += self._grid.getValueWeight(sensorPosition[0], sensorPosition[1])
            # no wall was measured
            else:
                sensorPosition = GeometryHelper.calculatePosition(direction, 5, particle)
                currentWeight = self._grid.getValueWeight(sensorPosition[0], sensorPosition[1])
                # if currentWeight is unequal OBSTACLE, than the particle saw no wall.
                # That's good, so we increase the weight.
                if currentWeight != self._grid.OBSTACLE:
                    weight += 1
                else:
                    weight += 0
        return weight  #  1000 / (weight + 0.001) TODO
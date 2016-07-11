import math
from robotsimulator import GeometryHelper, Stats


class BoxDetection(object):
    
    def __init__(self, world, robot):
        self._world = world
        self._robot = robot
        self._detectedBoxes = []
        
        self.BOXES_DISTANCES = 0
        self.BOXES_ANGLES = 1
        self.BOX_DETECTION_TOLERANCE = 0.4
            
    # --------
    # Sense boxes.
    # Return [distances, angles] for all sensed boxes.
    # Return None, if no boxes are visible.
    #
    def senseBoxes(self):
        distAngles = self._world.senseBox()
        if distAngles is None or distAngles[0] == []:
            return None
        else:
            return distAngles
    
    # --------
    # Use senseBoxes() to check for near boxes.
    #
    def findBoxes(self):
        # boxes in front
        self._findBoxes(0, 0)
        self._robot.rotate(math.radians(130), 0.2)
        self._findBoxes(10, 0)
        self._robot.rotate(math.radians(130), 0.2)
        self._findBoxes(10, 30)
    
    def _isNewBox(self, position, maxBoxDistance, distance):
        for box in self._detectedBoxes:
            if (position[0] < box[0] + self.BOX_DETECTION_TOLERANCE
            and position[0] > box[0] - self.BOX_DETECTION_TOLERANCE
            and position[1] < box[1] + self.BOX_DETECTION_TOLERANCE
            and position[1] > box[1] - self.BOX_DETECTION_TOLERANCE):
                return False;
        return distance < maxBoxDistance;
        

    def isInNewAngle(self, ignoreRight, ignoreLeft, angle):
        halfSensorAngle = self._world._boxSensorAngle / 2;
        return angle > -halfSensorAngle + ignoreLeft and angle < halfSensorAngle - ignoreRight

    def _findBoxes(self, ignoreRightDegree, ignoreLeftDegree, maxBoxDistance=3.5):
        ignoreRight = math.radians(ignoreRightDegree)
        ignoreLeft = math.radians(ignoreLeftDegree)
        boxes = self.senseBoxes()
        # no boxes found
        if (None == boxes):
            return False
        # check all found boxes
        for i in range(0, len(boxes[self.BOXES_DISTANCES])):
            # distance/angle to the found box
            angle = -boxes[self.BOXES_ANGLES][i]
            distance = boxes[self.BOXES_DISTANCES][i]
            
            if self.isInNewAngle(ignoreRight, ignoreLeft, angle):
                approximateBoxPosition = GeometryHelper.calculatePosition(angle, distance, self._robot.getRobotPose())
                trueBoxPosition = GeometryHelper.calculatePosition(angle, distance, self._world.getTrueRobotPose())
                
                if  self._isNewBox(trueBoxPosition, maxBoxDistance, distance):
                    self._detectedBoxes.append(approximateBoxPosition)
                    print ('Detected Boxes : ' , len(self._detectedBoxes))
                    self._world.drawBox(approximateBoxPosition)
    
                    Stats.boxPositions(round(approximateBoxPosition[0], 3), '\t', round(approximateBoxPosition[1], 3), '\t',
                                       round(trueBoxPosition[0], 3), '\t', round(trueBoxPosition[1], 3))
                    

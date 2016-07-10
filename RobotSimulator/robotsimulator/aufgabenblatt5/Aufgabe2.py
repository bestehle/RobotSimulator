from numpy import math

from robotsimulator import Robot
from robotsimulator.PathPlanning import PathPlanning
from robotsimulator.World import World
from robotsimulator.worlds import officeWorldWithGlobalGrid as officeWorld
from robotsimulator.LocalisationLandmarks import LocalisationLandmarks as LocalLocalisation
from robotsimulator.LocalisationGlobal import LocalisationGlobal as GlobalLocalisation

myWorld = World(20, 20)
myRobot = Robot.Robot()
myWorld = officeWorld.buildWorld()

myWorld.setRobot(myRobot, 4, 6.5, 0 * math.pi)

pathPlanning = PathPlanning(myRobot, myWorld)
pathPlanning._grid.addSafetyDistance(myRobot, 0.1)
pathPlanning._grid.brushfire()

# robot localized, replace robot now to other position (kidnapping)
def localisedRobotFirstTime():
    print ("Robot position is known 1")
    localisation.clearOnLocalisedListener()
    localisation.onLocalised(localisedRobotSecondTime)
    # kidnap robot
    myWorld.setRobot(myRobot, 14, 5, 2 * math.pi)
    
# robot localized use local localization now
def localisedRobotSecondTime():
    print ("Robot position is known 2")
    myRobot.clearOnMoveListener()
    # setup local localization
    localisation = LocalLocalisation(myRobot, myWorld)
    localisation.drawParticles = True
    localisation.printFault = True
    localisation.printPosition = True
    localisation.addLandmark(1, 1)
    localisation.addLandmark(18, 13)
    localisation.addLandmark(1, 13)
    localisation.addLandmark(14, 2)
    localisation.addLandmark(7, 5)
    
    myRobot.onMove(localisation.check)
    myRobot.useApproximatePosition(localisation.getPosition)

localisation = GlobalLocalisation(myRobot, myWorld, numberOfParticles=1000)
localisation.onLocalised(localisedRobotFirstTime)
localisation.drawParticles = True

myRobot.onMove(localisation.check)
myRobot.braitenberg(1, 9, 2, 0.5)

myWorld.close()


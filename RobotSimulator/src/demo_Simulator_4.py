from math import *

import World
import officeWorld


# Office world erzeugen:
myWorld = officeWorld.buildWorld()

# Aus office world Belegtheitsgitter generieren udn zeichnen:
myGrid = myWorld.getOccupancyGrid()
myGrid.drawGrid()

# Alle Raeume der Office world ausgeben:
print(myWorld.getRooms())

# Simulation schliessen:
myWorld.close()
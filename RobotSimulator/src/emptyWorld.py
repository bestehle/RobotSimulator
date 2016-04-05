from World import *

def buildWorld():
    world = World(20, 20)
    world.addBox(10,10)
    world.addBox(15,5)
    return world
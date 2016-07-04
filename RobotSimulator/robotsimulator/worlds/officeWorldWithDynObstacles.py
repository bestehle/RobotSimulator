from robotsimulator.World import *


def buildWorld():
    world = World(19, 14)
    world.addLine(1, 1, 18, 1)
    world.addLine(18, 1, 18, 13)
    world.addLine(18, 13, 1, 13)
    world.addLine(1, 13, 1, 1)


    # room 1 and 2
    #world.addLine(3, 3, 11, 3)
    world.addLine(3, 3, 8.5, 3)
    world.addLine(9.5, 3, 11, 3)

    #world.addLine(11, 6, 3, 6)
    world.addLine(3, 6, 4.5, 6)
    world.addLine(5.5, 6, 11, 6)

    world.addLine(11, 3, 11, 6)
    #world.addLine(7, 3, 7, 6)
    world.addLine(7, 3, 7, 4)
    world.addLine(7, 5, 7, 6)
    world.addLine(3, 6, 3, 3)

    world.defineRoom('Room 01',5,4.5)
    world.defineRoom('Room 02',9,4.5)


    # room 3 and 4
    #world.addLine(3, 8, 11, 8)
    world.addLine(3, 8, 8.5, 8)
    world.addLine(9.5, 8, 11, 8)

    #world.addLine(11, 11, 3, 11)
    world.addLine(3, 11, 4.5, 11)
    world.addLine(5.5, 11, 11, 11)

    world.addLine(11, 8, 11, 11)
    world.addLine(7, 8, 7, 11)
    world.addLine(3, 11, 3, 8)

    world.defineRoom('Room 03',5,9.5)
    world.defineRoom('Room 04',9,9.5)

    # room 5 and 6
    #world.addLine(16, 11, 13, 11)
    world.addLine(13, 11, 14, 11)
    world.addLine(15, 11, 16, 11)

    #world.addLine(13, 7, 16, 7)
    world.addLine(13, 7, 14, 7)
    world.addLine(15, 7, 16, 7)

    world.addLine(13, 3, 16, 3)

    #world.addLine(16, 3, 16, 11)
    world.addLine(16, 3, 16, 4.5)
    world.addLine(16, 5.5, 16, 11)

    world.addLine(13, 11, 13, 3)

    world.defineRoom('Room 05',14.5,9)
    world.defineRoom('Room 06',14.5,4.5)


    # Boxes:
    world.addBox( 3.2,  5.8)
    world.addBox( 3.4, 10.7)
    world.addBox( 5.1,  3.3)
    world.addBox( 6.5, 10.6)
    world.addBox( 7.2,  3.3)
    world.addBox(10.8,  5.8)
    world.addBox(10.5,  8.4)
    world.addBox(13.2,  6.8)
    world.addBox(15.7,  3.4)
    world.addBox(15.5,  6.7)
    world.addBox(15.6,  7.3)
    world.addBox(15.8, 10.6)


    # Dynamic Obstacles:
    world.addDynObstacleLine( 2.0,  4.5,  2.8,  4.5)
    world.addDynObstacleLine( 2.0,  9.5,  2.8,  9.5)
    world.addDynObstacleLine( 7.0,  6.2,  7.0,  7.0)
    world.addDynObstacleLine( 9.0, 11.2,  9.0, 12.0)
    world.addDynObstacleLine(11.2,  4.5, 12.0,  4.5)
    world.addDynObstacleLine(12.0,  9.5, 12.8,  9.5)
    world.addDynObstacleLine(14.5,  2.0, 14.5,  2.8)
    world.addDynObstacleLine(16.2,  7.0, 17.0,  7.0)


    return world
from robotsimulator import GeometryHelper
from copy import copy
from sys import maxsize

class PlanByPermutations:
    def __init__(self, world):
        rooms = world.getRooms()
        (x, y, _) = world.getTrueRobotPose()
        self.start = (x, y)
        # get all permutations
        self.permutations = []
        self.permute(rooms, 0, len(rooms) - 1)
        # the fastest room
        self.fastest = 0
        # current room to visit
        self.current = 0
        
        # minimum distance
        minDistance = maxsize
        # get the permutation with the minimum distance
        for i in range(len(self.permutations)):
            p = self.permutations[i]
            distance = self.getDistance(self.start, p)
            if (distance < minDistance):
                minDistance = distance
                self.fastest = i

    # gets permutations
    def permute(self, rooms, l, r):
        if l==r:
            self.permutations.append(copy(rooms))
        else:
            for i in range(l, r + 1):
                rooms[l], rooms[i] = rooms[i], rooms[l]
                self.permute(rooms, l + 1, r)
                rooms[l], rooms[i] = rooms[i], rooms[l]

    # get the distance to visit all rooms from start
    def getDistance(self, start, rooms):
        path = [start] + list(map(lambda room : (room[1], room[2]), rooms))
        distance = GeometryHelper.pathDistance(path)
        return distance
    
    def printablePath(self):
        return [self.start] + list(map(lambda room : (room[1], room[2]), self.permutations[self.fastest]))
    
    # are there more rooms to visit
    def roomsVisited(self):
        return self.current >= len(self.permutations[self.fastest])

    # return the next path to visit
    def nextRoom(self):
        room = self.permutations[self.fastest][self.current]
        self.current += 1
        print (room)
        return room        

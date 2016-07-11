from math import sqrt
from numpy import math
from robotsimulator import GeometryHelper
from robotsimulator.PriorityQueue import PriorityQueue

class PathPlanning:

    # --------
    # init: Sets the robot and the world
    #
    def __init__(self, robot, world):
        self._robot = robot
        self._world = world
        self._grid = self._world.getOccupancyGrid()

    # --------
    # Get shortest path from start to goal with A* algorithm
    #
    def shortestPath(self, start, goal, weightOfBrushfire=100):
        path = None
        # start and goal are points. Get the index in the grid.
        start = self._grid.transformCoordinateToIndexes(start)
        goal = self._grid.transformCoordinateToIndexes(goal)
        # initialize priority queue with start
        openList = PriorityQueue()
        openList.insert(start, 0)
        
        # map to save the shortest path
        cameFrom = {}
        cameFrom[start] = None
        # map to save the cost to reach a vertex
        cost = {}
        cost[start] = 0

        # visit each vertex
        while not openList.empty():
            # visit current vertex
            current = openList.delMin()
            # if the current vertex is the goal, we have reached the end
            if current == goal:
                path = self._getPath(cameFrom, start, goal);
                #print("Cost of the route: " + str(cost[goal]))
                break

            # visit each neighbor of the current vertex
            for neighbor in self._grid.getNeighbors(current):
                # with weights from brushfire
                weight = (1 / max(1, abs(self._grid.getValueCell(neighbor[0], neighbor[1])))) / weightOfBrushfire
                
                # calculate the cost to reach the neighbor from the current vertex
                updatedCost = cost[current] + self._grid.cost(current, neighbor) + weight
                # save the priority with cost + heuristic
                priority = updatedCost + self.heuristic(goal, neighbor)
                
                # neighbor is unknown (not visited before)
                if neighbor not in cost:
                    cost[neighbor] = updatedCost
                    cameFrom[neighbor] = current
                    openList.insert(neighbor, priority)
                
                # the cost to reach the neighbor decreased. Update the priority
                elif updatedCost < cost[neighbor]:
                    cost[neighbor] = updatedCost
                    cameFrom[neighbor] = current
                    openList.changePriority(neighbor, priority)
        return path

    # --------
    # returns the point with the biggest perpendicular distance from 
    # the Line between the startPoint and the EndPoint of the path
    #
    def getMaxDistance(self, path):
        indexOfMaxDistance = 0
        maxDistance = 0
        for i in range(2, len(path) - 1):
            distance = abs(GeometryHelper.perpendicularDistance(path[0], path[-1], path[i]))
            if distance > maxDistance:
                maxDistance = distance
                indexOfMaxDistance = i
        
        return maxDistance, indexOfMaxDistance
    
    # --------
    # Douglas-Peucker-Algorithm
    #
    def rdp(self, path, epsilon):
        maxDistance, indexOfMaxDistance = self.getMaxDistance(path)
        
        if maxDistance >= epsilon:
            left = self.rdp(path[0:indexOfMaxDistance], epsilon)
            right = self.rdp(path[indexOfMaxDistance:], epsilon)
            return left + right
        else:
            return [path[0], path[-1]]
        return path
    
    # --------
    # returns the distance from a (x,y) to b(x,y)
    # (Euklidischer Abstand)
    #
    def heuristic(self, a, b):
        # a and b are indexes, must have the coordinates in OccupancyGrid
        a = self._grid.transformIndexesToCoordinates(a)
        b = self._grid.transformIndexesToCoordinates(b)
        (x1, y1) = a
        (x2, y2) = b
        distance = math.sqrt(abs(x1 - y1) + abs(x2 - y2))
        return distance
    
    # --------
    # Get the path from a list of vertex
    #  
    def _getPath(self, cameFrom, start, goal):
        path = []
        current = goal

        while current != start:
            current = cameFrom[current]
            path.append(self._grid.transformIndexesToCoordinates(current))
        path.reverse()
        return path

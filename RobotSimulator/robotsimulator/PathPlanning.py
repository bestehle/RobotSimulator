from numpy import math
from robotsimulator.PriorityQueue import PriorityQueue

class PathPlanning:

    # --------
    # init: Sets the robot and the world
    #
    def __init__(self, robot, world):
        # constants
        self.OBSTACLE = 1
        
        self._robot = robot
        self._world = world
        # offset to add to the size of the robot
        self._safetyDistance = 0.1
        self._grid = self._getGrid()

    # --------
    # Get shortest path from start to goal with A* algorithm
    #
    def shortestPath(self, start, goal):
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
                break

            # visit each neighbor of the current vertex
            for neighbor in self._grid.getNeighbors(current):
                # with weights from brushfire
                weight = self._grid.getValueCell(neighbor[0], neighbor[1])
                
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
    # brushfire
    #
    def brushfire(self):
        # get all border cells
        openList = PriorityQueue()
        cost = {}
        
        for yi in range(self._grid.ySize):
            for xi in range(self._grid.xSize):
                # check if it is a border cell
                # - cell need to be a obstacle
                # - cell need to have minimum one neighbor cell 
                if((self._grid.getValueCell(xi,yi) == 1) 
                   and (0 < len(list(self._grid.getNeighbors((xi, yi)))))):
                    # add cell to openList with priority 0
                    cell = (xi,yi)
                    openList.insert(cell, 0)
                    cost[cell] = 0

        while not openList.empty():
            # visit current vertex
            current = openList.delMin()

            # visit each neighbor of the current vertex
            for neighbor in self._grid.getNeighbors(current):               
                # calculate the cost to reach the neighbor from the current vertex
                updatedCost = cost[current] + self._grid.cost(current, neighbor)
                # save the priority with cost
                priority = updatedCost
                
                # neighbor is unknown (not visited before)
                if neighbor not in cost:
                    cost[neighbor] = updatedCost
                    self._grid.setValueCell(current[0],current[1], updatedCost)
                    openList.insert(neighbor, priority)
                
                # the cost to reach the neighbor decreased. Update the priority
                elif updatedCost < cost[neighbor]:
                    cost[neighbor] = updatedCost
                    self._grid.setValueCell(current[0],current[1], updatedCost)
                    openList.changePriority(neighbor, priority)
    
    # --------
    # Ramer-Douglas-Peucker
    #   
    def rdp(self):
        print ("TODO Ramer-Douglas-Peucker-Algo")
    
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

    # --------
    # Get the grid and add the safety distance to the original grid.
    #
    def _getGrid(self):
        # calculate the distance to the obstacles to avoid a crash of the robot.
        robotRadian = self._robot._size/2
        dist = int ((robotRadian + self._safetyDistance) / 0.1)
        # get the grid from the world.
        grid = self._world.getOccupancyGrid()

        # get a list with all obstacles
        obstacles = []
        for y in range(grid.ySize):
            for x in range(grid.xSize):
                if grid.getValueCell(x,y) == self.OBSTACLE:
                    obstacles.append((x, y))

        # add obstacles
        for obstacle in obstacles:
            for x in range(obstacle[0] - dist, obstacle[0] + dist + 1):
                for y in range(obstacle[1] - dist, obstacle[1] + dist + 1):
                    grid.setValueCell(x, y, self.OBSTACLE)

        return grid
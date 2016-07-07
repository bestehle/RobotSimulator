# class OccupancyGrid.
#
# This class define methods to construct a occupancy grid.
#
# The grid cells can be set with addLine and setValue.
# Grid cell values can be get with getValue.
# An occupancy can be drawn or printed.
#
# O. Bittel
# V 2.0; 9.3.2016

import math

from robotsimulator.PriorityQueue import PriorityQueue
from robotsimulator.graphics import graphics
from robotsimulator.graphics.graphics import GraphWin, Point, Rectangle


class OccupancyGrid:

    # --------
    # init: creates a an empty occupancy grid with the given cell size
    #
    def __init__(self, xll, yll, width, height, cellSize=0.1):
        # constants
        self.OBSTACLE = 1
        # define grid:
        self.xSize = int((width + cellSize / 2) / cellSize) + 1
        self.ySize = int((height + cellSize / 2) / cellSize) + 1
        self.grid = [[0 for _ in range(self.ySize)] for _ in range(self.xSize)]
        self.width = width
        self.height = height
        self.cellSize = float(cellSize)
        self.numberOfNeighbors = 8

    def printGrid(self):
        print("xSize*ySize: ", self.xSize, self.ySize)
        for yi in range(self.ySize - 1, -1, -1):
            s = ""
            for xi in range(self.xSize):
                s += "|%2.1f" % abs(self.grid[xi][yi])
            print(s)

    def drawGrid(self, scale=10):
        # define graphic window:
        win = GraphWin("Occupancy Grid", int(800.0 * self.width / self.height), 800, autoflush=False)
        win.setCoords(-self.cellSize, -self.cellSize, self.width + self.cellSize, self.height + 2 * self.cellSize)

        # draw all grid cells:
        for yi in range(self.ySize):
            for xi in range(self.xSize):
                if self.grid[xi][yi] == 1:
                    p1 = Point(xi * self.cellSize, yi * self.cellSize)
                    p2 = Point((xi + 1) * self.cellSize, (yi + 1) * self.cellSize)
                    r = Rectangle(p1, p2)
                    r.setFill('black')
                    r.draw(win)
                else:
                    p1 = Point(xi * self.cellSize, yi * self.cellSize)
                    p2 = Point((xi + 1) * self.cellSize, (yi + 1) * self.cellSize)
                    r = Rectangle(p1, p2)
                    color = min(255, int(abs(self.grid[xi][yi]) * scale))
                    r.setFill(graphics.color_rgb(color, color, color))
                    r.draw(win)

        # close window when click:
        print("click in window to close")
        win.getMouse()  # pause for click in window
        win.close()

    # --------
    # Add new a new line from point (x0,y0) to (x1,y1) to the occupancy grid.
    # Currently only horizontal or vertical lines ar allowed.
    # To Do: Implement Bresenham algorithm for arbitrary lines.
    #
    def addLine(self, x0, y0, x1, y1, value=1):
        if x0 != x1 and y0 != y1:
            raise(ValueError, 'lines must be horizontal or vertical')
        x0_i = int(x0 / self.cellSize + 0.5)
        y0_i = int(y0 / self.cellSize + 0.5)
        # print x0, y0, x1, y1
        if x0 == x1:
            y1_i = int(y1 / self.cellSize + 0.5)
            if y0_i < y1_i:
                for yi in range(y0_i, y1_i + 1):
                    self.grid[x0_i][yi] = value
            else:
                for yi in range(y1_i, y0_i + 1):
                    self.grid[x0_i][yi] = value
        else:
            x1_i = int(x1 / self.cellSize + 0.5)
            if x0_i < x1_i:
                for xi in range(x0_i, x1_i + 1):
                    self.grid[xi][y0_i] = value
            else:
                for xi in range(x1_i, x0_i + 1):
                    self.grid[xi][y0_i] = value


    # --------
    # Set grid value at the coordinate (x,y).
    #
    def setValue(self, x, y, value=1):
        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.width:
            return
        xi = int(x / self.cellSize + 0.5)
        yi = int(y / self.cellSize + 0.5)
        self.grid[xi][yi] = value

    # --------
    # Set grid value at the cell indexes (xi,yi).
    #
    def setValueCell(self, xi, yi, value=1):
        if xi < 0 or xi > self.xSize - 1:
            return
        if yi < 0 or yi > self.ySize - 1:
            return
        self.grid[xi][yi] = value

    # --------
    # Get grid value at the coordinate (x,y).
    #
    def getValue(self, x, y):
        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.height:
            return
        xi = int(x / self.cellSize + 0.5)
        yi = int(y / self.cellSize + 0.5)

        return self.grid[xi][yi]
    
    def getValueWeight(self, x, y):
        value = self.getValue(x, y)
        if value is None:
            x = abs(x) % self.xSize
            y = abs(y) % self.ySize
            if x < 0 or x > self.xSize:
                if 0 < y < self.ySize:
                    value = -x
                else:
                    value = -math.sqrt(x ** 2 + y ** 2)
            value = -y
        if value < 1:
            return 1.0 / (abs(value) + 2)
        return value
    
    # --------
    # Gets the grid value at the cell indexes (xi,yi).
    #
    def getValueCell(self, xi, yi):
        if xi < 0 or xi > self.xSize:
            return
        if yi < 0 or yi > self.ySize:
            return
        return self.grid[xi][yi]

    # --------
    # returns the "eight" neighbors
    #
    def getNeighbors(self, coordinates):
        (x, y) = coordinates
        if (self.numberOfNeighbors == 4):
            # save coordinates of the eight neighbors.
            results = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
        elif (self.numberOfNeighbors == 8):
            # save the coordinates of the eight neighbors
            results = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1), (x + 1, y - 1), (x - 1, y - 1), (x - 1, y + 1), (x + 1, y + 1)]
        else:
            raise Exception('OccupancyGrid', 'numberOfNeighbors need to be 4 or 8')
        # filter all coordinates which are not in bounds or a obstacle
        results = filter(self.inBounds, results)
        results = filter(self.passable, results)
        return results

    # --------
    # true if coordinates is in bounds
    #
    def inBounds(self, coordinates):
        (x, y) = coordinates
        return (0 <= x < self.xSize and 0 <= y < self.ySize)
    
    # --------
    # true if coordinates is not a obstacle
    #
    def passable(self, coordinates):
        (x, y) = coordinates
        return (self.getValueCell(x, y) != 1)
    
    # --------
    # returns the cost of b, default cellSize (0.1m)
    #
    def cost(self, a, b):
        (xa, ya) = a
        (xb, yb) = b
        # diagonals
        if(xb == xa + 1 and yb == ya - 1
           or xb == xa - 1 and yb == ya - 1
           or xb == xa - 1 and yb == ya + 1
           or xb == xa + 1 and yb == ya + 1):
            return math.sqrt(2)
        return 1
    
    def transformCoordinateToIndexes(self, coordinates):
        (x, y) = coordinates
        return (int(x / self.cellSize + 0.5), int(y / self.cellSize + 0.5))
    
    def transformIndexesToCoordinates(self, coordinates):
        (xi, yi) = coordinates
        x = (xi) * self.cellSize
        y = (yi) * self.cellSize
        return (x, y)
    
    # --------
    # brushfire
    #
    def brushfire(self):
        # get all border cells
        openList = PriorityQueue()
        cost = {}
        
        for yi in range(self.ySize):
            for xi in range(self.xSize):
                # check if it is a border cell
                # - cell need not to be a obstacle
                # - cell need to have minimum one neighbor cell which is an obstacle
                if((self.getValueCell(xi, yi) == 0) 
                   and (len(list(self.getNeighbors((xi, yi))))) < self.numberOfNeighbors):
                    # add cell to openList with priority 0
                    cell = (xi, yi)
                    openList.insert(cell, 0)
                    cost[cell] = 0

        while not openList.empty():
            # visit current vertex
            current = openList.delMin()

            # visit each neighbor of the current vertex
            for neighbor in self.getNeighbors(current):               
                # calculate the cost to reach the neighbor from the current vertex
                updatedCost = cost[current] - self.cost(current, neighbor)
                # save the priority with cost
                priority = abs(updatedCost)
                
                # neighbor is unknown (not visited before) or shorter path found
                if neighbor not in cost  or cost[neighbor] < updatedCost:
                    cost[neighbor] = updatedCost
                    self.setValueCell(neighbor[0], neighbor[1], updatedCost)
                    openList.insert(neighbor, priority)
                    
    # --------
    # Add the safety distance to the original grid.
    #
    def addSafetyDistance(self, robot, safetyDistance):
        # calculate the distance to the obstacles to avoid a crash of the robot.
        robotRadian = robot._size / 2
        dist = int ((robotRadian + safetyDistance) / 0.1)

        # get a list with all obstacles
        obstacles = []
        for y in range(self.ySize):
            for x in range(self.xSize):
                if self.getValueCell(x, y) == self.OBSTACLE:
                    obstacles.append((x, y))

        # add obstacles
        for obstacle in obstacles:
            for x in range(obstacle[0] - dist, obstacle[0] + dist + 1):
                for y in range(obstacle[1] - dist, obstacle[1] + dist + 1):
                    self.setValueCell(x, y, self.OBSTACLE)

def test1():
    myGrid = OccupancyGrid(0, 0, 0.8, 0.5)
    myGrid.setValue(0.0, 0.0)
    myGrid.setValue(0.1, 0.0)
    myGrid.setValue(0.3, 0.0)
    myGrid.setValue(0.4, 0.0)
    myGrid.setValue(0.5, 0.0)

    myGrid.setValue(0.0, 0.2)
    myGrid.setValue(0.1, 0.2)
    myGrid.setValue(0.3, 0.2)
    myGrid.setValue(0.4, 0.2)
    myGrid.setValue(0.5, 0.2)

    myGrid.setValue(0.8, 0.4)
    myGrid.setValue(0.8, 0.5)

    myGrid.printGrid()

def test2():
    myGrid = OccupancyGrid(0, 0, 0.8, 0.5)
    myGrid.addLine(0.1, 0.1, 0.7, 0.1)
    myGrid.addLine(0.7, 0.1, 0.7, 0.3)

    myGrid.printGrid()
    myGrid.drawGrid()

if __name__ == "__main__":
    test2()

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

from graphics import *

class OccupancyGrid:

    # --------
    # init: creates a an empty occupancy grid with the given cell size
    #
    def __init__(self, xll, yll, width, height, cellSize = 0.1):
        # define grid:
        self.xSize = int((width+cellSize/2)/cellSize) + 1
        self.ySize = int((height+cellSize/2)/cellSize) + 1
        self.grid = [[0 for x in range(self.ySize)] for x in range(self.xSize)]
        self.width = width
        self.height = height
        self.cellSize = float(cellSize)


    def printGrid(self):
        print("xSize*ySize: ", self.xSize, self.ySize)
        for yi in range(self.ySize-1,-1,-1):
            s = ""
            for xi in range(self.xSize):
                s += "%1d" % self.grid[xi][yi]
            print(s)

    def drawGrid(self):
        # define graphic window:
        win = GraphWin("Occupancy Grid", int(800.0*self.width/self.height), 800, autoflush=False)
        win.setCoords(-self.cellSize, -self.cellSize, self.width+self.cellSize, self.height+2*self.cellSize)

        # draw all grid cells:
        for yi in range(self.ySize):
            for xi in range(self.xSize):
                if self.grid[xi][yi] == 1:
                    p1 = Point(xi*self.cellSize, yi*self.cellSize)
                    p2 = Point((xi+1)*self.cellSize, (yi+1)*self.cellSize)
                    r = Rectangle(p1, p2)
                    r.setFill('black')
                    r.draw(win)

        # close window when click:
        print("click in window to close")
        win.getMouse() # pause for click in window
        win.close()

    # --------
    # Add new a new line from point (x0,y0) to (x1,y1) to the occupancy grid.
    # Currently only horizontal or vertical lines ar allowed.
    # To Do: Implement Bresenham algorithm for arbitrary lines.
    #
    def addLine(self, x0, y0, x1, y1, value = 1):
        if x0 != x1 and y0 != y1:
            raise(ValueError, 'lines must be horizontal or vertical')
        x0_i = int(x0/self.cellSize + 0.5)
        y0_i = int(y0/self.cellSize + 0.5)
        # print x0, y0, x1, y1
        if x0 == x1:
            y1_i = int(y1/self.cellSize + 0.5)
            if y0_i < y1_i:
                for yi in range(y0_i,y1_i+1):
                    self.grid[x0_i][yi] = value
            else:
                for yi in range(y1_i,y0_i+1):
                    self.grid[x0_i][yi] = value
        else:
            x1_i = int(x1/self.cellSize + 0.5)
            if x0_i < x1_i:
                for xi in range(x0_i,x1_i+1):
                    self.grid[xi][y0_i] = value
            else:
                for xi in range(x1_i,x0_i+1):
                    self.grid[xi][y0_i] = value


    # --------
    # Set grid value at the coordinate (x,y).
    #
    def setValue(self, x, y, value = 1):
        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.width:
            return
        xi = int(x/self.cellSize + 0.5)
        yi = int(y/self.cellSize + 0.5)
        self.grid[xi][yi] = value


    # --------
    # Get grid value at the coordinate (x,y).
    #
    def getValue(self, x, y):
        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.width:
            return
        xi = int(x/self.cellSize + 0.5)
        yi = int(y/self.cellSize + 0.5)
        return self.grid[xi][yi]


def test1():
    myGrid = OccupancyGrid(0, 0, 0.8, 0.5)
    myGrid.setValue(0.0,0.0)
    myGrid.setValue(0.1,0.0)
    myGrid.setValue(0.3,0.0)
    myGrid.setValue(0.4,0.0)
    myGrid.setValue(0.5,0.0)

    myGrid.setValue(0.0,0.2)
    myGrid.setValue(0.1,0.2)
    myGrid.setValue(0.3,0.2)
    myGrid.setValue(0.4,0.2)
    myGrid.setValue(0.5,0.2)

    myGrid.setValue(0.8,0.4)
    myGrid.setValue(0.8,0.5)

    myGrid.printGrid()

def test2():
    myGrid = OccupancyGrid(0, 0, 0.8, 0.5)
    myGrid.addLine(0.1,0.1, 0.7,0.1)
    myGrid.addLine(0.7,0.1, 0.7,0.3)

    myGrid.printGrid()
    myGrid.drawGrid()

if __name__ == "__main__":
    test2()
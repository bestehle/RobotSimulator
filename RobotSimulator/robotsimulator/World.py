# class World.
#
# This class contains methods to define a world and
# to simulate movements and distance sensor measurements of a robot.
# The world consists just of a set of walls (as line segments).
# Only the walls can be seen by the distance sensor.
#
# A set of red boxes can be placed.
# The boxes can be sensed by a box sensor.
#
# Moreover, a polyline (i.e. planed path) can be drawn in the world.
#
# From the world an occupancy grid can be generated.
#
# Dynamic obstacles lines can be added.
# Unlike simple static world lines they are not considered
# for generating an occupancy grid.
#
# O. Bittel
# V 2.0; 9.3.2016


from math import *
from random import randint
import time

import numpy as np
from robotsimulator import GeometryHelper
from robotsimulator.CursorController import *
from robotsimulator.OccupancyGrid import *
from robotsimulator.graphics.graphics import *


class World:

    # --------
    # init: creates a an empty world
    # with the given size in [m]
    #
    def __init__(self, width, height):
        # World size:
        self._width = width
        self._height = height

        # Border lines:
        self._xll = 0.0  # x coord of left lower point
        self._yll = 0.0  # y coord of left lower point
        self._lines = []  # List of lines
        ll = Point(self._xll, self._yll)  # lower left
        lr = Point(self._xll + width, self._yll)  # lower right
        ur = Point(self._xll + width, self._yll + height)  # upper right
        ul = Point(self._xll, self._yll + height)  # upper left
        self._lines.append(Line(ll, lr))
        self._lines.append(Line(lr, ur))
        self._lines.append(Line(ur, ul))
        self._lines.append(Line(ul, ll))

        # Dynamic Obstacles:
        self._dynObstacles = set()
        # dynamic obstacles are defined as world lines (self._lines)
        # The set dynObstacles is used to differentiate obstacle lines
        # from simple static world lines.
        # Data set is used instead of list to make getOccupancyGrid more efficient.

        # Boxes:
        self._boxes = []
        self._boxSensor = True
        self._boxSensorAngle = 140 * (pi / 180)  # 140 degree
        self._boxRadius = 0.1  # box radius
        self._boxesSensedDist = []  # Distance to the sensed boxes
        self._boxesSensedAngles = []  # Angles of the sensed boxes

        # Rooms
        self._rooms = []

        # Define graphic window with coordinates and draw borderline
        self._win = GraphWin("HTWG Robot Simulator", int(800.0 * width / height), 800, autoflush=False)
        self._win.setCoords(self._xll - 0.1, self._yll - 0.3, self._xll + width + 0.1, self._yll + height + 0.1)
        for l in self._lines:
            l.draw(self._win)

        # Robot (initialization is done in setRobot()
        self._robot = None
        self._robotCircle = None  # robot is shown as circle with robot position as center
        self._robotTheta = None  # robot's global orientation
        self._robotLine = None  # local x-axis of the robot

        # Sensor values:
        self._sensorShow = True  # Sensor values are shown
        self._sensorDist = []  # Distances to obstacles
        self._sensorPoints = []  # Obstacle points
        self._sensorLines = []  # Sensor beams as lines for drawing

        # Clock:
        self._clockTime = 0.0
        p = Point(self._xll + width / 2, self._yll - 0.1)
        self._clockTimeText = Text(p, "Clock Time %4.2f" % self._clockTime)
        self._clockTimeText.draw(self._win)

        # Occupancy Grid:
        self._grid = None

        # Path history
        self._showPathHistory = True  # False
        self._drivenDistance = 0.0

        # Drawn Polyline:
        self._drawnPolyline = []
        
        # Drawn Particles:
        self._drawnParticles = []
        
        # approximate position of the robot
        self._approximatePosition = None
        
        # dict to store the values of the global localisation grid
        self._globalGrid = {}
        
        self.ROBOT_WAY_COLOR = 'red'

    # --------
    # Get the values from a global localization grid for the position (x,y).
    #
    def _getGlobalSenseValue(self, x, y):
        if (0 > x or x > self._width):
            return None
        if (0 > y or y > self._height):
            return None
        return self._globalGrid[str(x) + '.' + str(y)]

    # --------
    # Draw a polyline.
    #
    def drawGlobalLocalisationGrid(self):
        print(randint(0, 9))
        for y in range(self._height + 1):
            for x in range(self._width + 1):
                points = [Point(x, y), Point(x + 1, y), Point(x + 1, y + 1), Point(x, y + 1)]
                p = Polygon(points)
                p.draw(self._win)
                r = randint(1, 10)
                g = randint(1, 10)
                b = randint(1, 50)
                p.setFill(color_rgb(r, g, b))
                self._globalGrid[str(x) + '.' + str(y)] = r * g * b
                

    # --------
    # Draw a polyline without undraw previous polylines.
    #
    def drawPermanentPolyline(self, poly, color='green'):
        for n in range(len(poly) - 1):
            l = Line(poly[n], poly[n + 1])
            l.draw(self._win)
            l.setFill(color)
            l.setWidth(3)

    # --------
    # Draw a polyline.
    #
    def drawPolyline(self, poly, color='green'):
        self.undrawPolyline()
        for n in range(len(poly) - 1):
            l = Line(poly[n], poly[n + 1])
            l.draw(self._win)
            l.setFill(color)
            l.setWidth(3)
            self._drawnPolyline.append(l)

    # --------
    # Undraw the polyline.
    #
    def undrawPolyline(self):
        if self._drawnPolyline == []:
            return
        for l in self._drawnPolyline:
            l.undraw()
        self._drawnPolyline = []
        
    # --------
    # Draw particles.
    #
    def drawParticles(self, poly, color='black'):
        self.undrawParticles()
        for n in poly[0::2]:
            p = Point(n[0], n[1])
            c = Circle(p, 0.03)
            c.draw(self._win)
            color = min(255, int(n[3]) * 10)
            c.setFill(graphics.color_rgb(255 - color, color, 0))
            self._drawnParticles.append(c)

    # --------
    # Draw Box (x, y, ...)
    #        
    def drawBox(self, approximateBoxPosition):
        p = Point(approximateBoxPosition[0], approximateBoxPosition[1])
        c = Text(p, "+")
        c.setSize(35)
        c.draw(self._win)
        c.setFill('green')

    # --------
    # Draw postion (x, y, theta)
    #        
    def drawApproximatePosition(self, position):
        (x, y, _) = position
        if (self._approximatePosition):
            self._approximatePosition.undraw()
        p = Point(x, y)
        self._approximatePosition = Circle(p, 0.05)
        self._approximatePosition.draw(self._win)
        self._approximatePosition.setFill('yellow')  # --------
    # Draw postion (x, y, theta)
    #        
    def redrawApproximatePosition(self):
        if (self._approximatePosition):
            self._approximatePosition.undraw()
            self._approximatePosition.draw(self._win)
            self._approximatePosition.setFill('yellow')

    # --------
    # Undraw the polyline.
    #
    def undrawParticles(self):
        if self._drawnParticles == []:
            return
        for l in self._drawnParticles:
            l.undraw()
        self._drawnParticles = []


    # --------
    # add new a new line from point (x0,y0) to (x1,y1)
    #
    def addLine(self, x0, y0, x1, y1):
        l = Line(Point(x0, y0), Point(x1, y1))
        self._lines.append(l)
        l.setWidth(5)
        l.setFill('blue')
        l.draw(self._win)

    # --------
    # add new a new obstacle line from point (x0,y0) to (x1,y1)
    #
    def addDynObstacleLine(self, x0, y0, x1, y1):
        l = Line(Point(x0, y0), Point(x1, y1))
        self._lines.append(l)
        self._dynObstacles.add(l)
        l.setWidth(10)
        l.setFill('red')
        l.draw(self._win)

    # --------
    # add new a new round Box at point (x,y).
    #
    def addBox(self, x, y):
        box = Circle(Point(x, y), self._boxRadius)
        box.draw(self._win)
        self._boxes.append(box)

    # --------
    # Define a new room with name n and center position (x,y).
    #
    def defineRoom(self, n, x, y):
        self._rooms.append([n, x, y])
        t = Text(Point(x, y), n)
        t.draw(self._win)

    # --------
    # Return all rooms.
    #
    def getRooms(self):
        return self._rooms

    # --------
    # set the robot at pose (x,y,theta) and draw it.
    #
    def setRobot(self, robot, x, y, theta):
        self._robot = robot
        robot.setWorld(self)  # the robot must know his world

        # Set robot and draw robot:
        c = Point(x, y)
        r = robot.getSize() / 2
        self._robotCircle = Circle(c, r)
        self._robotTheta = theta
        p = Point(x + r * cos(theta), y + r * sin(theta))
        self._robotLine = Line(c, p)  # line shows the robot's orientation
        self._robotCircle.draw(self._win)
        self._robotLine.draw(self._win)
        self._robotLine.setWidth(3)

        # Update status bar
        self._clockTimeText.setText("Clock Time: %4.2f Driven Distance: %4.2f Position: %4.2f, %4.2f, %4.2f "
                                    % (self._clockTime, self._drivenDistance, x, y, theta * 180 / pi))

        # Show all:
        self._udateWindow()
        print ("click in window to start")
        self._win.getMouse()  # pause for click in window
        # k = self.win.getKey()
        # print "key "

    # --------
    # get the true robot pose (x,y,theta).
    #
    def getTrueRobotPose(self):
        x = self._robotCircle.getCenter().getX()
        y = self._robotCircle.getCenter().getY()
        theta = self._robotTheta
        return [x, y, theta]

    # --------
    # move the robot in the direction of his self._robotTheta + dTheta/2 by the length of d
    # and then change the robot's orientation by dTheta.
    # The movement takes dt in clock time (dt is only used to change clock time output).
    # If the robot's movement is not possible because of obstacles, the movement will be not
    # performed and False is returned.
    #
    def moveRobot(self, d, dTheta, dT):
        c = self._robotCircle.getCenter()
        r = self._robotCircle.getRadius()
        x = c.getX()
        y = c.getY()
        theta = self._robotTheta
        dx = d * cos(theta + 0.5 * dTheta)
        dy = d * sin(theta + 0.5 * dTheta)
        nc = Point(x + dx, y + dy)

        if self.getNearestDistance(nc) < r:  # movement is not possible because of obstacles
            print ("Robot stalled: ", x, y, theta)
            # raw_input("Enter: ")
            return False

        # move robot and draw robot:
        self._robotLine.undraw()
        self._robotCircle.move(dx, dy)
        self._robotTheta = (self._robotTheta + dTheta) % (2 * pi)
        p = Point(x + dx + r * cos(self._robotTheta), y + dy + r * sin(self._robotTheta))
        self._robotLine = Line(nc, p)
        self._robotLine.draw(self._win)
        self._robotLine.setWidth(3)

        # Path history:
        self._drivenDistance += d
        if self._showPathHistory == True:
            pathLine = Line(c, nc)
            pathLine.setFill(self.ROBOT_WAY_COLOR)
            pathLine.setWidth(3)
            pathLine.draw(self._win)
        # print x+dx, y+dy, self.robotTheta

         # Clear sensor values, compute new sensor values and draw it:
        self._sensorPoints = []
        self._sensorDist = []
        self.sense()
        self._drawSense()
        self._boxesSensedDist = []
        self._boxesSensedAngles = []
        self.senseBox()

        # Update clock and status bar
        self._clockTime += dT
        self._clockTimeText.setText("Clock Time: %4.2f Driven Distance: %4.2f Position: %4.2f, %4.2f, %4.2f "
                                    % (self._clockTime, self._drivenDistance, x + dx, y + dy, self._robotTheta * 180 / pi))

        # show all
        self.redrawApproximatePosition()
        self._udateWindow()
        return True

    # --------
    # Get the values from the global localization grid for the position p.
    # Position p is a list with (x, y, theta)
    #
    def globalSense(self, p):
        (_, _, pTheta) = p
        numberOfSensors = 36
        dTheta = 360.0 / numberOfSensors
        # sensor directions
        alphas = [(-90.0 + dTheta * i) * (math.pi / 180.0) for i in range(numberOfSensors)]
        # Maximum sensor value for each sensor beam
        distMax = 0.5
        # list to store all recognized sense values
        values = []
        # check all sensors
        for alpha in alphas:
            theta = (pTheta + alpha) % (2 * pi)
            q = GeometryHelper.calculatePosition(theta, distMax, p)
            # point = Point(q[0], q[1])
            # cicle = Circle(point, 0.06)
            # cicle.draw(self._win)
            # cicle.setFill(graphics.color_rgb(255, 0, 0))
            x = int(q[0])
            y = int(q[1])
            value = self._getGlobalSenseValue(x, y)
            values.append(value)
        
        return values

    # --------
    # Compute distance values in the given direction of the robot sensors
    # If sensorShow = True, sensor beams are displayed.
    #
    def sense(self):
        if self._sensorDist == []:
            alphas = self._robot.getSensorDirections()
            distMax = self._robot.getMaxSenseValue()
            p = self._robotCircle.getCenter()
            for alpha in alphas:
                theta = (self._robotTheta + alpha) % (2 * pi)
                q = self.getNearestIntersectionWithBeam(p, theta)
                # print "p: ", p.getX(), p.getY(), theta
                d = World._dist(p, q)
                # print "q: ", q.getX(), q.getY(), d
                if d > distMax:
                    self._sensorDist.append(None)
                    x = p.getX() + distMax * cos(theta)
                    y = p.getY() + distMax * sin(theta)
                    self._sensorPoints.append(Point(x, y))
                    # print "sensorPoint: ", x, y, "\n"
                else:
                    self._sensorDist.append(d)
                    self._sensorPoints.append(q)
                    # print "sensorPoint: ", q.getX(), q.getY(), "\n"
            self._drawSense()
            # print "time: ", self._clockTime, time.clock()

        return self._sensorDist


    # --------
    # Draw sensor beams.
    #
    def _drawSense(self):
        if not self._sensorShow:
            return

        # Undraw sensor beam lines:
        for l in self._sensorLines:
            l.undraw()

        # Draw new sensor beam lines:
        self._sensorLines = []
        p = self._robotCircle.getCenter()
        i = 0
        for q in self._sensorPoints:
            l = Line(p, q)
            if i >= 1 and i < 18:
                l.setFill('red')
            else:
                l.setFill('orange')
            self._sensorLines.append(l)
            l.draw(self._win)
            i += 1
        self._robotCircle.undraw()
        self._robotLine.undraw()
        self._robotCircle.draw(self._win)
        self._robotLine.draw(self._win)
        self._robotLine.setWidth(3)

    # --------
    # If boxSensor = True, try to detect box and
    # compute distance and orientation to the detected boxes.
    #
    def senseBox(self):
        if self._boxSensor == False:
            return None
        if self._boxesSensedDist == []:
            p = self._robotCircle.getCenter()
            for box in self._boxes:
                box.undraw()
                box.setFill('white')
                pb = box.getCenter()
                theta = atan2(pb.getY() - p.getY(), pb.getX() - p.getX())
                # print 'Box check', self._robotTheta, theta
                # angle to box relative to robot's x axis from [-pi,+pi)
                alphaBox = (self._robotTheta - theta + pi) % (2 * pi) - pi
                if abs(alphaBox) <= self._boxSensorAngle / 2:
                    ip = self.getNearestIntersectionWithBeam(p, theta)
                    d = World._dist(p, pb)
                    if World._dist(p, ip) > d:
                        # Box can be seen:
                        # print 'Box can be seen', d, alphaBox
                        box.setFill('red')
                        self.boxDist = World._dist(p, pb)
                        self._boxesSensedDist.append(d)
                        self._boxesSensedAngles.append(alphaBox)
                box.draw(self._win)
        # print "senseBox: ", self._boxesSensedDist,self._boxesSensedAngles
        return [self._boxesSensedDist, self._boxesSensedAngles]

    def getCursorController(self):
        return CursorController(self._win)

    # --------
    # update and draw the actual window.
    # If simulation runs to fast then delay wth a time.sleep()
    def _udateWindow(self):
        # time.sleep(0.05)
        self._win.update()
        # self.win.getMouse() # pause for click in window


    def close(self, waitForClick=True):
        if waitForClick:
            print ("click in window to close")
            self._win.getMouse()  # pause for click in window
        self._win.close()


    # --------
    # compute the nearest intersection point between the beam starting at point p in direction of theta
    # and a line of the world
    #
    def getNearestIntersectionWithBeam(self, p, theta):
        # print "\ntheta= ", theta*180/pi
        if len(self._lines) == 0:
            return None
        dmin = float("inf")
        ip = None
        for line in self._lines:
            # print "line: ", line.getP1().getX(),line.getP1().getY(),line.getP2().getX(),line.getP2().getY()
            q = World._intersectSegmentBeam(p, theta, line)
            if q is not None:
                # print "q =", q.getX(), q.getY(), self._dist(p,q)
                d = World._dist(p, q)
                if d < dmin:
                    dmin = d
                    ip = q
        if ip is None:
            return None
        # l = Line(p,ip)
        # l.setFill('red')
        # l.draw(self.win)
        return ip


    # --------
    # compute the distance to the line of the world which is nearest to p.
    #
    def getNearestDistance(self, p):
        if len(self._lines) == 0:
            return None
        dmin = float("inf")
        for l in self._lines:
            d = World._distPointSegment(p, l)
            # print "Distance to Line: ", l.getP1().getX(), l.getP1().getY(), l.getP2().getX(), l.getP2().getY(), d
            if d < dmin:
                dmin = d
        return dmin


    # --------
    # compute the distance between the points p and q.
    #
    @staticmethod
    def _dist(p, q):
        dx = p.getX() - q.getX()
        dy = p.getY() - q.getY()
        return sqrt(dx * dx + dy * dy)


    # --------
    # compute the distance between point p and segment line.
    #
    @staticmethod
    def _distPointSegment(p, line):
        p1 = line.getP1()
        p2 = line.getP2()
        x1 = p1.getX()
        y1 = p1.getY()
        x2 = p2.getX()
        y2 = p2.getY()
        theta = atan2(y2 - y1, x2 - x1) + pi / 2
        ip = World._intersectSegmentBeam(p, theta, line, oppositeDirectionInclusive=True)
        if ip is None:
            # print "ip = None: "
            d1 = World._dist(p, p1)
            d2 = World._dist(p, p2)
            if d1 <= d2:
                return d1
            else:
                return d2
        else:
            # print "ip: ", ip.getX(), ip.getY()
            return World._dist(ip, p)


    # --------
    # Compute the intersection point between segment line and the beam given by p and theta.
    # If oppositeDirectionInclusive = True, then the intersection point between segment line
    # and the line through point p with grade theta is computed.
    #
    @staticmethod
    def _intersectSegmentBeam(p, theta, line, oppositeDirectionInclusive=False):
        x0 = p.getX()
        y0 = p.getY()
        x1 = line.getP1().getX()
        y1 = line.getP1().getY()
        x2 = line.getP2().getX()
        y2 = line.getP2().getY()

        delta = fabs(atan2(y2 - y1, x2 - x1) - theta)
        if delta < 1.0e-03 or fabs(delta - pi) < 1.0e-03:
            # line and beam are nearly parallel
            # print 'parallel'
            return None

        # Define lines for beam and segment in parametric form
        # (parameter: k0 and k1).
        # Set x and y coordinates equal and solve for k0, k1.
        a = np.array([[x2 - x1, -cos(theta)],
                      [y2 - y1, -sin(theta)]])
        b = np.array([x0 - x1,
                      y0 - y1])
        k = np.linalg.solve(a, b)
        k0 = k[0]
        k1 = k[1]
        if k0 < 0 or k0 > 1:
            return None
        if not oppositeDirectionInclusive and k1 < 0:
            return None
        ip = Point(x1 + k0 * (x2 - x1), y1 + k0 * (y2 - y1))
        return ip

    def getOccupancyGrid(self, cellSize=0.1):
        if self._grid is None:
            self._grid = OccupancyGrid(self._xll, self._yll, self._width, self._height, cellSize)
            for l in self._lines:
                # Note: element check is done by reference.
                # This is valid, since each line is constructed only once.
                if l in self._dynObstacles:
                    continue
                x0 = l.getP1().getX()
                y0 = l.getP1().getY()
                x1 = l.getP2().getX()
                y1 = l.getP2().getY()
                self._grid.addLine(x0, y0, x1, y1)
        return self._grid











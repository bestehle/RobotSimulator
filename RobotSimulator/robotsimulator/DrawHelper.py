from robotsimulator.graphics import graphics
from robotsimulator.graphics.graphics import Point, Line


class DrawHelper:
    def __init__(self, world):
        self.world = world

    def polyline(self, path, color='green'):
        polyline = []
        for (x, y) in path:
            polyline.append(Point(x, y))
        self.world.drawPolyline(polyline, color)
        
    def permanentPolyline(self, path, color='green'):
        polyline = []
        for (x, y) in path:
            polyline.append(Point(x, y))
        self.world.drawPermanentPolyline(polyline, color)
         
    def drawWayOfLocalisation(self, localisation, x, y, theta):
        pathLine = Line(Point(localisation._position[localisation.X], localisation._position[localisation.Y]), Point(x, y))
        color = min(255, int(abs(localisation._position[localisation.THETA] - theta) * 1000))
        pathLine.setFill(graphics.color_rgb(color, 255 - color, 0))
        pathLine.setWidth(2)
        pathLine.draw(self.world._win)
        
    def drawWayOfParticle(self, localisation, i, x, y):
        pathLine = Line(Point(localisation._particles[i][localisation.X], localisation._particles[i][localisation.Y]), Point(x, y))
        pathLine.setFill('green')
        pathLine.setWidth(1)
        pathLine.draw(self.world._win)
from robotsimulator.graphics.graphics import Point

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
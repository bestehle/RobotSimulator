import math

# --------
# returns the perpendicular distance from point to Line(linaA, lineB)
#
def perpendicularDistance(lineA, lineB, point):
    t = lineB[0] - lineA[0], lineB[1] - lineA[1]  # Vector ab
    dd = math.sqrt(t[0] ** 2 + t[1] ** 2)  # Length of ab
    t = t[0] / dd, t[1] / dd  # unit vector of ab
    n = -t[1], t[0]  # normal unit vector to ab
    ac = point[0] - lineA[0], point[1] - lineA[1]  # vector ac
    return ac[0] * n[0] + ac[1] * n[1]  # Projection of ac to n (the minimum distance)

        
def addDegree(a, b):
    return (a + b) % (2 * math.pi)

def diffDegree(a, b):
    return ((a - b + math.pi) % (2 * math.pi)) - math.pi

def pathDistance(path):
    distance = 0
    for i in range(len(path) -1):
        (aX, aY) = path[i]
        (bX, bY) = path[i + 1]
        dx = aX - bX
        dy = aY - bY
        distance += math.sqrt(dx*dx+dy*dy)
    return distance
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
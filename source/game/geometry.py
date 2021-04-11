import math
from typing import Union
from shared import FIELD_DIMS


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    @classmethod
    def fromTopLeft(cls, x: float, y: float):
        return cls(x - FIELD_DIMS[0] / 2 + 0.5, -y + FIELD_DIMS[1] / 2 + 0.5)

    def toTopLeft(self, offset=(0., 0.)):
        return round(FIELD_DIMS[0] / 2 + self.x - 0.5 + offset[0]), round(FIELD_DIMS[1] / 2 - self.y - 0.5 - offset[1])

    def distanceTo(self, p: 'Point' = None):
        p = p or Point(0, 0)
        return math.sqrt((self.x - p.x) ** 2 + (self.y - p.y) ** 2)

    def sideOf(self, a: 'Point', b: 'Point'):
        return math.copysign(1, (a.y - self.y) * (b.x - self.x) - (b.y - self.y) * (a.x - self.x))

    def transform(self, shift: 'Point' = None, angle=0.):
        shift = shift or Point(0, 0)
        sin, cos = math.sin(angle), math.cos(angle)
        return Point(cos * self.x - sin * self.y, sin * self.x + cos * self.y) + shift

    def invTransform(self, shift: 'Point' = None, angle=0.):
        shift = shift or Point(0, 0)
        return (self - shift).transform(angle=-angle)

    def mirror(self, x=True, y=True):
        return Point((-1 if x else 1) * self.x, (-1 if y else 1) * self.y)

    def copy(self):
        return Point(self.x, self.y)

    def __add__(self, p: 'Point'):
        return Point(self.x + p.x, self.y + p.y)

    def __sub__(self, p: 'Point'):
        return Point(self.x - p.x, self.y - p.y)

    def __truediv__(self, f: float):
        return Point(self.x / f, self.y / f)

    def __mul__(self, f: float):
        return Point(self.x * f, self.y * f)


class Line:
    def __init__(self, a: Point, b: Point):
        self.a = a
        self.b = b

    def transform(self, shift=Point(0, 0), angle=0.):
        return Line(self.a.transform(shift, angle), self.b.transform(shift, angle))

    def invTransform(self, shift=Point(0, 0), angle=0.):
        return Line(self.a.invTransform(shift, angle), self.b.invTransform(shift, angle))

    def intersects(self, l: 'Line'):
        return self.a.sideOf(l.a, l.b) * self.b.sideOf(l.a, l.b) <= 0 and \
               l.a.sideOf(self.a, self.b) * l.b.sideOf(self.a, self.b) <= 0

    def mirror(self, x=True, y=True):
        return Line(self.a.mirror(x, y), self.b.mirror(x, y))


class Rectangle:
    def __init__(self, dims: Point, center=Point(0, 0)):
        self.dims = dims
        self.center = center
        self.radius = (self.dims / 2).distanceTo()
        self.l, self.r = center.x - dims.x / 2, center.x + dims.x / 2
        self.b, self.t = center.y - dims.y / 2, center.y + dims.y / 2
        self.corners = [Point(x, y) for x in (self.l, self.r) for y in (self.b, self.t)]

    def contains(self, p: Point):
        return self.l < p.x < self.r and self.b < p.y < self.t

    def intersects(self, l: Line):
        return not any([
            l.a.x < self.l and l.b.x < self.l, self.r < l.a.x and self.r < l.b.x,
            l.a.y < self.b and l.b.y < self.b, self.t < l.a.y and self.t < l.b.y,
            all([self.l < l.a.x < self.r, self.l < l.b.x < self.r, self.b < l.a.y < self.t, self.b < l.b.y < self.t]),
            abs(sum(p.sideOf(l.a, l.b) for p in self.corners)) == 4
        ])

    def mirror(self, x=True, y=True):
        return Rectangle(self.dims, self.center.mirror(x, y))


def xMirrors(g: Union[Point, Line, Rectangle]):
    return [g, g.mirror(True, False)]


def yMirrors(g: Union[Point, Line, Rectangle]):
    return [g, g.mirror(False, True)]


def xyMirrors(g: Union[Point, Line, Rectangle]):
    return [g, g.mirror()]

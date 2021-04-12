import math
from game.geometry import Point, Line, Rectangle, xMirrors, yMirrors, xyMirrors


class FIELD:
    rect = Rectangle(Point(808, 448))
    low_barriers = [  # low barriers B2, B8, B5
        *xyMirrors(Rectangle(Point(80, 20), Point(-214, 0))),
        Rectangle(Point(35.4, 35.4))]
    high_barriers = [  # high barriers B1, B6, B3, B7, B4, B9
        *xyMirrors(Rectangle(Point(100, 20), Point(-354, 114))),
        *xyMirrors(Rectangle(Point(20, 100), Point(-244, -174))),
        *xyMirrors(Rectangle(Point(100, 20), Point(0, 120.5)))]


class ZONE:
    dims = Point(36, 32)
    rects = [  # F4, F1, F5, F2, F3, F6
        *xyMirrors(Rectangle(dims, Point(354, -55))),
        *xyMirrors(Rectangle(dims, Point(214, 59))),
        *xyMirrors(Rectangle(dims, Point(0, 179.5)))]
    hpBuff = 200
    ammoBuff = 100


class ROBOT:
    rect = Rectangle(Point(60, 50))
    center = Point(354, 174)
    hp = 2000
    ammo = 50

    _armorLength = 14
    armors = [  # front, left, right, back
        Line(*yMirrors(Point(rect.dims.x / 2, _armorLength / 2))),
        Line(*xMirrors(Point(_armorLength / 2, rect.dims.y / 2))),
        Line(*xMirrors(Point(_armorLength / 2, -rect.dims.y / 2))),
        Line(*yMirrors(Point(-rect.dims.x / 2, _armorLength / 2)))]
    armorsDamage = [20, 40, 40, 60]

    @staticmethod
    def settleHeat(heat: int, hp: int):  # barrel heat (rules 4.1.2)
        if heat >= 360:
            hp -= (heat - 360) * 40
            heat = 360
        elif heat > 240:
            hp -= (heat - 240) * 4
        heat -= 12 if hp >= 400 else 24
        return max(heat, 0), max(hp, 0)


class MOTION:
    xAccel, xSpeed = 0.25, 1.5
    yAccel, ySpeed = 0.2, 1.2
    rotationAccel, rotationSpeed = 0.005, 0.02
    gimbalYawAccel, gimbalYawSpeed = 0.02, 0.05
    reboundCoefficient = 0.3
    gimbalYawRange = 4 / 3 * math.pi


class BULLET:
    speed = 20
    speedSigma = 1


class TIME:
    step = 10
    second = 100  # changing this will have indirect consequences
    shotCooldown = 8
    heatSettlement = 10
    debuffTimeout = 10 * second
    zoneReset = 60 * second
    gameDuration = 180 * second

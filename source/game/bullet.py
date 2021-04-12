import random
from game.geometry import Point
from game.config import BULLET
from game.robot import Robot


class Bullet:
    def __init__(self, owner: Robot):
        self.ownerId = owner.id_
        self.center = owner.center
        relativeSpeed = Point(random.gauss(BULLET.speed, BULLET.speedSigma), random.gauss(0, BULLET.speedSigma))
        self.speed = relativeSpeed.transform(angle=owner.rotation + owner.gimbalYaw) + owner.speed.transform(angle=owner.rotation)

    def cycle(self):
        self.center += self.speed

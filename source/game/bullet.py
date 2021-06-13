import random
import typing
from game.geometry import Vector, LineSegment
from game.config import BULLET, FIELD


class Bullet:
    def __init__(self, owner):
        self.owner_id = owner.id_
        self.center = owner.center
        relative_speed = Vector(random.gauss(BULLET.speed, BULLET.speed_sigma), random.gauss(0, BULLET.speed_sigma))
        self.speed = relative_speed.transform(angle=owner.rotation + owner.gimbal_yaw)

    def cycle(self, robots: typing.Tuple['Robot', ...]):
        old_center = self.center.copy()
        self.center += self.speed
        trajectory = LineSegment(old_center, self.center)

        return any([
            not FIELD.box.contains(self.center),
            any(r.intersects(trajectory) for r in FIELD.high_barriers),
            any(r.absorbs_bullet(trajectory) for r in robots if r.id_ != self.owner_id)
        ])


# from game.robot import Robot

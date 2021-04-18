import math
import random
from geometry import Vector, LineSegment, Box
from game.bullet import Bullet
from game.config import ROBOT, MOTION, CYCLES, BULLET, FIELD
from shared import Team, RobotState, RobotCommand, RobotNumber


class Robot:
    def __init__(self, team: Team, number: RobotNumber):
        self.team = team
        self.number = number
        self.index = 2 * team.value + number.value  # kinda janky

        self.center = ROBOT.center.mirror(team is Team.blue, number.value == team.value)
        self.rotation = 0. if (team is Team.blue) else math.pi
        self.gimbalYaw = 0.
        self.speed = Vector(0., 0.)
        self.rotationSpeed = 0.
        self.gimbalYawSpeed = 0.

        self.ammo = ROBOT.ammo if (number is RobotNumber.zero) else 0
        self.isShooting = False
        self.shotCooldownCycles = 0
        self.heat = 0
        self.hp = ROBOT.hp
        self.barrierHits = 0
        self.robotHits = 0
        self.can_move = True
        self.can_shoot = True
        self.debuffTimeoutCycles = 0

        self.damage = 0
        self.corners = [p.transform(self.center, self.rotation) for p in ROBOT.box.corners]
        self.armors = [l.transform(self.center, self.rotation) for l in ROBOT.armors]

    def shoot(self):
        if all([self.isShooting, not self.shotCooldownCycles, self.ammo, self.can_shoot]):
            self.ammo -= 1
            self.shotCooldownCycles = CYCLES.shot_cooldown
            self.heat += BULLET.speed
            return Bullet(self)

    def absorbsBullet(self, trajectory: LineSegment):
        if self.hp:
            for armor, armorDamage in zip(self.armors, ROBOT.armors_damage):
                if trajectory.intersects(armor):
                    self.damage += armorDamage
                    self.hp -= armorDamage
                    return True
        return ROBOT.box.intersects(trajectory.invTransform(self.center, self.rotation))

    def hitsBarrier(self, barrier: Box):
        if self.center.distanceTo(barrier.center) < ROBOT.box.radius + barrier.radius and \
                (any(barrier.contains(p) for p in self.corners) or
                 any(ROBOT.box.contains(p.invTransform(self.center, self.rotation)) for p in barrier.corners)):
            self.barrierHits += 1
            return True
        return False

    def hitsRobot(self, robot: 'Robot'):
        if self.center.distanceTo(robot.center) < 2 * ROBOT.box.radius and \
                (any(ROBOT.box.contains(p.invTransform(robot.center, robot.rotation)) for p in self.corners) or
                 any(ROBOT.box.contains(p.invTransform(self.center, self.rotation)) for p in robot.corners)):
            self.robotHits += 1
            return True
        return False

    def control(self, command: RobotCommand):
        if self.can_move and self.hp:
            self.speed.x = self._throttleResponse(self.speed.x, command.x, MOTION.x_accel, MOTION.x_speed)
            self.speed.y = self._throttleResponse(self.speed.y, command.y, MOTION.y_accel, MOTION.y_speed)
            self.rotationSpeed = self._throttleResponse(self.rotationSpeed, command.rotation, MOTION.rotation_accel, MOTION.rotation_speed)
            self.gimbalYawSpeed = self._throttleResponse(self.gimbalYawSpeed, command.gimbal_yaw, MOTION.gimbal_yaw_accel, MOTION.gimbal_yaw_speed)
        if self.can_shoot and self.hp:
            self.isShooting = bool(command.shoot)

    @staticmethod
    def _throttleResponse(speed: float, throttle: float, topAccel: float, topSpeed: float):
        newSpeed = speed + (throttle - speed / topSpeed) * topAccel
        return math.copysign(min(abs(newSpeed), topSpeed), newSpeed)

    def interferes(self, robots):
        return any([
            any(not FIELD.box.contains(p) for p in self.corners),
            any(self.hitsBarrier(r) for r in [*FIELD.low_barriers, *FIELD.high_barriers]),
            any(self.hitsRobot(r) for r in robots if r != self)])

    def cycle(self, cyclesRemaining, robots):
        self.debuffTimeoutCycles -= min(1, self.debuffTimeoutCycles)
        self.shotCooldownCycles -= min(1, self.shotCooldownCycles)

        if not cyclesRemaining % CYCLES.heat_settlement:
            self.heat, self.hp = ROBOT.settle_heat(self.heat, self.hp)
        if not self.debuffTimeoutCycles:
            self.can_move = True
            self.can_shoot = True
        if not self.hp:
            return

        if any([self.speed.x, self.speed.y, self.rotationSpeed]):  # move chassis
            oldCenter, oldRotation, oldCorners = self.center.copy(), self.rotation, self.corners.copy()
            self.center += self.speed.transform(angle=oldRotation)
            self.rotation = (self.rotation + self.rotationSpeed) % (2 * math.pi)
            self.corners = [p.transform(self.center, self.rotation) for p in ROBOT.box.corners]

            if self.interferes(robots):
                self.rotationSpeed *= -MOTION.rebound_coefficient
                self.speed *= -MOTION.rebound_coefficient
                self.center, self.rotation, self.corners = oldCenter, oldRotation, oldCorners
        self.gimbalYaw = min(max(self.gimbalYaw + self.gimbalYawSpeed, -MOTION.gimbal_yaw_range / 2), MOTION.gimbal_yaw_range / 2)
        self.armors = [l.transform(self.center, self.rotation) for l in ROBOT.armors]

    def getState(self):
        return RobotState(
            x=self.center.x,
            y=self.center.y,
            rotation=self.rotation,
            gimbal_yaw=self.gimbalYaw,
            ammo=self.ammo,
            heat=self.heat,
            hp=self.hp,
            can_move=self.can_move,
            can_shoot=self.can_shoot,
            debuff_timeout=self.debuffTimeoutCycles / CYCLES.second
        )
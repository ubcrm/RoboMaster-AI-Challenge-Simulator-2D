import math
from game.geometry import Point, Line, Rectangle
from game.config import ROBOT, MOTION, TIME
from shared import Team, RobotState, RobotCommand, PI


class Robot:
    def __init__(self, number: int, team: Team):
        self.number = number
        self.team = team
        self.id_ = 2 * number + team.value

        self.center = ROBOT.center.mirror(team is Team.blue, number == team.value)
        self.rotation = 0. if (team is Team.blue) else PI
        self.gimbalYaw = 0.
        self.speed = Point(0., 0.)
        self.rotationSpeed = 0.
        self.gimbalYawSpeed = 0.

        self.ammo = ROBOT.ammo if (number == 0) else 0
        self.isShooting = False
        self.shotCooldownSteps = 0
        self.heat = 0
        self.hp = ROBOT.hp
        self.barrierHits = 0
        self.robotHits = 0
        self.canMove = True
        self.canShoot = True
        self.debuffTimeoutSteps = 0

        self.damage = 0
        self.corners = [p.transform(self.center, self.rotation) for p in ROBOT.rect.corners]
        self.armors = [l.transform(self.center, self.rotation) for l in ROBOT.armors]

    def absorbsBullet(self, trajectory: Line):
        if self.hp:
            for armor, armorDamage in zip(self.armors, ROBOT.armorsDamage):
                if trajectory.intersects(armor):
                    self.damage += armorDamage
                    self.hp -= armorDamage
                    return True
        return ROBOT.rect.intersects(trajectory.invTransform(self.center, self.rotation))

    def hitsBarrier(self, barrier: Rectangle):
        if self.center.distanceTo(barrier.center) < ROBOT.rect.radius + barrier.radius and \
                (any(barrier.contains(p) for p in self.corners) or
                 any(ROBOT.rect.contains(p.invTransform(self.center, self.rotation)) for p in barrier.corners)):
            self.barrierHits += 1
            return True
        return False

    def hitsRobot(self, robot: 'Robot'):
        if self.center.distanceTo(robot.center) < 2 * ROBOT.rect.radius and \
                (any(ROBOT.rect.contains(p.invTransform(robot.center, robot.rotation)) for p in self.corners) or
                 any(ROBOT.rect.contains(p.invTransform(self.center, self.rotation)) for p in robot.corners)):
            self.robotHits += 1
            return True
        return False

    def control(self, command: RobotCommand):
        if self.canMove and self.hp:
            self.speed.x = self._throttleResponse(self.speed.x, command.x, MOTION.xAccel, MOTION.xSpeed)
            self.speed.y = self._throttleResponse(self.speed.y, command.y, MOTION.yAccel, MOTION.ySpeed)
            self.rotationSpeed = self._throttleResponse(self.rotationSpeed, command.rotation, MOTION.rotationAccel, MOTION.rotationSpeed)
            self.gimbalYawSpeed = self._throttleResponse(self.gimbalYawSpeed, command.gimbalYaw, MOTION.gimbalYawAccel, MOTION.gimbalYawSpeed)
        if self.canShoot and self.hp:
            self.isShooting = bool(command.shoot)

    @staticmethod
    def _throttleResponse(speed: float, throttle: float, topAccel: float, topSpeed: float):
        newSpeed = speed + (throttle - speed / topSpeed) * topAccel
        return math.copysign(min(abs(newSpeed), topSpeed), newSpeed)

    def getState(self):
        return RobotState(
            x=self.center.x,
            y=self.center.y,
            rotation=self.rotation,
            gimbalYaw=self.gimbalYaw,
            xSpeed=self.speed.x,
            ySpeed=self.speed.y,
            rotationSpeed=self.rotationSpeed,
            gimbalYawSpeed=self.gimbalYawSpeed,
            ammo=self.ammo,
            heat=self.heat,
            hp=self.hp,
            isShooting=self.isShooting,
            shotCooldown=self.shotCooldownSteps / TIME.second,
            barrierHits=self.barrierHits,
            robotHits=self.robotHits,
            canMove=self.canMove,
            canShoot=self.canShoot,
            debuffTimeout=self.debuffTimeoutSteps / TIME.second
        )

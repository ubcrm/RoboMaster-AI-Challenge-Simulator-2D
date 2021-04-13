import math
from game.geometry import Line
from game.robot import Robot
from game.bullet import Bullet
from game.config import TIME, ROBOT, FIELD, BULLET, MOTION
from shared import Team, Winner, RobotCommand, GameState, TeamState


class Game:
    def __init__(self):
        self._state: GameState = None
        self._cyclesRemaining = self._blueRobots = self._redRobots = self._bullets = self._zones = None

    def reset(self):
        self._cyclesRemaining = TIME.gameDuration
        self._blueRobots = [Robot(Team.blue, i) for i in range(2)]
        self._redRobots = [Robot(Team.red, i) for i in range(2)]
        self._bullets = []
        self._state = self._getState()
        return self._state

    def step(self, blueCommands: tuple[RobotCommand], redCommands: tuple[RobotCommand]):
        for robot, command in zip([*self._blueRobots, *self._redRobots], [*blueCommands, *redCommands]):
            robot.control(command)
        for _ in range(TIME.step):
            self._cycle()
        self._state = self._getState()
        return self._state

    def _getState(self):
        blueDamageOutput = sum(r.damage for r in self._redRobots)
        redDamageOutput = sum(r.damage for r in self._blueRobots)
        bluesDead = all(r.hp == 0 for r in self._blueRobots)
        redsDead = all(r.hp == 0 for r in self._redRobots)
        winner = Winner.tbd

        if redsDead and not bluesDead:
            winner = Winner.blue
        elif bluesDead and not redsDead:
            winner = Winner.red
        elif self._cyclesRemaining <= 0:
            if blueDamageOutput > redDamageOutput:
                winner = Winner.blue
            elif redDamageOutput > blueDamageOutput:
                winner = Winner.red
            else:
                winner = Winner.tied
        return GameState(
            timeRemaining=self._cyclesRemaining / TIME.second,
            blueState=TeamState(robotStates=(r.getState() for r in self._blueRobots), damageOutput=blueDamageOutput),
            redState=TeamState(robotStates=(r.getState() for r in self._blueRobots), damageOutput=redDamageOutput),
            winner=winner)

    def _cycle(self):
        for robot in [*self._blueRobots, *self._redRobots]:
            self._cycleRobot(robot)
        for index in reversed(range(len(self._bullets))):
            if self._cycleBullet(self._bullets[index]):
                del self._bullets[index]
        self._cyclesRemaining -= 1

    def _cycleRobot(self, robot: Robot):
        robot.debuffTimeoutCycles -= min(1, robot.debuffTimeoutCycles)
        robot.shotCooldownCycles -= min(1, robot.shotCooldownCycles)

        if not self._cyclesRemaining % TIME.heatSettlement:
            robot.heat, robot.hp = ROBOT.settleHeat(robot.heat, robot.hp)
        if not robot.debuffTimeoutCycles:
            robot.canMove = True
            robot.canShoot = True
        if not robot.hp:
            return

        if any([robot.speed.x, robot.speed.y, robot.rotationSpeed]):  # move chassis
            center, rotation, corners = robot.center.copy(), robot.rotation, robot.corners.copy()
            robot.center += robot.speed.transform(angle=rotation)
            robot.rotation = (robot.rotation + robot.rotationSpeed) % (2 * math.pi)
            robot.corners = [p.transform(robot.center, robot.rotation) for p in ROBOT.rect.corners]

            if self._robotInterferes(robot):
                robot.rotationSpeed *= -MOTION.reboundCoefficient
                robot.speed *= -MOTION.reboundCoefficient
                robot.center, robot.rotation, robot.corners = center, rotation, corners

        robot.gimbalYaw = min(max(robot.gimbalYaw + robot.gimbalYawSpeed, -MOTION.gimbalYawRange / 2), MOTION.gimbalYawRange / 2)
        robot.armors = [l.transform(robot.center, robot.rotation) for l in ROBOT.armors]

        if all([robot.isShooting, not robot.shotCooldownCycles, robot.ammo, robot.canShoot]):  # shoot
            self._bullets.append(Bullet(robot))
            robot.ammo -= 1
            robot.shotCooldownCycles = TIME.shotCooldown
            robot.heat += BULLET.speed

    def _cycleBullet(self, bullet: Bullet):
        center = bullet.center.copy()
        bullet.cycle()
        trajectory = Line(center, bullet.center)

        return any([
            not FIELD.rect.contains(bullet.center),
            any(r.intersects(trajectory) for r in FIELD.high_barriers),
            any(r.absorbsBullet(trajectory) for r in [*self._blueRobots, *self._redRobots] if r.id_ != bullet.ownerId)
        ])

    def _robotInterferes(self, robot: Robot):
        return any([
            any(not FIELD.rect.contains(p) for p in robot.corners),
            any(robot.hitsBarrier(r) for r in [*FIELD.low_barriers, *FIELD.high_barriers]),
            any(robot.hitsRobot(r) for r in [*self._blueRobots, *self._redRobots] if r != robot)])

from game.geometry import Line
from game.robot import Robot
from game.bullet import Bullet
from game.config import TIME, ROBOT, FIELD, BULLET, MOTION
from shared import GameMode, Team, Winner, RobotCommand, GameState, PI


class Game:
    def __init__(self, mode=GameMode.twoVsTwo):
        self.stepsRemaining = TIME.gameDuration
        self.blueRobots = [Robot(n, Team.blue) for n in range(mode.value)]
        self.redRobots = [Robot(n, Team.red) for n in range(mode.value)]
        self.bullets = []

    def advance(self, blueCommands: tuple[RobotCommand], redCommands: tuple[RobotCommand]):
        for robot, command in zip(self.blueRobots, blueCommands):
            robot.control(command)
        for robot, command in zip(self.redRobots, redCommands):
            robot.control(command)
        for _ in range(TIME.cycle):
            self._step()

        blueDamageOutput = sum(r.damage for r in self.redRobots)
        redDamageOutput = sum(r.damage for r in self.blueRobots)
        bluesDead = all(r.hp == 0 for r in self.blueRobots)
        redsDead = all(r.hp == 0 for r in self.redRobots)
        winner = Winner.tbd

        if redsDead and not bluesDead:
            winner = Winner.blue
        elif bluesDead and not redsDead:
            winner = Winner.red
        elif self.stepsRemaining <= 0:
            if blueDamageOutput > redDamageOutput:
                winner = Winner.blue
            elif redDamageOutput > blueDamageOutput:
                winner = Winner.red
            else:
                winner = Winner.tied

        return GameState(
            timeRemaining=self.stepsRemaining / TIME.second,
            blueRobotsState=(r.getState() for r in self.blueRobots),
            redRobotsState=(r.getState() for r in self.redRobots),
            blueDamageOutput=blueDamageOutput,
            redDamageOutput=redDamageOutput,
            winner=winner)

    def _step(self):
        for robot in [*self.blueRobots, *self.redRobots]:
            self._stepRobot(robot)

        for index in reversed(range(len(self.bullets))):
            if self._stepBullet(self.bullets[index]):
                del self.bullets[index]
        self.stepsRemaining -= 1

    def _stepRobot(self, robot: Robot):
        robot.debuffTimeoutSteps -= min(1, robot.debuffTimeoutSteps)
        robot.shotCooldownSteps -= min(1, robot.shotCooldownSteps)

        if not self.stepsRemaining % TIME.heatSettlement:
            robot.heat, robot.hp = ROBOT.settleHeat(robot.heat, robot.hp)
        if not robot.debuffTimeoutSteps:
            robot.canMove, robot.canShoot = True, True

        if not robot.hp:
            return

        if any([robot.speed.x, robot.speed.y, robot.rotationSpeed]):  # move chassis
            center, rotation, corners = robot.center.copy(), robot.rotation, robot.corners.copy()
            robot.center += robot.speed.transform(angle=rotation)
            robot.rotation = (robot.rotation + robot.rotationSpeed) % (2 * PI)
            robot.corners = [p.transform(robot.center, robot.rotation) for p in ROBOT.rect.corners]

            if self._robotInterferes(robot):
                robot.rotationSpeed *= -MOTION.reboundCoefficient
                robot.speed *= -MOTION.reboundCoefficient
                robot.center, robot.rotation, robot.corners = center, rotation, corners

        robot.gimbalYaw = min(max(robot.gimbalYaw + robot.gimbalYawSpeed, -MOTION.gimbalYawRange / 2), MOTION.gimbalYawRange / 2)
        robot.armors = [l.transform(robot.center, robot.rotation) for l in ROBOT.armors]

        if all([robot.isShooting, not robot.shotCooldownSteps, robot.ammo, robot.canShoot]):  # shoot
            self.bullets.append(Bullet(robot))
            robot.ammo -= 1
            robot.shotCooldownSteps = TIME.shotCooldown
            robot.heat += BULLET.speed

    def _stepBullet(self, bullet: Bullet):
        center = bullet.center.copy()
        bullet.step()
        trajectory = Line(center, bullet.center)

        return any([
            not FIELD.rect.contains(bullet.center),
            any(r.intersects(trajectory) for r in FIELD.high_barriers),
            any(r.absorbsBullet(trajectory) for r in [*self.blueRobots, *self.redRobots] if r.id_ != bullet.ownerId)
        ])

    def _robotInterferes(self, robot: Robot):
        return any([
            any(not FIELD.rect.contains(p) for p in robot.corners),
            any(robot.hitsBarrier(r) for r in [*FIELD.low_barriers, *FIELD.high_barriers]),
            any(robot.hitsRobot(r) for r in [*self.blueRobots, *self.redRobots] if r != robot)])

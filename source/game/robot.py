import math
import typing

from game.geometry import Vector
from game.bullet import Bullet
from game.config import ROBOT, MOTION, CYCLES, BULLET, FIELD
from shared import RobotState

def get_max_speed_at_current_direction(x_speed_without_restriction, y_speed_without_restriction, rotation_speed_without_restriction, x_max_speed, y_max_speed, rotation_max_speed):
    x_speed_without_restriction_abs = math.fabs(x_speed_without_restriction)
    y_speed_without_restriction_abs = math.fabs(y_speed_without_restriction)
    rotation_speed_without_restriction_abs = math.fabs(rotation_speed_without_restriction)

    print("in get_max_speed_at_current_direction")
    print("x_speed_without_restriction_abs")
    print("y_speed_without_restriction_abs")
    print("rotation_speed_without_restriction")
    print("x_max_speed")
    print("y_max_speed")
    print("rotation_max_speed")
    print(x_speed_without_restriction_abs)
    print(y_speed_without_restriction_abs)
    print(rotation_speed_without_restriction)
    print(x_max_speed)
    print(y_max_speed)
    print(rotation_max_speed)

    percentageX = x_speed_without_restriction_abs / x_max_speed
    percentageY = y_speed_without_restriction_abs / y_max_speed
    percentageR = rotation_speed_without_restriction_abs / rotation_max_speed

    totalPercentage = percentageX + percentageY + percentageR # can be greater than 1
    print("total percentage")
    print(totalPercentage)
    print("percentageX")
    print(percentageX)
    print("percentageY")
    print(percentageY)
    print("percentageR")
    print(percentageR)
    if(totalPercentage > 1):
        print("need to reduce ---------------------------------------------------")
        return (math.sqrt( math.pow(x_speed_without_restriction / totalPercentage, 2) + math.pow(y_speed_without_restriction / totalPercentage, 2) + math.pow(rotation_speed_without_restriction / totalPercentage, 2)))
    else:
        print("no need to reduce")
        return math.sqrt( math.pow(x_speed_without_restriction, 2) + math.pow(y_speed_without_restriction, 2) + math.pow(rotation_speed_without_restriction, 2))

    # return math.sqrt(
    #             math.pow(
    #                 (rotation_max_speed -
    #                 x_speed_without_restriction_abs * (rotation_max_speed / x_max_speed) -
    #                 y_speed_without_restriction_abs * (rotation_max_speed / y_max_speed)) ,2) +\
    #             math.pow(
    #                 (x_max_speed -
    #                 rotation_speed_without_restriction * (x_max_speed / rotation_max_speed) -
    #                 y_speed_without_restriction_abs * (x_max_speed / y_max_speed)) ,2) +\
    #             math.pow(
    #                 (y_max_speed -
    #                 rotation_speed_without_restriction * (y_max_speed / rotation_max_speed) -
    #                 x_speed_without_restriction_abs * (y_max_speed / x_max_speed)) ,2)
    #             )



class Robot:
    def __init__(self, id_: int):
        self.id_ = id_
        self.is_blue = ROBOT.id_is_blue[id_]
        self.is_zero = ROBOT.id_is_zero[id_]

        self.center = ROBOT.center.mirror(self.is_blue, self.is_blue == self.is_zero)
        self.rotation = 0. if self.is_blue else math.pi
        self.gimbal_yaw = 0.
        self.speed = Vector(0., 0.)
        self.rotation_speed = 0.
        self.gimbal_yaw_speed = 0.

        self.ammo = ROBOT.ammo if self.is_zero else 0
        self.is_shooting = False
        self.shot_cooldown_cycles = 0
        self.heat = 0
        self.hp = ROBOT.hp
        self.barrier_hits = 0
        self.robot_hits = 0
        self.can_move = True
        self.can_shoot = True
        self.debuff_timeout_cycles = 0

        self.damage = 0
        self.corners = [c.transform(self.center, self.rotation) for c in ROBOT.box.corners]
        self.armors = [a.transform(self.center, self.rotation) for a in ROBOT.armors]

    @property
    def state(self):
        return RobotState(
            x=self.center.x,
            y=self.center.y,
            rotation=self.rotation,
            gimbal_yaw=self.gimbal_yaw,
            ammo=self.ammo,
            heat=self.heat,
            hp=self.hp,
            can_move=self.can_move,
            can_shoot=self.can_shoot,
            debuff_timeout=self.debuff_timeout_cycles / CYCLES.second
        )

    def shoot(self):
        if all([self.is_shooting, not self.shot_cooldown_cycles, self.ammo, self.can_shoot]):
            self.ammo -= 1
            self.shot_cooldown_cycles = CYCLES.shot_cooldown
            self.heat += BULLET.speed
            return Bullet(self)

    def absorbs_bullet(self, trajectory: 'LineSegment'):
        if self.hp:
            for armor, armorDamage in zip(self.armors, ROBOT.armors_damage):
                if trajectory.intersects(armor):
                    self.damage += armorDamage
                    self.hp -= armorDamage
                    return True
        return ROBOT.box.intersects(trajectory.inv_transform(self.center, self.rotation))

    def hits_barrier(self, barrier: 'Box'):
        if self.center.distance_to(barrier.center) < ROBOT.box.radius + barrier.radius and \
                (any(barrier.contains(p) for p in self.corners) or
                 any(ROBOT.box.contains(p.inv_transform(self.center, self.rotation)) for p in barrier.corners)):
            self.barrier_hits += 1
            return True
        return False

    def hits_robot(self, robot: 'Robot'):
        if self.center.distance_to(robot.center) < 2 * ROBOT.box.radius and \
                (any(ROBOT.box.contains(p.inv_transform(robot.center, robot.rotation)) for p in self.corners) or
                 any(ROBOT.box.contains(p.inv_transform(self.center, self.rotation)) for p in robot.corners)):
            self.robot_hits += 1
            return True
        return False

    def hits(self, robots: typing.Tuple['Robot', ...]):
        return any([
            any(not FIELD.box.contains(p) for p in self.corners),
            any(self.hits_barrier(r) for r in [*FIELD.low_barriers, *FIELD.high_barriers]),
            any(self.hits_robot(r) for r in robots if r != self)])

    def control(self, command: 'RobotCommand'):
        if self.can_move and self.hp:
            self.gimbal_yaw_speed = self._throttle_response(self.gimbal_yaw_speed, command.gimbal_yaw, MOTION.gimbal_yaw_accel,
                                                            MOTION.gimbal_yaw_speed)
            print("begin")
            print("call _throttle_response2 0-1 x")
            x_speed_without_restriction = self._throttle_response2(self.speed.x, command.x, MOTION.x_accel, MOTION.x_speed, command.x ,command.y ,command.rotation, 1, 1, 1)# MOTION.x_accel, MOTION.y_accel, MOTION.rotation_accel)
            print("call _throttle_response2 0-1 y")
            y_speed_without_restriction = self._throttle_response2(self.speed.y, command.y, MOTION.y_accel, MOTION.y_speed, command.x ,command.y ,command.rotation, 1, 1, 1)# MOTION.x_accel, MOTION.y_accel, MOTION.rotation_accel)
            print("call _throttle_response2 0-1 rotate")
            rotatation_speed_without_restriction = self._throttle_response2(self.rotation_speed, command.rotation, MOTION.rotation_accel, MOTION.rotation_speed, command.x ,command.y ,command.rotation,1, 1, 1)# MOTION.x_accel, MOTION.y_accel, MOTION.rotation_accel)

            print("x_speed_without_restriction in method control")
            print(x_speed_without_restriction)
            print("y_speed_without_restriction in method control")
            print(rotatation_speed_without_restriction)
            print("rotatation_speed_without_restriction in method control")
            print(rotatation_speed_without_restriction)
            print("call cap_speed_under_max 0-1")
            self.speed.x, self.speed.y, self.rotation_speed = self.cap_speed_under_max(x_speed_without_restriction,
                                                                                       y_speed_without_restriction,
                                                                                       rotatation_speed_without_restriction,
                                                                                       MOTION.x_speed,
                                                                                       MOTION.y_speed,
                                                                                       MOTION.rotation_speed)
            print("out cap_speed_under_max 1-0")
            # also might need to be tested and debugged here
        if self.can_shoot and self.hp:
            self.is_shooting = bool(command.shoot)


    def cap_speed_under_max(self, x_speed_without_restriction, y_speed_without_restriction, rotatation_speed_without_restriction , x_max_speed, y_max_speed, rotatation_max_speed):
        multiplier = 1
        print("in cap_speed_under_max 1")
        print("x_speed_without_restriction in cap_speed_under_max")
        print("y_speed_without_restriction in cap_speed_under_max")
        print("rotatation_speed_without_restriction in cap_speed_under_max")
        print(x_speed_without_restriction)
        print(y_speed_without_restriction)
        print(rotatation_speed_without_restriction)

        current_speed_without_restriction_abs = math.sqrt(math.pow(x_speed_without_restriction, 2) + math.pow(y_speed_without_restriction, 2)  + math.pow(y_speed_without_restriction, 2))
        x_speed_without_restriction_square = math.pow(x_speed_without_restriction, 2)
        y_speed_without_restriction_square=math.pow(y_speed_without_restriction, 2)
        rotatation_speed_without_restriction_square=math.pow(rotatation_speed_without_restriction, 2)
        current_speed_without_restriction_abs=math.sqrt(x_speed_without_restriction_square
                                                        +y_speed_without_restriction_square
                                                        +rotatation_speed_without_restriction_square)
        print("x_speed_without_restriction_square")
        print("y_speed_without_restriction_square")
        print("rotatation_speed_without_restriction_square")
        print(x_speed_without_restriction_square)
        print(y_speed_without_restriction_square)
        print(rotatation_speed_without_restriction_square)
        print("call get_max_speed_at_current_direction 1-1")
        max_speed_at_current_direction = get_max_speed_at_current_direction(x_speed_without_restriction,
                                                                            y_speed_without_restriction,
                                                                            rotatation_speed_without_restriction,
                                                                            x_max_speed,
                                                                            y_max_speed,
                                                                            rotatation_max_speed)
        print("out get_max_speed_at_current_direction 2-1")
        print("no need to reduce speed")
        print("max_speed_at_current_direction")
        print("current_speed_without_restriction_abs")
        print(max_speed_at_current_direction)
        print(current_speed_without_restriction_abs)
        if(max_speed_at_current_direction < current_speed_without_restriction_abs):
            print("needed to reduce speed====================================================================================")
            print("max_speed_at_current_direction")
            print("current_speed_without_restriction_abs")
            print(max_speed_at_current_direction)
            print(current_speed_without_restriction_abs)
            multiplier = max_speed_at_current_direction / current_speed_without_restriction_abs
        print("multiplier")
        print(multiplier)
        realSpeedToReturn = x_speed_without_restriction * multiplier, y_speed_without_restriction * multiplier, rotatation_speed_without_restriction * multiplier
        print("realSpeedToReturn")
        print(realSpeedToReturn)
        return x_speed_without_restriction * multiplier, y_speed_without_restriction * multiplier, rotatation_speed_without_restriction * multiplier


    def cycle(self, cycles_remaining: int, robots: typing.Tuple['Robot', ...]):
        self.debuff_timeout_cycles -= min(1, self.debuff_timeout_cycles)
        self.shot_cooldown_cycles -= min(1, self.shot_cooldown_cycles)

        if not cycles_remaining % CYCLES.heat_settlement:
            self.heat, self.hp = ROBOT.settle_heat(self.heat, self.hp)
        if not self.debuff_timeout_cycles:
            self.can_move = True
            self.can_shoot = True
        if not self.hp:
            return

        if any([self.speed.x, self.speed.y, self.rotation_speed]):  # move chassis
            old_center, old_rotation, old_corners = self.center.copy(), self.rotation, self.corners.copy()
            self.center += self.speed.transform(angle=old_rotation)
            self.rotation = (self.rotation + self.rotation_speed) % (2 * math.pi)
            self.corners = [p.transform(self.center, self.rotation) for p in ROBOT.box.corners]

            if self.hits(robots):
                self.rotation_speed *= -MOTION.rebound_coefficient
                self.speed *= -MOTION.rebound_coefficient
                self.center, self.rotation, self.corners = old_center, old_rotation, old_corners
        self.gimbal_yaw = min(max(self.gimbal_yaw + self.gimbal_yaw_speed, -MOTION.gimbal_yaw_range / 2), MOTION.gimbal_yaw_range / 2)
        self.armors = [a.transform(self.center, self.rotation) for a in ROBOT.armors]

    @staticmethod
    def _throttle_response2(speed: float, throttle: float, top_accel: float, top_speed: float, throttleX: float, throttleY: float, throttleR: float, throttleXMax: float, throttleYMax: float, throttleRMax: float ):
        accelerationWithoutRestriction = math.sqrt(math.pow(throttleX,2) + math.pow(throttleY,2) + math.pow(throttleR,2))
        restrictionAtCurrentDirection = get_max_speed_at_current_direction(throttleX, throttleY, throttleR, throttleXMax, throttleYMax, throttleRMax)
        print("no need to reduce acceleration")
        if(accelerationWithoutRestriction > restrictionAtCurrentDirection):
            print("needed to reduce acceleration |||||||||||||||||||||| |||||||||||||||||||||| ||||||||||||||||||||||")
            top_accel=top_accel * (restrictionAtCurrentDirection / accelerationWithoutRestriction)
        new_speed = speed + (throttle - speed / top_speed) * top_accel
        return math.copysign(min(abs(new_speed), top_speed), new_speed)

    @staticmethod
    def _throttle_response(speed: float, throttle: float, top_accel: float, top_speed: float):
        new_speed = speed + (throttle - speed / top_speed) * top_accel
        return math.copysign(min(abs(new_speed), top_speed), new_speed)


from game.geometry import LineSegment, Box
from shared import RobotCommand

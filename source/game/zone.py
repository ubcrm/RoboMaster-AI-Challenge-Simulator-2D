import typing
from game.config import CYCLES, ZONE
from shared import ZoneType


class Zone:
    def __init__(self, index: int):
        self.box = ZONE.boxes[index]
        self.is_activated, self.type_ = None, None

    def reset(self, type_: ZoneType):
        self.is_activated = False
        self.type_ = type_

    def apply(self, activating_robot: 'Robot', robots: typing.Tuple['Robot', ...]):  # Buff/Debuff (Rules 2.3.1)
        self.is_activated = True
        if self.type_ == ZoneType.blue_hp_buff:
            for robot in robots:
                if robot.is_blue:
                    robot.hp += ZONE.hp_buff
        elif self.type_ == ZoneType.red_hp_buff:
            for robot in robots:
                if not robot.is_blue:
                    robot.hp += ZONE.hp_buff
        elif self.type_ == ZoneType.blue_ammo_buff:
            for robot in robots:
                if robot.is_blue:
                    robot.ammo += ZONE.ammo_buff
        elif self.type_ == ZoneType.red_ammo_buff:
            for robot in robots:
                if not robot.is_blue:
                    robot.ammo += ZONE.ammo_buff
        elif self.type_ == ZoneType.move_debuff:
            activating_robot.can_move = False
            activating_robot.debuff_timeout_cycles = CYCLES.debuff_timeout
        elif self.type_ == ZoneType.shoot_debuff:
            activating_robot.can_shoot = False
            activating_robot.debuff_timeout_cycles = CYCLES.debuff_timeout


from game.robot import Robot

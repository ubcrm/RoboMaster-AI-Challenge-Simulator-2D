import random
from game.config import ZONE, TIME

def randomize_zones():
    self._randomize()
    self.activation_status = [False] * 6


class Zone:
    count = 0

    def __init__(self):
        self.rect = ZONE.rects[Zone.count]
        self.type = None
        self.activation_status = None
        Zone.count += 1

    def apply(self, robots):
        for robot in robots:
            if rect.contains(robot.centers) and not activated:
                self._apply_buff_debuff(robot, robots, type_)
                self.activation_status[i] = True

    def _randomize(self):
        random.seed(time.time())
        indices = [0, 2, 4]
        random.shuffle(indices)  # randomly order zones
        self.types = [0] * 6

        for i in range(3):
            side = random.choice([0, 1])  # choose left/right side of field
            self.types[2 * i] = indices[i] + side
            self.types[2 * i + 1] = indices[i] + 1 - side

    @staticmethod
    def _apply_buff_debuff(activating_robot, robots, type_):  # Buff/Debuff (Rules 2.3.1)
        if type_ == ZONE.types['hp_blue']:
            for robot in robots:
                if robot.team:
                    robot.hpBuff += ZONE.hpBuff
        elif type_ == ZONE.types['hp_red']:
            for robot in robots:
                if not robot.team:
                    robot.hpBuff += ZONE.hpBuff
        elif type_ == ZONE.types['ammo_blue']:
            for robot in robots:
                if robot.team:
                    robot.ammoBuff += ZONE.ammoBuff
        elif type_ == ZONE.types['ammo_red']:
            for robot in robots:
                if not robot.team:
                    robot.ammoBuff += ZONE.ammoBuff
        elif type_ == ZONE.types['no_move']:
            activating_robot.canMove = False
            activating_robot.debuffTimeoutCycles = TIME.zone_timeout
        elif type_ == ZONE.types['no_shoot']:
            activating_robot.canShoot = False
            activating_robot.debuffTimeoutCycles = TIME.zone_timeout

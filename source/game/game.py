import time
import random
from game.robot import Robot
from game.zone import Zone
from game.config import CYCLES
from shared import Winner, ZoneType, ZoneState, GameState, TeamState, RobotCommand


class Game:
    def __init__(self):
        self._cycles_remaining = CYCLES.game_duration
        self._blue_robots = Robot(0), Robot(1)
        self._red_robots = Robot(2), Robot(3)
        self._zones = tuple(Zone(i) for i in range(6))
        self._bullets = []
        
    @property
    def _robots(self):
        return *self._blue_robots, *self._red_robots

    def reset(self) -> GameState:
        self.__init__()
        self._update_state()
        return self._state

    def step(self, blue_commands: tuple[RobotCommand], red_commands: tuple[RobotCommand]) -> GameState:
        if not self._cycles_remaining % CYCLES.zone_reset:
            self._randomize_zones()
        for robot, command in zip(self._robots, [*blue_commands, *red_commands]):
            robot.control(command)
        for _ in range(CYCLES.step):
            self._cycle()
        self._update_state()
        return self._state

    def _update_state(self):
        blue_damage_output = sum(r.damage for r in self._red_robots)
        red_damage_output = sum(r.damage for r in self._blue_robots)
        blues_dead = all(r.hp == 0 for r in self._blue_robots)
        reds_dead = all(r.hp == 0 for r in self._red_robots)
        winner = Winner.tbd

        if reds_dead and not blues_dead:
            winner = Winner.blue
        elif blues_dead and not reds_dead:
            winner = Winner.red
        elif self._cycles_remaining <= 0:
            if blue_damage_output > red_damage_output:
                winner = Winner.blue
            elif red_damage_output > blue_damage_output:
                winner = Winner.red
            else:
                winner = Winner.tied
        self._state = GameState(
            blue_state=TeamState(robot_states=tuple(r.state for r in self._blue_robots), damage_output=blue_damage_output),
            red_state=TeamState(robot_states=tuple(r.state for r in self._red_robots), damage_output=red_damage_output),
            time_remaining=self._cycles_remaining / CYCLES.second,
            zone_states=(tuple(ZoneState(type_=z.type_, is_activated=z.is_activated) for z in self._zones)),
            winner=winner)

    def _cycle(self):
        robots = self._robots
        for robot in robots:
            robot.cycle(self._cycles_remaining, robots)
            if (bullet := robot.shoot()) is not None:
                self._bullets.append(bullet)
        for index in reversed(range(len(self._bullets))):
            if self._bullets[index].cycle(self._robots):
                del self._bullets[index]
        for zone in self._zones:
            for robot in robots:
                if zone.box.contains(robot.center) and not zone.is_activated:
                    zone.apply(robot, robots)
        self._cycles_remaining -= 1

    def _randomize_zones(self):
        random.seed(time.time())
        indices = random.sample([0, 2, 4], k=3)  # randomly order zones
        sides = random.choices([0, 1], k=3)  # choose left/right side of field
        types = [ZoneType(i + s) for i, s in zip(indices, sides)] + [ZoneType(i + 1 - s) for i, s in zip(indices, sides)]
        for zone, type_ in zip(self._zones, types):
            zone.reset(type_)

import enum
import dataclasses
import typing
import pathlib
import os

SOURCE_DIR = pathlib.Path(os.path.dirname(__file__))
IMAGE_DIR = SOURCE_DIR / 'assets/images'
FIELD_DIMS = (808, 448)
ROBOT_IDS = {0: (0, 0), 1: (0, 1), 2: (1, 0), 3: (1, 1)}


class Winner(enum.Enum):
    blue = 0
    red = 1
    tied = 2
    tbd = 3  # to be determined - game in progress


class ZoneType(enum.Enum):
    blue_hp_buff = 0
    red_hp_buff = 1
    blue_ammo_buff = 2
    red_ammo_buff = 3
    move_debuff = 4
    shoot_debuff = 5


@dataclasses.dataclass(frozen=True)
class ZoneState:
    type_: ZoneType
    is_activated: bool
    center: typing.Tuple[int, int]


@dataclasses.dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    rotation: float
    gimbal_yaw: float
    ammo: int
    heat: int
    hp: int
    can_move: bool
    can_shoot: bool
    debuff_timeout: float


@dataclasses.dataclass(frozen=True)
class TeamState:
    robot_states: typing.Tuple[RobotState, ...]
    damage_output: int


@dataclasses.dataclass(frozen=True)
class GameState:
    blue_state: TeamState
    red_state: TeamState
    time_remaining: float
    zone_states: typing.Tuple[ZoneState, ...]
    winner: Winner

    @property
    def zone_state_by_type(self):
        return {s.type_: s for s in self.zone_states}


@dataclasses.dataclass(frozen=True)
class RobotCommand:  # throttle values in range [-1, 1]
    x: float = 0.
    y: float = 0.
    rotation: float = 0.
    gimbal_yaw: float = 0.
    shoot: bool = False

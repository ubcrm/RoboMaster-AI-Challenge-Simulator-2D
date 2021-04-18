import enum
import dataclasses
import typing
import pathlib
import os

SOURCE_DIR = pathlib.Path(os.path.dirname(__file__))
IMAGE_DIR = SOURCE_DIR / 'assets/images'
FIELD_DIMS = (808, 448)
ROBOT_COUNT = 2  # per team


class Team(enum.Enum):
    blue = 0
    red = 1


class RobotNumber(enum.Enum):
    zero = 0
    one = 1


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
    _zero_state: RobotState
    _one_state: RobotState
    damage_output: int
    
    @property
    def robot_states(self):
        return {RobotNumber.zero: self._zero_state, RobotNumber.one: self._one_state}


@dataclasses.dataclass(frozen=True)
class GameState:
    _blue_state: TeamState
    _red_state: TeamState
    time_remaining: float
    zone_states: typing.Tuple[ZoneState, ZoneState, ZoneState, ZoneState, ZoneState, ZoneState]
    winner: Winner
    
    @property
    def team_states(self):
        return {Team.blue: self._blue_state, Team.red: self._red_state}


@dataclasses.dataclass(frozen=True)
class RobotCommand:  # throttle values in range [-1, 1]
    x: float = 0.
    y: float = 0.
    rotation: float = 0.
    gimbal_yaw: float = 0.
    shoot: bool = False

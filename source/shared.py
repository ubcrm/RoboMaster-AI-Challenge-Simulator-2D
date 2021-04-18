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
    blueHpBuff = 0
    redHpBuff = 1
    blueAmmoBuff = 2
    redAmmoBuff = 3
    moveDebuff = 4
    shootDebuff = 5


@dataclasses.dataclass(frozen=True)
class ZoneState:
    type_: ZoneType
    isActivated: bool


@dataclasses.dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    rotation: float
    gimbalYaw: float
    ammo: int
    heat: int
    hp: int
    canMove: bool
    canShoot: bool
    debuffTimeout: float


@dataclasses.dataclass(frozen=True)
class TeamState:
    _zeroState: RobotState
    _oneState: RobotState
    damageOutput: int
    
    @property
    def robotStates(self):
        return {RobotNumber.zero: self._zeroState, RobotNumber.one: self._oneState}


@dataclasses.dataclass(frozen=True)
class GameState:
    _blueState: TeamState
    _redState: TeamState
    timeRemaining: float
    zoneStates: typing.Tuple[ZoneState, ZoneState, ZoneState, ZoneState, ZoneState, ZoneState]
    winner: Winner
    
    @property
    def teamStates(self):
        return {Team.blue: self._blueState, Team.red: self._redState}


@dataclasses.dataclass(frozen=True)
class RobotCommand:  # throttle values in range [-1, 1]
    x: float = 0.
    y: float = 0.
    rotation: float = 0.
    gimbalYaw: float = 0.
    shoot: bool = False

from enum import Enum
from dataclasses import dataclass
import pathlib
import os

SOURCE_DIR = pathlib.Path(os.path.dirname(__file__))
IMAGE_DIR = SOURCE_DIR / 'assets/images'
FIELD_DIMS = (808, 448)
ROBOT_COUNT = 2  # robots per team


class Team(Enum):
    blue = 0
    red = 1


class Winner(Enum):
    blue = 0
    red = 1
    tied = 2
    tbd = 3  # to be determined - game in progress


class ZoneType(Enum):
    blueHpBuff = 0
    redHpBuff = 1
    blueAmmoBuff = 2
    redAmmoBuff = 3
    moveDebuff = 4
    shootDebuff = 5


@dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    rotation: float
    gimbalYaw: float

    xSpeed: float
    ySpeed: float
    rotationSpeed: float
    gimbalYawSpeed: float

    ammo: int
    heat: int
    hp: int
    isShooting: bool
    shotCooldown: float

    barrierHits: int
    robotHits: int
    canMove: bool
    canShoot: bool
    debuffTimeout: float


@dataclass(frozen=True)
class TeamState:
    robotStates: tuple[RobotState, RobotState]
    damageOutput: int


@dataclass(frozen=True)
class GameState:
    timeRemaining: float
    blueState: TeamState
    redState: TeamState
    zoneTypes: tuple[ZoneType, ZoneType, ZoneType, ZoneType, ZoneType, ZoneType]
    winner: Winner


@dataclass(frozen=True)
class RobotCommand:  # throttle values in range [-1, 1]
    x: float = 0.
    y: float = 0.
    rotation: float = 0.
    gimbalYaw: float = 0.
    shoot: bool = False

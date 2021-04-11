from enum import Enum
from dataclasses import dataclass
import pathlib
import os

PI = 3.1416
RAD2DEG = 180. / PI
FIELD_DIMS = (808, 448)

SOURCE_DIR = pathlib.Path(os.path.dirname(__file__))
IMAGE_DIR = SOURCE_DIR / 'assets/images'


class Team(Enum):
    blue = 0
    red = 1


class GameMode(Enum):
    oneVsOne = 1
    twoVsTwo = 2


class Winner(Enum):
    blue = 0
    red = 1
    tied = 2
    tbd = 3


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
class GameState:
    timeRemaining: float
    blueRobotsState: tuple[RobotState]
    redRobotsState: tuple[RobotState]
    blueDamageOutput: int
    redDamageOutput: int
    winner: Winner


@dataclass(frozen=True)
class RobotCommand:  # throttle values in range [-1, 1]
    x: float
    y: float
    rotation: float
    gimbalYaw: float
    shoot: bool

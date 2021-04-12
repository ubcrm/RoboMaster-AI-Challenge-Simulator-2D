import pygame
import math
from game.game import Game
from game.robot import Robot
from game.bullet import Bullet
from graphicGame.config import TEXT, COLOR, RENDER
from shared import Team


class GraphicGame(Game):
    def __init__(self):
        super().__init__()
        pygame.init()
        self._screen = pygame.display.set_mode(RENDER.screenDims)
        pygame.display.set_caption(TEXT.title)
        pygame.display.set_icon(RENDER.logo)
        pygame.font.init()
        self._font = pygame.font.SysFont(*TEXT.font)

    def render(self):
        self._screen.blit(*RENDER.background)
        for robot in [*self._blueRobots, *self._redRobots]:
            self._renderRobot(robot)
            self._renderRobotStatus(robot)
        for bullet in self._bullets:
            self._renderBullet(bullet)
        self._renderText(self._state.timeRemaining, TEXT.infoCoords[0])
        self._renderText(self._state.blueState.damageOutput, TEXT.infoCoords[1])
        self._renderText(self._state.redState.damageOutput, TEXT.infoCoords[2])
        self._renderText(self._state.winner.name, TEXT.infoCoords[3])
        pygame.display.flip()

    def _renderRobot(self, robot: Robot):
        if not robot.hp:
            chassisImage = RENDER.deadRobot
        elif robot.team is Team.blue:
            chassisImage = RENDER.blueRobot
        else:
            chassisImage = RENDER.redRobot
        chassisImage = pygame.transform.rotate(chassisImage, math.degrees(robot.rotation))
        gimbalImage = pygame.transform.rotate(RENDER.gimbal, math.degrees(robot.gimbalYaw + robot.rotation))
        chassisRect = chassisImage.get_rect()
        gimbalRect = gimbalImage.get_rect()
        chassisRect.center = gimbalRect.center = robot.center.toTopLeft(offset=RENDER.offset)
        self._screen.blit(chassisImage, chassisRect)
        self._screen.blit(gimbalImage, gimbalRect)
        self._renderText(f'{robot.id_} | {robot.hp}', robot.center.toTopLeft(offset=TEXT.robotLabelOffset),
                         COLOR.blue if (robot.team is Team.blue) else COLOR.red)

    def _renderRobotStatus(self, robot: Robot):
        state = robot.getState()
        rowCoords = TEXT.stateCoords[robot.id_]
        self._renderText(f'{state.x:.0f}', rowCoords[0])
        self._renderText(f'{state.y:.0f}', rowCoords[1])
        self._renderText(f'{math.degrees(state.rotation):.0f}', rowCoords[2])
        self._renderText(f'{math.degrees(state.gimbalYaw):.0f}', rowCoords[3])
        self._renderText(f'{state.xSpeed:.2f}', rowCoords[4])
        self._renderText(f'{state.ySpeed:.2f}', rowCoords[5])
        self._renderText(f'{math.degrees(state.rotationSpeed):.2f}', rowCoords[6])
        self._renderText(f'{math.degrees(state.gimbalYawSpeed):.2f}', rowCoords[7])
        self._renderText(f'{state.ammo}', rowCoords[8])
        self._renderText(f'{state.heat}', rowCoords[9])
        self._renderText(f'{state.hp}', rowCoords[10])
        self._renderText(f'{state.isShooting}', rowCoords[11])
        self._renderText(f'{state.shotCooldown:.3f}', rowCoords[12])
        self._renderText(f'{state.barrierHits}', rowCoords[13])
        self._renderText(f'{state.robotHits}', rowCoords[14])
        self._renderText(f'{state.canMove}', rowCoords[15])
        self._renderText(f'{state.canShoot}', rowCoords[16])
        self._renderText(f'{state.debuffTimeout: .0f}', rowCoords[17])

    def _renderBullet(self, bullet: Bullet):
        bulletRect = RENDER.bullet.get_rect()
        bulletRect.center = bullet.center.toTopLeft(offset=RENDER.offset)
        self._screen.blit(RENDER.bullet, bulletRect)

    def _renderText(self, text, position: tuple[int], color=COLOR.black):
        label = self._font.render(str(text), True, color)
        self._screen.blit(label, position)

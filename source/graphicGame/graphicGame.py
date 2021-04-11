import pygame
from game.game import Game
from game.robot import Robot
from game.bullet import Bullet
from graphicGame.config import TEXT, COLOR, GRAPHICS
from shared import GameMode, RobotCommand, Team, GameState, RAD2DEG


class GraphicGame:
    def __init__(self, mode=GameMode.twoVsTwo):
        self.game = Game(mode)
        pygame.init()
        self.screen = pygame.display.set_mode(GRAPHICS.screenDims)
        pygame.display.set_caption(TEXT.title)
        pygame.display.set_icon(GRAPHICS.logo)
        pygame.font.init()
        self.font = pygame.font.SysFont(*TEXT.font)

    def advance(self, blueCommands: tuple[RobotCommand], redCommands: tuple[RobotCommand]):
        state = self.game.advance(blueCommands, redCommands)
        self._display(state)
        return state

    def _display(self, state: GameState):
        self.screen.blit(*GRAPHICS.background)
        for robot in [*self.game.blueRobots, *self.game.redRobots]:
            self._displayRobot(robot)
            self._displayRobotStatus(robot)
        for bullet in self.game.bullets:
            self._displayBullet(bullet)
        self._displayText(state.timeRemaining, TEXT.infoCoords[0])
        self._displayText(state.blueDamageOutput, TEXT.infoCoords[1])
        self._displayText(state.redDamageOutput, TEXT.infoCoords[2])
        self._displayText(state.winner.name, TEXT.infoCoords[3])
        pygame.display.flip()

    def _displayRobot(self, robot: Robot):
        if not robot.hp:
            chassisImage = GRAPHICS.deadRobot
        elif robot.team is Team.blue:
            chassisImage = GRAPHICS.blueRobot
        else:
            chassisImage = GRAPHICS.redRobot
        chassisImage = pygame.transform.rotate(chassisImage, robot.rotation * RAD2DEG)
        gimbalImage = pygame.transform.rotate(GRAPHICS.gimbal, (robot.gimbalYaw + robot.rotation) * RAD2DEG)
        chassisRect = chassisImage.get_rect()
        gimbalRect = gimbalImage.get_rect()
        chassisRect.center = gimbalRect.center = robot.center.toTopLeft(offset=GRAPHICS.offset)
        self.screen.blit(chassisImage, chassisRect)
        self.screen.blit(gimbalImage, gimbalRect)
        self._displayText(f'{robot.id_} | {robot.hp}', robot.center.toTopLeft(offset=TEXT.robotLabelOffset),
                          COLOR.blue if (robot.team is Team.blue) else COLOR.red)

    def _displayRobotStatus(self, robot: Robot):
        state = robot.getState()
        self._displayText(f'{state.x:.0f}', TEXT.stateCoords[robot.id_][0])
        self._displayText(f'{state.y:.0f}', TEXT.stateCoords[robot.id_][1])
        self._displayText(f'{state.rotation * RAD2DEG:.0f}', TEXT.stateCoords[robot.id_][2])
        self._displayText(f'{state.gimbalYaw * RAD2DEG:.0f}', TEXT.stateCoords[robot.id_][3])
        self._displayText(f'{state.xSpeed:.2f}', TEXT.stateCoords[robot.id_][4])
        self._displayText(f'{state.ySpeed:.2f}', TEXT.stateCoords[robot.id_][5])
        self._displayText(f'{state.rotationSpeed * RAD2DEG:.2f}', TEXT.stateCoords[robot.id_][6])
        self._displayText(f'{state.gimbalYawSpeed * RAD2DEG:.2f}', TEXT.stateCoords[robot.id_][7])
        self._displayText(f'{state.ammo}', TEXT.stateCoords[robot.id_][8])
        self._displayText(f'{state.heat}', TEXT.stateCoords[robot.id_][9])
        self._displayText(f'{state.hp}', TEXT.stateCoords[robot.id_][10])
        self._displayText(f'{state.isShooting}', TEXT.stateCoords[robot.id_][11])
        self._displayText(f'{state.shotCooldown:.3f}', TEXT.stateCoords[robot.id_][12])
        self._displayText(f'{state.barrierHits}', TEXT.stateCoords[robot.id_][13])
        self._displayText(f'{state.robotHits}', TEXT.stateCoords[robot.id_][14])
        self._displayText(f'{state.canMove}', TEXT.stateCoords[robot.id_][15])
        self._displayText(f'{state.canShoot}', TEXT.stateCoords[robot.id_][16])
        self._displayText(f'{state.debuffTimeout: .0f}', TEXT.stateCoords[robot.id_][17])

    def _displayBullet(self, bullet: Bullet):
        bulletRect = GRAPHICS.bullet.get_rect()
        bulletRect.center = bullet.center.toTopLeft(offset=GRAPHICS.offset)
        self.screen.blit(GRAPHICS.bullet, bulletRect)

    def _displayText(self, text, position: tuple[int], color=COLOR.black):
        label = self.font.render(str(text), True, color)
        self.screen.blit(label, position)

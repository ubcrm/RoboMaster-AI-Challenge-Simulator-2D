import pygame
from graphicGame.graphicGame import GraphicGame
from shared import RobotCommand
from interactiveGame.config import DELAY, SHORT_DELAY, GUIDE_RENDER


class InteractiveGame:
    def __init__(self):
        self._game = GraphicGame()
        self._selectedId = None
        self._speedUp = False
        self._showGuide = False

    def reset(self):
        self._game.reset()
        self._selectedId = 0
        while (commands := self._receiveCommands()) is not None:
            self._game.step(*commands)
            self._game.render()
            if self._showGuide:  # this is bad and violates LoD, what to do instead?
                self._game._screen.blit(*GUIDE_RENDER)
                pygame.display.flip()
            pygame.time.wait(SHORT_DELAY if self._speedUp else DELAY)

    def _receiveCommands(self):
        pressed = pygame.key.get_pressed()
        for event in pygame.event.get():
            if (event.type == pygame.QUIT) or pressed[pygame.K_ESCAPE]:
                return None
        self._speedUp = pressed[pygame.K_LSHIFT]
        self._showGuide = pressed[pygame.K_TAB]

        if pressed[pygame.K_BACKQUOTE]: self._selectedId = 0
        elif pressed[pygame.K_1]: self._selectedId = 1
        elif pressed[pygame.K_2]: self._selectedId = 2
        elif pressed[pygame.K_3]: self._selectedId = 3

        commands = [[RobotCommand() for _ in range(2)] for _ in range(2)]
        commands[self._selectedId // 2][self._selectedId % 2] = RobotCommand(
            x=pressed[pygame.K_w] - pressed[pygame.K_s],
            y=pressed[pygame.K_q] - pressed[pygame.K_e],
            rotation=pressed[pygame.K_a] - pressed[pygame.K_d],
            gimbalYaw=pressed[pygame.K_j] - pressed[pygame.K_l],
            shoot=pressed[pygame.K_k])
        return commands

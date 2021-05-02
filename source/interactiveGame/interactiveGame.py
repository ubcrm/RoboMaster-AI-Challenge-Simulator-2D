import pygame
from graphicGame.graphicGame import GraphicGame
from shared import RobotCommand
from interactiveGame.config import DELAY, SHORT_DELAY, GUIDE_RENDER


class InteractiveGame:
    def __init__(self):
        self._game = GraphicGame()
        self._selected_id = 0
        self._speed_up = False
        self._view_guide = False
        self._run()

    def _run(self):
        while (commands := self._receive_commands()) is not None:
            self._game.step(*commands)
            if self._view_guide:
                self._game._screen.blit(*GUIDE_RENDER)  # this is not good
            self._game._blit()
            pygame.display.flip()
            pygame.time.wait(SHORT_DELAY if self._speed_up else DELAY)

    def _receive_commands(self):
        pressed = pygame.key.get_pressed()
        for event in pygame.event.get():
            if (event.type == pygame.QUIT) or pressed[pygame.K_ESCAPE]:
                return
        self._speed_up = pressed[pygame.K_LSHIFT]
        self._view_guide = pressed[pygame.K_TAB]

        if pressed[pygame.K_BACKQUOTE]:
            self._selected_id = 0
        elif pressed[pygame.K_1]:
            self._selected_id = 1
        elif pressed[pygame.K_2]:
            self._selected_id = 2
        elif pressed[pygame.K_3]:
            self._selected_id = 3

        commands = [RobotCommand() for _ in range(4)]
        commands[self._selected_id] = RobotCommand(
            x=pressed[pygame.K_w] - pressed[pygame.K_s],
            y=pressed[pygame.K_q] - pressed[pygame.K_e],
            rotation=pressed[pygame.K_a] - pressed[pygame.K_d],
            gimbal_yaw=pressed[pygame.K_j] - pressed[pygame.K_l],
            shoot=pressed[pygame.K_k])

        return commands[0:2], commands[2:4]

import pygame
from graphicGame.graphicGame import GraphicGame
from shared import GameMode, RobotCommand

DELAY = 60


class InteractiveGame:
    def __init__(self, mode=GameMode.twoVsTwo):
        self.mode = mode
        self.selectedId = 0
        self.game = GraphicGame(mode)
        self._run()

    def _run(self):
        while (commands := self._receiveCommands()) is not None:
            self.game.advance(*commands)
            pygame.time.wait(DELAY)

    def _receiveCommands(self):
        pressed = pygame.key.get_pressed()
        for event in pygame.event.get():
            if (event.type == pygame.QUIT) or pressed[pygame.K_ESCAPE]:
                return None

        if pressed[pygame.K_1]: self.selectedId = 0
        elif pressed[pygame.K_2]: self.selectedId = 1
        elif pressed[pygame.K_3]: self.selectedId = 2
        elif pressed[pygame.K_4]: self.selectedId = 3
        self.selectedId = min(self.selectedId, 2 * self.mode.value - 1)

        commands = [[RobotCommand(x=0., y=0., rotation=0., gimbalYaw=0., shoot=False) for _ in range(self.mode.value)] for _ in range(2)]
        commands[self.selectedId % 2][self.selectedId // 2] = RobotCommand(
            x=pressed[pygame.K_w] - pressed[pygame.K_s],
            y=pressed[pygame.K_q] - pressed[pygame.K_e],
            rotation=pressed[pygame.K_a] - pressed[pygame.K_d],
            gimbalYaw=pressed[pygame.K_b] - pressed[pygame.K_m],
            shoot=pressed[pygame.K_SPACE])
        return commands

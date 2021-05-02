import os

from game.actor import Actor
from shared import RobotCommand

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
from game.game import Game
from graphicGame.graphicGame import GraphicGame
from interactiveGame.interactiveGame import InteractiveGame


if __name__ == '__main__':
    game = GraphicGame()
    game_state = game.reset()
    blue_0 = Actor(0, game_state)
    red_0 = Actor(1, game_state)
    blue_1 = Actor(2, game_state)
    red_1 = Actor(3, game_state)
    command_blue_0 = RobotCommand()
    command_red_0 = RobotCommand()
    command_blue_1 = RobotCommand()
    command_red_1 = RobotCommand()

    for _ in range(1000):
        game_state = game.step((command_blue_0, command_blue_1), (command_red_0, command_red_1))
        action_blue_0 = blue_0.commands_from_state(game_state)
        action_red_0 = red_0.commands_from_state(game_state)
        action_blue_1 = blue_1.commands_from_state(game_state)
        action_red_1 = red_1.commands_from_state(game_state)

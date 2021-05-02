import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
from game.game import Game
from graphicGame.graphicGame import GraphicGame
from interactiveGame.interactiveGame import InteractiveGame


if __name__ == '__main__':
    InteractiveGame()

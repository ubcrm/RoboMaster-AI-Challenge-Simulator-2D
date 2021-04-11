import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
from source.graphicGame.graphicGame import GraphicGame
from source.interactiveGame.interactiveGame import InteractiveGame
from shared import GameMode

if __name__ == '__main__':
    InteractiveGame(mode=GameMode.twoVsTwo)

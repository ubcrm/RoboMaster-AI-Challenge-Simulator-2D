import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
from source.graphicGame.graphicGame import GraphicGame
from source.interactiveGame.interactiveGame import InteractiveGame


if __name__ == '__main__':
    iGame = InteractiveGame()
    iGame.reset()

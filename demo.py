import time
import numpy as np
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

from modules.rmaics import Rmaics
from modules.actor import Actor

'''
if __name__ == '__main__':
    start_time = time.time()
    game = Rmaics(agent_num=4, render=True)
    game.play()

    print(f'Execution time: {int(time.time() - start_time)}s')
'''

game = Rmaics(agent_num=1, render=True)
# game.reset()

actor0 = Actor(1, game.game.robots[0])
# actor1 = Actor(2, game.game.robots[1])
# actor2 = Actor(3, game.game.robots[2])
# actor3 = Actor(4, game.game.robots[3])

action0 = [1, 0, 0, 0, 0]
# action1 = [0, 1, 0, 0, 0]
# action2 = [0, 0, 1, 0, 0]
# action3 = [0, 0, 0, 1, 0]

for _ in range(1000):
    #obs, reward, done, info = game.step(np.array([action0,action1,action2,action3]))
    obs, reward, done, info = game.step(np.array([action0]))
    action0 = actor0.commands_from_state(obs)
    # action1 = actor1.commands_from_state(obs)
    # action2 = actor2.commands_from_state(obs)
    # action3 = actor3.commands_from_state(obs)
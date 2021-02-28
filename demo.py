from modules.rmaics import Rmaics
from modules.actor import Actor
import numpy as np

# if __name__ == '__main__':
#     game = Rmaics(agent_num=4, render=True)
#     game.play()

# game.save_record('./records/record0.npy')
# player = record_player()
# player.play('./records/record_test.npy')

#%% This cell runs the game using rule-based agents
#TODO: Update this to work with new changes

game = Rmaics(agent_num=4, render=True)
# game.reset()

actor0 = Actor(1, game.game.robots[0])
actor1 = Actor(2, game.game.robots[1])
actor2 = Actor(3, game.game.robots[2])
actor3 = Actor(4, game.game.robots[3])

action0 = [1, 0, 0, 0, 0]
action1 = [0, 1, 0, 0, 0]
action2 = [0, 0, 1, 0, 0]
action3 = [0, 0, 0, 1, 0]

for _ in range(1000):
    obs, reward, done, info = game.step(np.array([action0,action1,action2,action3]))
    action0 = actor0.commands_from_state(obs)
    action1 = actor1.commands_from_state(obs)
    action2 = actor2.commands_from_state(obs)
    action3 = actor3.commands_from_state(obs)
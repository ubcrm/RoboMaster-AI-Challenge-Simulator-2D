import gym
from gym import error, spaces
from gym import utils
from gym.utils import seeding
import logging

from modules.rmaics import Rmaics

from modules.rmaics import Rmaics
from modules.actor import Actor
import numpy as np

logger = logging.getLogger(__name__)


class RMAICSEnv(gym.Env, utils.ezpickle):

    def __init__(self, num_agents=1):
        self.rmaics = Rmaics(agent_num=num_agents, render=False)
        self.game = self.rmaics.game

        # observation space for now is just x, y of each agent
        self.observation_space = spaces.Discrete(2 * num_agents)
        # Currently just movement: [x_dir, y_dir]
        self.action_space = spaces.Discrete(2)
        self.iter = 0

    def step(self, action):
        self._take_action(action)
        self.game.one_epoch()
        reward = self.rmaics.get_reward()

        self.iter += 1

    def _take_action(self, action):
        self.game.set_actions([action])


    def reset(self):
        pass

    def render(self, mode='human'):
        pass

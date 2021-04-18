from typing import List

import gym
from gym import error, spaces
from gym import utils
import logging

from modules.constants import FIELD, IMAGE
from modules.rmaics import Rmaics

logger = logging.getLogger(__name__)


class RMAICSEnv(gym.Env):

    def __init__(self, num_agents=1, num_steps=400):
        self.rmaics = Rmaics(agent_num=num_agents, render=False)

        # observation space for now is just x, y of each agent
        self.observation_space = spaces.Discrete(2 * num_agents)
        # Currently just movement: [up, down, left, right]
        self.action_space = spaces.Discrete(4)
        self.iter = 0
        self.num_steps = num_steps
        self.rmaics.game.init_screen()

    @property
    def game(self):
        return self.rmaics.game

    def step(self, action: List[float]):
        self._take_action(action)
        self.game.one_epoch()
        reward = self.rmaics.get_reward()
        ob = self.rmaics.get_observation()
        episode_over = self.iter > self.num_steps
        self.iter += 1
        return ob, reward, episode_over, {}

    def _take_action(self, action: List[float]):
        y = action[0] - action[1]  # up - down = y movement
        x = action[3] - action[2]  # right - left = x movement

        self.game.set_actions([x, y, 0, 0, 0])

    def reset(self):
        reset_state = self.rmaics.reset()
        self.iter = 0
        return reset_state

    def render(self, mode="human"):
        self.game.draw()

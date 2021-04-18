from typing import List

import gym
from gym import error, spaces
from gym import utils
import numpy as np
import logging

from modules.constants import FIELD
from modules.geometry import to_draw_coords
from modules.objects import State
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

    @staticmethod
    def _parse_observation(observation: State):
        robot_state_dict = observation.robots_status[0]
        field_dims = FIELD.dims
        robot_coords = to_draw_coords([robot_state_dict["x_center"], robot_state_dict["y_center"]])
        robot_coords = [robot_coords[0] / field_dims[0], robot_coords[1] / field_dims[1]]
        return np.array(robot_coords)

    def step(self, action: np.array):
        self._take_action(action)
        self.game.one_epoch()
        reward = self.rmaics.get_reward()
        ob = self._parse_observation(self.rmaics.get_observation())
        episode_over = self.iter > self.num_steps
        self.iter += 1

        return ob, reward, episode_over, {}

    def _take_action(self, np_action: np.array):
        action = list(np_action)
        x = action[0]
        y = action[1]

        self.game.set_actions([x, y, 0, 0, 0])

    def reset(self):
        reset_state = self.rmaics.reset()
        self.iter = 0
        return self._parse_observation(reset_state)

    def render(self, mode="human"):
        self.game.draw()

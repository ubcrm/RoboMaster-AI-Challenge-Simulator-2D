from modules.kernel import Kernel


class Rmaics(object):
    def __init__(self, agent_num, render=False):
        self.game = Kernel(robot_count=agent_num, render=render)
        self.memory = []
        self.state = None

    def reset(self):
        self.state = self.game.reset()
        self.obs = self.get_observation(self.state)
        return self.obs

    def step(self, commands):
        state = self.game.step(commands)
        obs = self.get_observation(state)
        rewards = self.get_reward(state)

        self.memory.append([obs, commands, rewards])
        self.state = state
        return obs, rewards, False, None
    
    def get_observation(self, state):
        # personalize your observation here
        return state
    
    def get_reward(self, state):
        # personalize your reward here
        rewards = None
        return rewards

    def play(self):
        self.game.play()

    def save_record(self, file):
        self.game.save_record(file)

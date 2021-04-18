from modules.kernel import Kernel


class Rmaics(object):
    def __init__(self, agent_num, render=False):
        self.game = Kernel(robot_count=agent_num, render=render)
        self.memory = []

    @property
    def state(self):
        return self.game.state

    def reset(self):
        self.game.reset()
        self.memory = []
        return self.get_observation()

    def step(self, commands):
        state = self.game.step(commands)
        obs = self.get_observation(state)
        rewards = self.get_reward(state)

        self.memory.append([obs, commands, rewards])
        return obs, rewards, False, None
    
    def get_observation(self):
        # personalize your observation here
        return self.state
    
    def get_reward(self):
        # personalize your reward here
        return 0

    def play(self):
        self.game.play()

    def save_record(self, file):
        self.game.save_record(file)

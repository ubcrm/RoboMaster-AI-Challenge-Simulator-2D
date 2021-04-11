from modules.kernel import Kernel


class Rmaics(object):
    def __init__(self, agent_num, render=True):
        self.game = Kernel(robot_count=agent_num, render=render)
        self.memory = []

    @property
    def state(self):
        return self.game.state

    def reset(self):
        self.game.reset()
        self.memory = []
        return self.get_observation()

    def step(self, actions):
        state = self.game.step(actions)
        obs = self.get_observation()
        reward = self.get_reward()

        self.memory.append([obs, actions, reward])
        return obs, reward, state.done, None
    
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

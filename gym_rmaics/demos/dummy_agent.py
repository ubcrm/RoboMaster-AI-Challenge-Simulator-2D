from gym_rmaics.gym_rmaics.envs import RMAICSEnv

if __name__ == '__main__':
    env = RMAICSEnv()
    obs = env.reset()
    for i in range(2000):
        action = [1, 0, 0, 0]
        obs, rewards, done, info = env.step(action)
        env.render()
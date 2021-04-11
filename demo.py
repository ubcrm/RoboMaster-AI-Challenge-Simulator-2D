from gym_rmaics.gym_rmaics.envs import RMAICSEnv

from modules.rmaics import Rmaics
from modules.actor import Actor
import numpy as np
import gym
import subprocess

from garage import wrap_experiment
from garage.envs import GymEnv, normalize
from garage.experiment import deterministic
from garage.sampler import LocalSampler
from garage.torch.algos import TRPO as PyTorch_TRPO
from garage.torch.policies import GaussianMLPPolicy as PyTorch_GMP
from garage.torch.value_functions import GaussianMLPValueFunction
from garage.trainer import Trainer

hyper_parameters = {
    'hidden_sizes': [32, 32],
    'max_kl': 0.01,
    'gae_lambda': 0.97,
    'discount': 0.99,
    'n_epochs': 999,
    'batch_size': 1024,
}


@wrap_experiment
def trpo_garage_pytorch(ctxt, env_id, seed):
    """Create garage PyTorch TRPO model and training.
    Args:
        ctxt (garage.experiment.ExperimentContext): The experiment
                configuration used by Trainer to create the
                snapshotter.
        env_id (str): Environment id of the task.
        seed (int): Random positive integer for the trial.
    """
    deterministic.set_seed(seed)

    trainer = Trainer(ctxt)

    env = normalize(GymEnv(env_id))

    policy = PyTorch_GMP(env.spec,
                         hidden_sizes=hyper_parameters['hidden_sizes'],
                         hidden_nonlinearity=torch.tanh,
                         output_nonlinearity=None)

    value_function = GaussianMLPValueFunction(env_spec=env.spec,
                                              hidden_sizes=(32, 32),
                                              hidden_nonlinearity=torch.tanh,
                                              output_nonlinearity=None)

    sampler = LocalSampler(agents=policy,
                           envs=env,
                           max_episode_length=env.spec.max_episode_length)

    algo = PyTorch_TRPO(env_spec=env.spec,
                        policy=policy,
                        value_function=value_function,
                        sampler=sampler,
                        discount=hyper_parameters['discount'],
                        gae_lambda=hyper_parameters['gae_lambda'])

    trainer.setup(algo, env)
    trainer.train(n_epochs=hyper_parameters['n_epochs'],
                  batch_size=hyper_parameters['batch_size'])



if __name__ == '__main__':
    env = RMAICSEnv()
    trpo_garage_pytorch()
    # game = Rmaics(agent_num=4, render=True)
    # game.play()

# game.save_record('./records/record0.npy')
# player = record_player()
# player.play('./records/record_test.npy')

# %% This cell runs the game using rule-based agents
# TODO: Update this to work with new changes

# game = Rmaics(agent_num=4, render=True)
# game.reset()

# actor0 = Actor(0)
# actor1 = Actor(1)
# actor2 = Actor(2)
# actor3 = Actor(3)

# action0 = [1, 0, 0, 0, 0, 0, 0, 0]
# action1 = [1, 0, 0, 0, 0, 0, 0, 0]
# action2 = [1, 0, 0, 0, 0, 0, 0, 0]
# action3 = [1, 0, 0, 0, 0, 0, 0, 0]

# for _ in range(1000):
#     obs, reward, done, info = game.step(np.array([action0,action1,action2,action3]))
#     action0 = actor0.action_from_state(obs, game.g_map)
#     action1 = actor1.action_from_state(obs, game.g_map)
#     action2 = actor2.action_from_state(obs, game.g_map)
#     action3 = actor3.action_from_state(obs, game.g_map)

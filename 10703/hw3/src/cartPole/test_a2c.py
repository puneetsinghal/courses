import sys
import os
import argparse
import numpy as np
import tensorflow as tf
import keras
from keras.models import Sequential, load_model, Model
from keras.layers import Activation, Dropout, Input, Add
from keras.layers.core import Dense, Lambda
from keras import backend as K
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint
import gym
import pickle

from IPython import embed

import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt

from reinforce import Reinforce
try:
	from keras.callbacks import TensorBoard
except Exception as e:
	print("{} from keras.callback. This will prevent gathering data on tensorboard".format(e))

os.environ["CUDA_VISIBLE_DEVICES"]="0"
config = tf.ConfigProto()
config.log_device_placement=False
config.allow_soft_placement = True
config.gpu_options.allow_growth=True
config.gpu_options.per_process_gpu_memory_fraction = 1
SESS = tf.Session(config=config)
K.set_session(SESS)

class A2C(Reinforce):
	# Implementation of N-step Advantage Actor Critic.
	# This class inherits the Reinforce class, so for example, you can reuse
	# generate_episode() here.

	def __init__(self, model, lr, critic_model, critic_lr, n, numStates, numActions):
		# Initializes A2C.
		# Args:
		# - model: The actor model.
		# - lr: Learning rate for the actor model.
		# - critic_model: The critic model.
		# - critic_lr: Learning rate for the critic model.
		# - n: The value of N in N-step A2C.
		self.model = model
		self.critic_model = critic_model
		self.n = n

def parse_arguments():
	# Command-line flags are defined here.
	parser = argparse.ArgumentParser()
	parser.add_argument('--model-config-path', dest='model_config_path',
						type=str, default='LunarLander-v2-config.json',
						help="Path to the actor model config file.")
	parser.add_argument('--num-episodes', dest='num_episodes', type=int,
						default=50000, help="Number of episodes to train on.")
	parser.add_argument('--lr', dest='lr', type=float,
						default=5e-4, help="The actor's learning rate.")
	parser.add_argument('--critic-lr', dest='critic_lr', type=float,
						default=1e-4, help="The critic's learning rate.")
	parser.add_argument('--n', dest='n', type=int,
						default=20, help="The value of N in N-step A2C.")

	# https://stackoverflow.com/questions/15008758/parsing-boolean-values-with-argparse
	parser_group = parser.add_mutually_exclusive_group(required=False)
	parser_group.add_argument('--render', dest='render',
							  action='store_true',
							  help="Whether to render the environment.")
	parser_group.add_argument('--no-render', dest='render',
							  action='store_false',
							  help="Whether to render the environment.")
	parser.set_defaults(render=False)

	return parser.parse_args()

def createModel(hiddenUnits, numStates, numActions):
	critic_model = Sequential()
	critic_model.add(Dense(hiddenUnits, input_dim=numStates, activation="relu"))
	critic_model.add(Dense(hiddenUnits, activation="relu"))
	critic_model.add(Dense(hiddenUnits, activation="relu"))
	critic_model.add(Dense(1))
	print("Critic model initialized")

	actor_model = Sequential()
	actor_model.add(Dense(16, input_dim=numStates, activation="relu"))
	actor_model.add(Dense(16, activation="relu"))
	actor_model.add(Dense(16, activation="relu"))
	actor_model.add(Dense(numActions, activation="softmax"))
	print("Actor model initialized")

	return critic_model, actor_model

def main(args):
	# Parse command-line arguments.
	args = parse_arguments()
	model_config_path = args.model_config_path
	num_episodes = args.num_episodes
	lr = args.lr
	critic_lr = args.critic_lr
	n = args.n
	render = args.render

	# Hyperparameters
	num_episodes, lr, critic_lr, n, hiddenUnits = [50000, 1e-4, 1e-4, 20, 16]

	# Create the environment.
	# env = gym.make('LunarLander-v2')
	env = gym.make('CartPole-v0')
	numStates = env.observation_space.shape[0]
	numActions = env.action_space.n

	# Load the actor model from file.
	N = [20]#[1, 20, 50, 100]
	samplingFreq = 500
	EPISODES = np.linspace(500,50000,100, True, dtype=np.int)
	REWARD = np.zeros((len(N), EPISODES.size, 100))
	MEAN = np.zeros((len(N), EPISODES.size))
	STD_VAR = np.zeros((len(N), EPISODES.size))

	# embed()
	for trial in range(len(N)):
		n = N[trial]
		fileName = './results_' + str(n)
		critic_model, actor_model = createModel(hiddenUnits, numStates, numActions)
		a2c = A2C(actor_model, lr, critic_model, critic_lr, n, numStates, numActions)
		
		for test_ep_index in range(EPISODES.size):
			actorFileName = './model/' + str(n) + '/actor-' + str(EPISODES[test_ep_index]) + '.hdf5'
			a2c.model.load_weights(actorFileName)
			for ep in range(100):
				s, a, r = a2c.generate_episode(env, render)
				REWARD[trial, test_ep_index, ep] = sum(r)

			MEAN[trial, test_ep_index] = np.mean(REWARD[trial, test_ep_index,:])
			STD_VAR[trial, test_ep_index] = np.std(REWARD[trial, test_ep_index,:])
			print("test_ep_index #: {}, MEAN: {}, STD_VAR: {}".format(EPISODES[test_ep_index], MEAN[trial, test_ep_index], STD_VAR[trial, test_ep_index]))
		pickle.dump([REWARD, MEAN, STD_VAR], open(fileName, 'wb'))
	
	# fileName = './results_' + str(n)
	# REWARD, MEAN, STD_VAR = pickle.load(open(fileName, 'rb'))
	# print(EPISODES.shape, MEAN[0].shape, STD_VAR[0].shape)
	plt.figure()
	plt.errorbar(EPISODES, MEAN[0], STD_VAR[0])
	plt.show()

if __name__ == '__main__':
	main(sys.argv)

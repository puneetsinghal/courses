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
import math

from IPython import embed

import matplotlib
matplotlib.use('Agg')
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

class Logger(object):
    """Logging in tensorboard without tensorflow ops."""

    def __init__(self, log_dir):
        """Creates a summary writer logging to log_dir."""
        self.writer = tf.summary.FileWriter(log_dir)

    def log_scalar(self, tag, value, step):
        """Log a scalar variable.
        Parameter
        ----------
        tag : basestring
            Name of the scalar
        value
        step : int
            training iteration
        """
        summary = tf.Summary(value=[tf.Summary.Value(tag=tag,
                                                     simple_value=value)])
        self.writer.add_summary(summary, step)

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
		
		# TODO: Define any training operations and optimizers here, initialize
		#       your variables, or alternately compile your model here.
		self.actions = tf.placeholder(shape=[None, 1], dtype=tf.int32)
		self.X = tf.placeholder(tf.float32, shape=(None, numStates))
		self.Y = tf.placeholder(tf.float32, shape=(None, 1))
		# self.A = tf.placeholder(tf.float32, shape=(None, numActions))
		self.G = tf.placeholder(tf.float32, shape=(None, 1))
		self.A = tf.one_hot(self.actions, numActions, dtype=tf.float32)
		self.criticOutputPlaceHolder = tf.placeholder(tf.float32, shape=(None, 1))

		self.criticOutput = self.critic_model.output
		self.criticLoss = tf.reduce_mean(tf.square(self.G - self.criticOutput))
		# self.criticOptimizer = tf.train.AdamOptimizer(critic_lr).minimize(self.criticLoss)
		self.criticTrainer = tf.train.AdamOptimizer(critic_lr)
		self.criticOptimizer = self.criticTrainer.minimize(self.criticLoss)
		
		# (self.critic_model).compile(loss="mse", optimizer=Adam(lr=critic_lr))
		
		self.actorOutput =self.model.output
		# self.grad = tf.gradients()
		# print(self.model.predict(self.X))
		# self.logLoss = tf.log(tf.reduce_sum(tf.multiply(self.actorOutput, self.A), 1))
		self.logLoss = tf.log(tf.clip_by_value(tf.reduce_sum(tf.multiply(self.actorOutput, self.A), [1]), 1e-25, 1.0))
		self.policyLoss = -tf.reduce_mean(tf.multiply((self.G - self.criticOutputPlaceHolder), self.logLoss))
		# self.actorOptimizer = tf.train.GradientDescentOptimizer(0.1).minimize(self.policyLoss)
		# self.actorOptimizer = tf.train.AdamOptimizer(lr).minimize(self.policyLoss)
		self.actorTrainer = tf.train.AdamOptimizer(lr)
		self.actorOptimizer = self.actorTrainer.minimize(self.policyLoss)

		# checkpoint_weights_filename = './model/critic-' + '{epoch:02d}-{loss:.4f}.h5f'
		# self.callbacks_list = [ModelCheckpoint(checkpoint_weights_filename, monitor='loss', verbose=1, period=100)]
		# log_filename = 'dqn_{}_log.json'.format(args.env_name)
		# callbacks += [FileLogger(log_filename, interval=100)]
		# self.callbacks_list += [TensorBoard('./train_log')]

		# Initialize all variables
		init_op = tf.global_variables_initializer()
		SESS.run(init_op)

	def train(self, env, batchSize, gamma=1.0, render=False, update_actor = True):
		# Trains the model on a single episode using A2C.
		# TODO: Implement this method. It may be helpful to call the class
		#       method generate_episode() to generate training data.
		statesCummulative = []
		actionsCummulative = []
		rewardsCummulative = []
		TCummulative = []
		sumRewardCummulative = []
		numStates = env.observation_space.shape[0]
		numActions = env.action_space.n
		# batchSize = 4
		for i in range(batchSize):
			# embed()
			states, actions, rewards = self.generate_episode(env, render)
			T = len(states)
			statesCummulative += (states)
			actionsCummulative += (actions)
			rewardsCummulative += (rewards)
			TCummulative += ([T])
			# self.actions = np.array(actions).reshape(T,1)
			# sumRewardEpisode = []

			# actorOptimizer = tf.train.GradientDescentOptimizer(0.5).minimize(lossPolicy)
			for t in range(T-1, -1, -1):
				if (t+self.n >= T):
					V_end = 0
				else:
					V_end = self.critic_model.predict(np.array(states[t+self.n]).reshape(1,numStates))

				sumReward = (gamma**self.n)*V_end
				for k in range(self.n):
					if(k+t<T):
						sumReward += 1e-2*(gamma**k) * rewards[t+k]
					# else we need to add 0, so skipping that component
				sumRewardCummulative.append(sumReward)
			# sumRewardCummulative.append(sumRewardEpisode)

		totalT = sum(TCummulative)
		sumRewardCummulative = np.array(sumRewardCummulative).reshape(sum(TCummulative),1)
		# sumRewardCummulative -= np.mean(sumRewardCummulative, 0)*np.ones((T,1))
		# sumRewardCummulative /= np.std(sumRewardCummulative, 0)
		# A = np.zeros((totalT, numActions))
		# for i in range(totalT):
		# 	A[i,actionsCummulative[i]] = 1
		# embed()
		# print(np.array(statesCummulative).shape)
		critic_feed_dict = {self.critic_model.input:np.array(statesCummulative).reshape(totalT,numStates), 
							self.G:np.array(sumRewardCummulative).reshape(sum(TCummulative),1)}

		# criticOutput = self.critic_model.predict(np.array(statesCummulative).reshape(totalT,numStates))
		criticOutput, criticLoss, _ = SESS.run([self.critic_model.output, self.criticLoss, self.criticOptimizer], feed_dict= critic_feed_dict)
		print(criticOutput.shape)
		if(update_actor):
			actor_feed_dict = {self.actions:np.array(actionsCummulative).reshape(totalT,1), 
								self.model.input:np.array(statesCummulative).reshape(totalT,numStates), 
								self.G:sumRewardCummulative, 
								self.criticOutputPlaceHolder:criticOutput}
			_, actorLoss,_ = SESS.run([self.A, self.policyLoss, self.actorOptimizer], feed_dict=actor_feed_dict)
		else:
			actorLoss = None
		
		return actorLoss, criticLoss, sum(rewardsCummulative)/batchSize


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
	critic_model.add(Dense(1, activation="linear"))
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
	batchSize, gamma, criticVsActor = [1, 1., 1]

	# Hyperparameters
	# num_episodes, lr, critic_lr, n, hiddenUnits, batchSize, gamma, criticVsActor = [50000, 5e-4, 1e-4, 100, 16, 1, 0.99, 1] # working parameters for N = 100
	# num_episodes, lr, critic_lr, n, hiddenUnits, batchSize, gamma, criticVsActor = [50000, 5e-7, 1e-7, 100, 16, 1, 1.0, 1] # working parameters for N = 100. contd
	# num_episodes, lr, critic_lr, n, hiddenUnits, batchSize, gamma, criticVsActor = [50000, 5e-4, 1e-4, 50, 16, 1, 1.0, 1] # working parameters for N = 50
	num_episodes, lr, critic_lr, n, hiddenUnits, batchSize, gamma, criticVsActor = [150000, 5e-5, 1e-5, 20, 16, 1, 1., 10] # working parameters for N = 20

	print(num_episodes, lr, critic_lr, n, hiddenUnits, batchSize, gamma, criticVsActor)
	# Create the environment.
	env = gym.make('LunarLander-v2')
	# env = gym.make('CartPole-v0')
	numStates = env.observation_space.shape[0]
	numActions = env.action_space.n

	# Load the actor model from file.
	# with open(model_config_path, 'r') as f:
	# 	model = keras.models.model_from_json(f.read())
	# embed()
	# TODO: Train the model using A2C and plot the learning curves.

	critic_model, actor_model = createModel(hiddenUnits, numStates, numActions)
	a2c = A2C(actor_model, lr, critic_model, critic_lr, n, numStates, numActions)
	# actorFileName = './model/100/run_1/actor-run1(25000).hdf5'
	# a2c.model.load_weights(actorFileName)
	# criticFileName = './model/100/run_1/critic-run1(25000).hdf5'
	# a2c.critic_model.load_weights(criticFileName)

	logger = Logger('./train_log/' + str(n) + '/')
	for ep in range(1, num_episodes+1):
		rend = False
		if(ep % 500 ==0):
			rend = True
			actorFileName = './model/' + str(n) + '/actor-' + str(ep) + '.hdf5'
			a2c.model.save_weights(actorFileName)
			criticFileName = './model/' + str(n) + '/critic-' + str(ep) + '.hdf5'
			a2c.critic_model.save_weights(criticFileName)
		if(ep % criticVsActor == 0):
			update_actor = True
		else:
			update_actor = False
		actorLoss, criticLoss, reward = a2c.train(env, batchSize, gamma, rend, update_actor)
		logger.log_scalar(tag='reward',value=reward, step=ep)
		logger.log_scalar(tag='criticLoss',value=criticLoss, step=ep)
		if(update_actor):
			logger.log_scalar(tag='actorLoss',value=actorLoss, step=ep)
		print("Episode #: {}, actorloss: {}, criticLoss: {}, reward: {}".format(ep, actorLoss, criticLoss, reward))
if __name__ == '__main__':
	main(sys.argv)

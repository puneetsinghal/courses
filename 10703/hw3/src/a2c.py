import sys
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

from IPython import embed

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from reinforce import Reinforce
try:
	from keras.callbacks import TensorBoard
except Exception as e:
	print("{} from keras.callback. This will prevent gathering data on tensorboard".format(e))

SESS = tf.Session()

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

	def __init__(self, model, lr, critic_model, critic_lr, n=20):
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
		self.X = tf.placeholder(tf.float32, shape=(None, 8))
		self.Y = tf.placeholder(tf.float32, shape=(None, 1))
		self.A = tf.placeholder(tf.float32, shape=(None, 4))
		self.G = tf.placeholder(tf.float32, shape=(None, 1))
		self.criticOutputPlaceHolder = tf.placeholder(tf.float32, shape=(None, 1))

		self.criticOutput = self.critic_model.output
		self.criticLoss = tf.reduce_mean(tf.square(self.G - self.criticOutput))
		self.criticOptimizer = tf.train.AdamOptimizer(critic_lr).minimize(self.criticLoss)
		# (self.critic_model).compile(loss="mse", optimizer=Adam(lr=critic_lr))
		

		self.actorOutput =self.model.output
		# self.grad = tf.gradients()
		# print(self.model.predict(self.X))
		self.logLoss = tf.log(tf.reduce_sum(tf.multiply(self.actorOutput, self.A), 1))
		self.policyLoss = -tf.reduce_mean(tf.multiply((self.G - self.criticOutputPlaceHolder), self.logLoss))
		# self.actorOptimizer = tf.train.GradientDescentOptimizer(0.1).minimize(self.policyLoss)
		self.actorOptimizer = tf.train.AdamOptimizer(lr).minimize(self.policyLoss)

		# checkpoint_weights_filename = './model/critic-' + '{epoch:02d}-{loss:.4f}.h5f'
		# self.callbacks_list = [ModelCheckpoint(checkpoint_weights_filename, monitor='loss', verbose=1, period=100)]
		# log_filename = 'dqn_{}_log.json'.format(args.env_name)
		# callbacks += [FileLogger(log_filename, interval=100)]
		# self.callbacks_list += [TensorBoard('./train_log')]

		# Initialize all variables
		init_op = tf.global_variables_initializer()
		SESS.run(init_op)

	def train(self, env, gamma=1.0, render=False):
		# Trains the model on a single episode using A2C.
		# TODO: Implement this method. It may be helpful to call the class
		#       method generate_episode() to generate training data.
		states, actions, rewards = self.generate_episode(env, render)
		T = len(states)
		numStates = env.observation_space.shape[0]
		cummulativeRewardList = np.zeros((T,1))
		lossPolicy = 0

		# actorOptimizer = tf.train.GradientDescentOptimizer(0.5).minimize(lossPolicy)
		for t in range(T-1, -1, -1):
			if (t+self.n >= T):
				V_end = 0
			else:
				V_end = self.critic_model.predict(np.array(states[t+self.n]).reshape(1,numStates))

			cummulativeReward = (gamma**self.n)*V_end
			for k in range(self.n):
				if(k+t<T):
					cummulativeReward += 1e-2*(gamma**k) * rewards[t+k]
				# else we need to add 0, so skipping that component
			cummulativeRewardList[t,0] = cummulativeReward

			# logLoss = tf.log(self.model.predict(np.array(states[t]).reshape(1,numStates))[0,actions[t]])
			# lossPolicy += (cummulativeReward - self.critic_model.predict(np.array(states[t]).reshape(1,numStates)))*logLoss
		
		# (self.critic_model).fit(np.array(states).reshape(T,numStates), cummulativeRewardList)#, callbacks=self.callbacks_list)
		# (self.critic_model).fit(np.array(states).reshape(T,numStates), cummulativeRewardList, batch_size=T, epochs=1, callbacks=self.callbacks_list)
		
		# self.A = np.array(actions).reshape(T,1)
		A = np.zeros((T,4))
		for i in range(T):
			A[i,actions[i]] = 1
		# actorOptimizer = tf.train.GradientDescentOptimizer(0.5).minimize(self.Y)
		# with SESS.as_default():
		# 	self.actorOptimizer.run(feed_dict={self.X:np.array(states).reshape(T,numStates), self.A:A, self.G:cummulativeRewardList})
		# embed()
		critic_feed_dict = {self.critic_model.input:np.array(states).reshape(T,numStates), self.G:cummulativeRewardList}
		criticLoss, criticOutput, _ = SESS.run([self.criticLoss, self.critic_model.output, self.criticOptimizer], feed_dict= critic_feed_dict)
		
		actor_feed_dict = {self.model.input:np.array(states).reshape(T,numStates), 
			self.A:A, self.G:cummulativeRewardList, self.criticOutputPlaceHolder:criticOutput}
		actorLoss,_ = SESS.run([self.policyLoss, self.actorOptimizer], feed_dict=actor_feed_dict)
		
		return actorLoss, criticLoss, sum(rewards)


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

def createCritic(hiddenUnits, numStates):

	model = Sequential()
	model.add(Dense(hiddenUnits, input_dim=numStates, activation="relu"))
	model.add(Dense(hiddenUnits, activation="relu"))
	model.add(Dense(hiddenUnits, activation="relu"))
	model.add(Dense(1, activation="linear"))
	print("Critic model initialized")
	return model

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
	num_episodes, lr, critic_lr, n, hiddenUnits = [50000, 1e-4, 1e-4, 20, 128]

	# Create the environment.
	env = gym.make('LunarLander-v2')
	numStates = env.observation_space.shape[0]
	numActions = env.action_space.n

	# Load the actor model from file.
	with open(model_config_path, 'r') as f:
		model = keras.models.model_from_json(f.read())

	# TODO: Train the model using A2C and plot the learning curves.

	critic_model = createCritic(hiddenUnits, numStates)
	a2c = A2C(model, lr, critic_model, critic_lr, n)
	logger = Logger('./train_log')
	for ep in range(num_episodes):
		rend = False
		if(ep % 500 ==0):
			rend = True
			actorFileName = './model/actor-' + str(ep) + '.hdf5'
			a2c.model.save_weights(actorFileName)
			criticFileName = './model/critic-' + str(ep) + '.hdf5'
			a2c.critic_model.save_weights(criticFileName)
		actorLoss, criticLoss, reward = a2c.train(env, 1.0, rend)
		logger.log_scalar(tag='reward',value=reward, step=ep)
		logger.log_scalar(tag='actorLoss',value=actorLoss, step=ep)
		logger.log_scalar(tag='criticLoss',value=criticLoss, step=ep)
		print("Episode #: {}, actorloss: {}, criticLoss: {}, reward: {}".format(ep, actorLoss, criticLoss, reward))
if __name__ == '__main__':
	main(sys.argv)

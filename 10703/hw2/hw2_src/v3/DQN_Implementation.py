#!/usr/bin/env python
import keras, tensorflow as tf, numpy as np, gym, sys, copy, argparse
from keras.models import Sequential, load_model
from keras.layers import Dense, Activation
from keras import backend as K
from keras.engine.topology import Layer
from keras.optimizers import Adam
from IPython import embed
import pickle
from collections import deque
from keras.callbacks import TensorBoard
from time import time
np.random.seed(1)

class QNetwork():
	# This class essentially defines the network architecture. 
	# The network should take in state of the world as an input, 
	# and output Q values of the actions available to the agent as the output. 

	def __init__(self, environment_name):
		# Define your network architecture here. It is also a good idea to define any training operations 
		# and optimiz mmers here, initialize your variables, or alternately compile your model here.  
		self.env = gym.make(environment_name)
		self.envName = environment_name
		# Parameters
		self.numStates = (self.env).observation_space.shape[0]
		# print((self.env).action_space.shape)
		self.numActions = (self.env).action_space.n
		self.networkOutputSize = 1
		self.epsilon = 0.5
		self.epsilonLB = 0.05
		self.epsilonDecay = (0.5-0.05)/100000
		# Hyper parameters
		self.iterations = 1000
		if (self.envName == "MountainCar-v0"):
			self.gamma = 1.
			self.learningRate = 0.0001
		else:
			self.gamma = 0.99
			self.learningRate = 0.0001

		# tf Graph input
		# self.state = tf.placeholder(shape=[None, self.numStates], dtype=tf.float32, name="state")
		# self.action = tf.placeholder(shape=[None, self.numActions], dtype=tf.float32, name="action")
		# self.reward = tf.placeholder(shape=[None, 1], dtype=tf.float64, name="reward")
		# self.nextQEvaluation = tf.placeholder(shape=[None,self.numActions],dtype=tf.float32)

		# Construct model
		# self.functionEvaluation = tf.concat([1, self.state],1)
		# self.functionEvaluation = tf.concat([1, tf.concat([self.state, self.action],1)],1)
		# self.inputSize = tf.size(self.functionEvaluation)
		
		self.model = self.createModel()

		# self.weights = tf.Variable("weights", shape=(self.inputSize,self.numActions), initializer=tf.zeros_initializer())
		# self.outputQ = tf.matmul(self.functionEvaluation, self.weights, name="actionValue")
		# self.predict = tf.argmax(self.outputQ,1)

		# Define loss and optimizer
		# self.nextQEvaluation = tf.placeholder(shape=[None,self.numActions],dtype=tf.float32)
		# self.loss = tf.reduce_sum(tf.square(self.nextQEvaluation - self.outputQ))
		# self.trainer = tf.train.GradientDescentOptimizer(learning_rate=self.learningRate)
		# self.updateModel = trainer.minimize(loss)

		pass

	def createModel(self):
		model = Sequential()
		model.add(Dense(self.numActions, input_shape=(self.numStates+1,)))
		model.compile(loss="mean_squared_error", optimizer=Adam(lr=self.learningRate))
		# self.tensorboard = TensorBoard(log_dir="train_log/{}". format(time()))
		return model

	def chooseAction(self, state, policyType):
		# embed(header="inside chooseAction")
		if (policyType == "greedy"):
			return np.argmax(self.model.predict(self.functionApproximation(state))[0])

		self.epsilon -= self.epsilonDecay
		self.epsilon = max(self.epsilon, self.epsilonLB)
		if np.random.random() < self.epsilon:
			return self.env.action_space.sample()
		return np.argmax(self.model.predict(self.functionApproximation(state))[0])

	def functionApproximation(self, state, action=None):
		functionEvaluation = np.hstack((np.ones([1,1]), state))
		return functionEvaluation

	def train(self, numIterations, numSteps, policyType = "epsilon_greedy", rendering = False, model_file=None):
		startIndex = 0
		if not (model_file == None):
			self.load_model_weights(model_file)
			startIndex = 16001
		# embed(header='First time')
		filename = "./" + self.envName + "/" + self.envName  + "_" + policyType + "_final"
		for itr in range(startIndex, numIterations):
			converged = True
			currentState = self.env.reset().reshape(1,self.numStates)
			totalReward = 0
			delta = 0
			for step in range(numSteps):
				if rendering:
					self.env.render()
				action = self.chooseAction(currentState, policyType)
				nextState, reward, done, _ = self.env.step(action)
				totalReward+=reward
				nextState = nextState.reshape(1,self.numStates)
				target = self.model.predict(self.functionApproximation(nextState))
				nextQ = max(self.model.predict(self.functionApproximation(nextState))[0])
				temp = reward + self.gamma * nextQ
				delta = max(delta, abs(temp - target[0][action]))
				target[0][action] = temp
				self.model.fit(self.functionApproximation(currentState), target, epochs=1, verbose=0)#, callbacks = [self.tensorboard])
				currentState = nextState
				if done:
					break

			if self.envName == "MountainCar-v0" and delta > 0.0001: #and step>=170:
				print("Failed to reach goal at trial: {}".format(itr))
				if(itr%2000==0):
					changingFileName = "./" + self.envName + "/" + self.envName + "_" + policyType + "_" + str(itr) + "_iterations_replay"
					self.save_model_weights(changingFileName)	
					self.test(changingFileName)
					# pickle.dump([rewardHistory, stepsHistory], open(filename, 'wb'))

			elif self.envName == "CartPole-v0" and step <=190:
				print("Failed to reach goal at trial: {}".format(itr))
				if(itr%2000==0):
					changingFileName = "./" + self.envName + "/" + self.envName + "_" + policyType + "_" + str(itr) + "_iterations_replay"
					self.save_model_weights(changingFileName)	
					self.test(changingFileName)
					# pickle.dump([rewardHistory, stepsHistory], open(filename, 'wb'))
			else:
				print("Completed in {} trials with {} number of steps".format(itr, step))
				changingFileName = "./" + self.envName + "/" + self.envName + "_" + policyType + "_final_replay.model"
				self.save_model_weights(changingFileName)	
				self.test(changingFileName)
				# pickle.dump([rewardHistory, stepsHistory], open(filename, 'wb'))
				break

	def batchTrain(self, numIterations, numSteps, policyType = "epsilon_greedy", rendering = False, model_file=None):
		startIndex = 0
		if not (model_file == None):
			self.load_model_weights(model_file)
			startIndex = 17001

		self.replayMemory = Replay_Memory(self.envName)
		filename = "./" + self.envName + "/" + self.envName  + "_" + policyType + "_replay_final"
		for itr in range(startIndex, numIterations):
			currentState = self.env.reset()
			delta = 0
			for step in range(numSteps):
				if rendering:
					self.env.render()
				action = self.chooseAction(currentState, policyType)
				nextState, reward, done, _ = self.env.step(action)
				self.Replay_Memory.append([list(currentState), [action], [reward], list(nextState), [done]])

				samples = self.Replay_Memory.sample_batch()
				for sample in samples:
					state, action, reward, nextState, done = sample
					target = self.model.predict(self.functionApproximation(nextState))
					if done:
						target[0][action] = reward
					else:
						nextQ = max(self.model.predict(self.functionApproximation(nextState))[0])
						temp = reward + self.gamma * nextQ
						delta = max(delta, abs(temp - target[0][action]))
						target[0][action] = temp
					self.model.fit(self.functionApproximation(currentState), target, epochs=1, verbose=0)#, callbacks = [self.tensorboard])

			if(self.save_model_weights(itr, delta, policyType)):
				break

	def save_model_weights(self, itr, delta, policyType):
		# Helper function to save your model / weights. 
		if self.envName == "MountainCar-v0" and delta > 0.0001: #and step>=170:
			print("Failed to reach goal at trial: {}".format(itr))
			if(itr%2000==0):
				changingFileName = "./" + self.envName + "/" + self.envName + "_" + policyType + "_" + str(itr) + "_iterations_replay"
				self.model.save_weights(changingFileName)	
				self.test(changingFileName)
				# pickle.dump([rewardHistory, stepsHistory], open(filename, 'wb'))

		elif self.envName == "CartPole-v0" and step <=190:
			print("Failed to reach goal at trial: {}".format(itr))
			if(itr%2000==0):
				changingFileName = "./" + self.envName + "/" + self.envName + "_" + policyType + "_" + str(itr) + "_iterations_replay"
				self.model.save_weights(changingFileName)	
				self.test(changingFileName)
				# pickle.dump([rewardHistory, stepsHistory], open(filename, 'wb'))
		else:
			print("Completed in {} trials with {} number of steps".format(itr, step))
			changingFileName = "./" + self.envName + "/" + self.envName + "_" + policyType + "_final_replay.model"
			self.model.save_weights(changingFileName)	
			self.test(changingFileName)
			# pickle.dump([rewardHistory, stepsHistory], open(filename, 'wb'))
			return True

		return False
		# self.model.save_weights(suffix)

	def load_saved_model(self, model_file):
		# Helper function to load an existing model.
		model = load_model(model_file)
		return model

	def load_model_weights(self, weight_file):
		# Helper funciton to load model weights. 
		self.model.load_weights(weight_file)

	def test(self, model_file):
		# del self.model
		self.load_model_weights(model_file)
		print(self.model.layers[0].get_weights())

		self.epsilon = 0.05
		self.epsilonDecay = 0
		currentState = self.env.reset().reshape(1,self.numStates)
		totalReward = 0
		for step in range(1000):
			self.env.render()
			action = self.chooseAction(currentState, "greedy")
			action = self.env.action_space.sample()
			nextState, reward, done, _ = self.env.step(action)
			# print(action, nextState, reward, done)
			totalReward+=reward
			nextState = nextState.reshape(1, self.numStates)
			currentState = nextState
			if done:
				break

class Replay_Memory():

	def __init__(self, environment_name, memory_size=50000, burn_in=10000):

		# The memory essentially stores transitions recorder from the agent
		# taking actions in the environment.

		# Burn in episodes define the number of episodes that are written into the memory from the 
		# randomly initialized agent. Memory size is the maximum size after which old elements in the memory are replaced. 
		# A simple (if not the most efficient) was to implement the memory is as a list of transitions. 
		env = gym.make(environment_name)

		self.memory = deque(maxlen=memory_size)
		for itr in range(startIndex, numIterations):
			currentState = env.reset()
			for step in range(1000):
				action = env.action_space.sample()
				nextState, reward, done, _ = env.step(action)
				self.memory.append([list(currentState), [action], [reward], list(nextState), [done]])
				currentState = nextState
				if done:
					break

	def sample_batch(self, batch_size=32):
		# This function returns a batch of randomly sampled transitions - i.e. state, action, reward, next state, terminal flag tuples. 
		# You will feed this to your model to train.
		return random.sample(self.memory, batch_size)

	def append(self, transition):
		# Appends transition to the memory.
		self.memory.append(transition)

class DQN_Agent():

	# In this class, we will implement functions to do the following. 
	# (1) Create an instance of the Q Network class.
	# (2) Create a function that constructs a policy from the Q values predicted by the Q Network. 
	#		(a) Epsilon Greedy Policy.
	# 		(b) Greedy Policy. 
	# (3) Create a function to train the Q Network, by interacting with the environment.
	# (4) Create a function to test the Q Network's performance on the environment.
	# (5) Create a function for Experience Replay.
	
	def __init__(self, environment_name, render=False):

		# Create an instance of the network itself, as well as the memory. 
		# Here is also a good place to set environmental parameters,
		# as well as training parameters - number of episodes / iterations, etc. 

		pass 

	def epsilon_greedy_policy(self, q_values):
		# Creating epsilon greedy probabilities to sample from.             
		pass

	def greedy_policy(self, q_values):
		# Creating greedy policy for test time. 
		pass 

	def train(self):
		# In this function, we will train our network. 
		# If training without experience replay_memory, then you will interact with the environment 
		# in this function, while also updating your network parameters. 

		# If you are using a replay memory, you should interact with environment here, and store these 
		# transitions to memory, while also updating your model.
		pass

	def test(self, model_file=None):
		# Evaluate the performance of your agent over 100 episodes, by calculating cummulative rewards for the 100 episodes.
		# Here you need to interact with the environment, irrespective of whether you are using a memory. 
		pass
	def burn_in_memory():
		# Initialize your replay memory with a burn_in number of episodes / transitions. 

		pass

def parse_arguments():
	parser = argparse.ArgumentParser(description='Deep Q Network Argument Parser')
	parser.add_argument('--env',dest='env',type=str)
	parser.add_argument('--render',dest='render',type=int,default=0)
	parser.add_argument('--train',dest='train',type=int,default=1)
	parser.add_argument('--model',dest='model_file',type=str)
	return parser.parse_args()

def main(args):

	args = parse_arguments()
	environment_name = args.env
	if(args.render == 1):
		rendering = True
	else:
		rendering = False
	model_file = args.model_file
	mode = args.train

	# Setting the session to allow growth, so it doesn't allocate all GPU memory. 
	gpu_ops = tf.GPUOptions(allow_growth=True)
	config = tf.ConfigProto(gpu_options=gpu_ops)
	sess = tf.Session(config=config)

	# Setting this as the default tensorflow session. 
	keras.backend.tensorflow_backend.set_session(sess)

	linearQAgent = QNetwork(environment_name)

	if(mode == 1):
		if(environment_name == "MountainCar-v0"):
			numIterations = 100000
			numSteps = 500
			
		if(environment_name == "CartPole-v0"):
			numIterations = 100000
			numSteps = 250

		policyType = "greedy"
		# policyType = "epsilon_greedy"
		linearQAgent.train(numIterations, numSteps, policyType, rendering, model_file)
		model_file = "./" + linearQAgent.envName + "/" + linearQAgent.envName + "_" + policyType + "_final.model"
		linearQAgent.test(model_file)

	# You want to create an instance of the DQN_Agent class here, and then train / test it. 
	else:
		if model_file == None:
			policyType = "greedy"	
			# policyType = "epsilon_greedy"

			model_file = "./" + linearQAgent.envName + "/" + linearQAgent.envName + "_" + policyType + "_final.model"
		linearQAgent.test(model_file)

if __name__ == '__main__':
	main(sys.argv)


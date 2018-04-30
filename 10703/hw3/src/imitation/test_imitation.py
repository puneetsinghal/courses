import sys
import argparse
import numpy as np
import keras
import random
import gym
from IPython import embed
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint
import pickle
import matplotlib.pyplot as plt

class Imitation():
	def __init__(self, model_config_path, expert_weights_path, model_weights_path):
		# Load the expert model.
		
		with open(model_config_path, 'r') as f:
			self.expert = keras.models.model_from_json(f.read())
		self.expert.load_weights(expert_weights_path)
		
		# Initialize the cloned model (to be trained).
		with open(model_config_path, 'r') as f:
			self.model = keras.models.model_from_json(f.read())
		self.model.load_weights(model_weights_path)
		
		# TODO: Define any training operations and optimizers here, initialize
		#       your variables, or alternatively compile your model here.
		learningRate = 1e-2
		(self.model).compile(loss="categorical_crossentropy", optimizer=Adam(lr=learningRate), metrics=['accuracy'])

	def run_expert(self, env, render=False):
		# Generates an episode by running the expert policy on the given env.
		return Imitation.generate_episode(self.expert, env, render)

	def run_model(self, env, render=False):
		# Generates an episode by running the cloned policy on the given env.
		return Imitation.generate_episode(self.model, env, render)

	@staticmethod
	def generate_episode(model, env, render=False):
		# Generates an episode by running the given model on the given env.
		# Returns:
		# - a list of states, indexed by time step
		# - a list of actions, indexed by time step
		# - a list of rewards, indexed by time step
		# TODO: Implement this method.
		numStates = env.observation_space.shape[0]
		numActions = env.action_space.n
		states = []
		actions = []
		rewards = []
		s = env.reset()
		done = False
		while not done:
			if render: 
				env.render()
			states.append(s)
			a = np.argmax(model.predict(s.reshape(1,numStates)))
			s, r, done, _ = env.step(a)
			rewards.append(r)
			actions.append(a)
		return states, actions, rewards
	
	def train(self, env, num_episodes=100, num_epochs=50, render=False):
		# Trains the model on training data generated by the expert policy.
		# Args:
		# - env: The environment to run the expert policy on. 
		# - num_episodes: # episodes to be generated by the expert.
		# - num_epochs: # epochs to train on the data generated by the expert.
		# - render: Whether to render the environment.
		# Returns the final loss and accuracy.
		# TODO: Implement this method. It may be helpful to call the class
		#       method run_expert() to generate training data.
		loss = 0
		acc = 0
		return loss, acc


def parse_arguments():
	# Command-line flags are defined here.
	parser = argparse.ArgumentParser()
	parser.add_argument('--model-config-path', dest='model_config_path',
						type=str, default='LunarLander-v2-config.json',
						help="Path to the model config file.")
	parser.add_argument('--expert-weights-path', dest='expert_weights_path',
						type=str, default='LunarLander-v2-weights.h5',
						help="Path to the expert weights file.")

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

def sampleBatch(states, actions, rewards, batchSize):
	random.seed(a=None)
	indeces = random.sample(rangle(0,len(actions)),batchSize)
	statesOutput = np.zeros((batchSize, 8))
	actionOutput = np.zeros((batchSize, 4))
	
	for i in range(batchSize):
		statesOutput[i,:] = states[indeces[i]]
		# actionOutput[i,:] = actions[indeces[i]]
		actionOutput[i,actions[indeces[i]]] = 1
	
	return statesOutput, actionOutput

def main(args):
	# Parse command-line arguments.
	args = parse_arguments()
	model_config_path = args.model_config_path
	expert_weights_path = args.expert_weights_path
	render = args.render

	# Create the environment.
	env = gym.make('LunarLander-v2')
	numActions = env.action_space.n
	numStates = env.observation_space.shape[0]

	# TODO: Train cloned models using imitation learning, and record their
	#       performance.
	# Load the actor model from file.
	N = [1, 10, 50, 100, 150]
	samplingFreq = 500
	EPISODES = 1
	REWARD = np.zeros((2, len(N), EPISODES))
	MEAN = np.zeros((2, len(N)))
	STD_VAR = np.zeros((2, len(N)))

	# for trial in range(len(N)):
	# 	n = N[trial]
	# 	model_weights_path="./model/imitation/imitation-train_" + str(n) + "_final.hdf5"		
	# 	IMT = Imitation(model_config_path, expert_weights_path, model_weights_path)

	# 	for test_ep_index in range(EPISODES):
	# 		s, a, r = IMT.run_expert(env, render)
	# 		REWARD[0, trial, test_ep_index] = sum(r)
			
	# 		s, a, r = IMT.run_model(env, render)
	# 		REWARD[1, trial, test_ep_index] = sum(r)
		
	# 	MEAN[0, trial] = np.mean(REWARD[0, trial,:])
	# 	STD_VAR[0, trial] = np.std(REWARD[0, trial,:])
		
	# 	MEAN[1, trial] = np.mean(REWARD[1, trial,:])
	# 	STD_VAR[1, trial] = np.std(REWARD[1, trial,:])
	# 	print("index #: {}, MEAN: {}, STD_VAR: {}".format(n, MEAN[0, trial], STD_VAR[0, trial]))
	# 	print("index #: {}, MEAN: {}, STD_VAR: {}".format(n, MEAN[1, trial], STD_VAR[1, trial]))
		
	# 	pickle.dump([REWARD, MEAN, STD_VAR], open('./results_imitation', 'wb'))

	# REWARD, MEAN, STD_VAR = pickle.load(open('./results_imitation', 'rb'))
	# print(EPISODES.shape, MEAN[0].shape, STD_VAR[0].shape)
	embed()
	plt.figure()
	plt.errorbar(np.array(N), MEAN[0], STD_VAR[0])
	plt.errorbar(np.array(N), MEAN[1], STD_VAR[1])
	plt.show()

def resultsAccuracy(args):
	# Parse command-line arguments.
	args = parse_arguments()
	model_config_path = args.model_config_path
	expert_weights_path = args.expert_weights_path
	render = args.render

	# Create the environment.
	env = gym.make('LunarLander-v2')
	numActions = env.action_space.n
	numStates = env.observation_space.shape[0]

	# TODO: Train cloned models using imitation learning, and record their
	#       performance.
	# Load the actor model from file.
	N = [1, 10, 50, 100]#, 150]
	inliers = np.zeros(len(N))
	total = np.zeros(len(N))
	accuracy = np.zeros(len(N))

	samplingFreq = 500
	EPISODES = 1

	for trial in range(len(N)):
		n = N[trial]
		model_weights_path="./model/imitation/imitation-train_" + str(n) + "_final.hdf5"		
		IMT = Imitation(model_config_path, expert_weights_path, model_weights_path)
		
		s = env.reset()

		done = False
		while not done:
			a1 = np.argmax(IMT.model.predict(s.reshape(1,numStates)))
			a2 = np.argmax(IMT.expert.predict(s.reshape(1,numStates)))
			if (a1==a2):
				inliers[trial] += 1
			total[trial] += 1
			s, _, done, _ = env.step(a1)
		
		accuracy[trial] = 100.*inliers[trial]/total[trial]
		print("trial: {} has inliers: {}, and total: {} and accuracy: {}".format(trial, inliers[trial], total[trial], accuracy[trial]))

	pickle.dump([inliers, total, accuracy], open('./results_imitation_accuracy', 'wb'))
	plt.figure()
	plt.plot(np.array(N), accuracy, marker='o', linestyle='-', color='k', label='Square')
	plt.show()

if __name__ == '__main__':
	# main(sys.argv)
	resultsAccuracy(sys.argv)
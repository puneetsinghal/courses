import gym
import deeprl_hw1.lake_envs
from deeprl_hw1.rl import *
import time as time

import matplotlib.pyplot as plt
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--case', type=int, default=1)
args = parser.parse_args()

action_names = {0:"L", 1:"D", 2:"R", 3:"U"}

if(args.case == 1): # Deterministic Synchronized policy iteration 
	env = gym.make('Deterministic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numPolicyIterations, numValueIterations = policy_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Synchronous Policy Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("value_func: \n{}".format(value_func.reshape(size,size)))
	print("numValueIterations: {}".format(numValueIterations))
	print("numPolicyIterations: {}".format(numPolicyIterations))
	print_policy(policy, action_names)
	plt.figure(1)
	plt.imshow(value_func.reshape(size,size))
	plt.colorbar()

	env = gym.make('Deterministic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numPolicyIterations, numValueIterations = policy_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("8X8: Synchronous Policy Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("value_func: \n{}".format(value_func.reshape(size,size)))
	print("numValueIterations: {}".format(numValueIterations))
	print("numPolicyIterations: {}".format(numPolicyIterations))
	print_policy(policy, action_names)
	plt.figure(2)
	plt.imshow(value_func.reshape(size,size))
	plt.colorbar()

	plt.show()

if(args.case == 2): # Deterministic Synchronized value iteration 
	env = gym.make('Deterministic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numValueIterations = value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Synchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("numValueIterations: {}".format( numValueIterations))
	print_policy(policy, action_names)
	plt.figure(1)
	plt.imshow(value_func.reshape(size,size))
	plt.colorbar()

	env = gym.make('Deterministic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numValueIterations = value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)
	
	print("8X8: Synchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("numValueIterations: {}".format( numValueIterations))
	print_policy(policy, action_names)
	plt.figure(2)
	plt.imshow(value_func.reshape(size,size))
	plt.colorbar()

	plt.show()

if(args.case ==3): # Deterministic Asynchronized policy iteration 
	env = gym.make('Deterministic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, value_func, numPolicyIterationsOrdered, numValueIterationsOrdered = policy_iteration_async_ordered(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("Asynchronous Policy Iteration (ordered): \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policyOrdered.reshape(size,size)))
	print("value_func: \n{}".format(value_func.reshape(size,size)))
	print("numValueIterations: {}".format(numValueIterationsOrdered))
	print("numPolicyIterations: {}".format(numPolicyIterationsOrdered))

	time1 = time.clock()
	policyRandomPerm, value_func, numPolicyIterationsRandomPerm, numValueIterationsRandomPerm = policy_iteration_async_randperm(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("Asynchronous Policy Iteration (random permutation): \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policyRandomPerm.reshape(size,size)))
	print("value_func: \n{}".format(value_func.reshape(size,size)))
	print("numValueIterations: {}".format(numValueIterationsRandomPerm))
	print("numPolicyIterations: {}".format(numPolicyIterationsRandomPerm))

if(args.case == 4): # Deterministic Asynchronized value iteration 
	env = gym.make('Deterministic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsOrdered = value_iteration_async_ordered(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("Asynchronous Value Iteration (ordered): \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policyOrdered.reshape(size,size)))
	print("numValueIterations: {}".format(numValueIterationsOrdered))

	time1 = time.clock()
	policyRandomPerm, numValueIterationsRandomPerm = value_iteration_async_randperm(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("Asynchronous Value Iteration (random permutation): \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policyRandomPerm.reshape(size,size)))
	print("numValueIterations: {}".format(numValueIterationsRandomPerm))

if(args.case== 5): # Stochastic Synchronized value iteration 
	env = gym.make('Stochastic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numValueIterations = value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Synchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("numValueIterations: {}".format( numValueIterations))
	print_policy(policy, action_names)
	plt.figure(1)
	plt.imshow(value_func.reshape(size,size))
	plt.colorbar()

	env = gym.make('Stochastic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numValueIterations = value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("8X8: Synchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("numValueIterations: {}".format( numValueIterations))
	print_policy(policy, action_names)
	plt.figure(2)
	plt.imshow(value_func.reshape(size,size))
	plt.colorbar()

	plt.show()

if(args.case == 6): # Stochastic Synchronized value iteration Testing
	env = gym.make('Stochastic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numValueIterations = value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	N = 100
	cummulativeReward = 0
	for trial in range(N):
		nextState = env.reset()
		# env.render()
		# time.sleep(1)  # just pauses so you can see the output
		rewardVector = []
		num_steps = 0
		while True:
			nextState, reward, is_terminal, debug_info = env.step(policy[nextState])
			# env.render()

			rewardVector += [reward]
			num_steps += 1

			if is_terminal:
				break

			# time.sleep(1)
		weight = 1.0
		for r in rewardVector:
			cummulativeReward += r*weight
			weight*=gamma


	print("cummulative reward for 4x4 stochastic map = {}".format(cummulativeReward/N))
	print("Value computed for starting state = {}".format(value_func[env.reset()]))

	env = gym.make('Stochastic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numValueIterations = value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	cummulativeReward = 0
	for trial in range(N):
		nextState = env.reset()
		# env.render()
		# time.sleep(1)  # just pauses so you can see the output
		rewardVector = []
		num_steps = 0
		while True:
			nextState, reward, is_terminal, debug_info = env.step(policy[nextState])
			# env.render()

			rewardVector += [reward]
			num_steps += 1

			if is_terminal:
				break

			# time.sleep(1)
		weight = 1.0
		for r in rewardVector:
			cummulativeReward += r*weight
			weight*=gamma


	print("cummulative reward for 8x8 stochastic map = {}".format(cummulativeReward/N))
	print("Value computed for starting state = {}".format(value_func[env.reset()]))

if(args.case == 7): # Deterministic Synchronized value iteration Testing
	env = gym.make('Deterministic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numValueIterations = value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	N = 1
	cummulativeReward = 0
	for trial in range(N):
		nextState = env.reset()
		# env.render()
		# time.sleep(1)  # just pauses so you can see the output
		rewardVector = []
		num_steps = 0
		while True:
			nextState, reward, is_terminal, debug_info = env.step(policy[nextState])
			# env.render()

			rewardVector += [reward]
			num_steps += 1

			if is_terminal:
				break

			# time.sleep(1)
		weight = 1.0
		for r in rewardVector:
			cummulativeReward += r*weight
			weight*=gamma


	print("cummulative reward for 4x4 deterministic map = {}".format(cummulativeReward/N))
	print("Value computed for starting state = {}".format(value_func[env.reset()]))

	env = gym.make('Deterministic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, value_func, numValueIterations = value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	cummulativeReward = 0
	for trial in range(N):
		nextState = env.reset()
		# env.render()
		# time.sleep(1)  # just pauses so you can see the output
		rewardVector = []
		num_steps = 0
		while True:
			nextState, reward, is_terminal, debug_info = env.step(policy[nextState])
			# env.render()

			rewardVector += [reward]
			num_steps += 1

			if is_terminal:
				break

			# time.sleep(1)
		weight = 1.0
		for r in rewardVector:
			cummulativeReward += r*weight
			weight*=gamma


	print("cummulative reward for 8x8 stochastic map = {}".format(cummulativeReward/N))
	print("Value computed for starting state = {}".format(value_func[env.reset()]))

if(args.case== 8): #  Asynchronized value iteration with custom heuristic
	env = gym.make('Deterministic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, numStateUpdates = value_iteration_async_custom(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Deterministic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("numStateUpdates: {}".format( numStateUpdates))

	env = gym.make('Deterministic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, numStateUpdates = value_iteration_async_custom(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("8X8: Deterministic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("numStateUpdates: {}".format( numStateUpdates))

	env = gym.make('Stochastic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, numStateUpdates = value_iteration_async_custom(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Stochastic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("numStateUpdates: {}".format( numStateUpdates))

	env = gym.make('Stochastic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policy, numStateUpdates = value_iteration_async_custom(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("8X8: Stochastic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policy.reshape(size,size)))
	print("numStateUpdates: {}".format( numStateUpdates))

if(args.case== 9): #  Asynchronized value iteration with ordered heuristic
	env = gym.make('Deterministic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsOrdered = value_iteration_async_ordered(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Deterministic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policyOrdered.reshape(size,size)))
	print("numStateUpdates: {}".format( numValueIterationsOrdered))

	env = gym.make('Deterministic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsOrdered = value_iteration_async_ordered(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("8X8: Deterministic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policyOrdered.reshape(size,size)))
	print("numStateUpdates: {}".format( numValueIterationsOrdered))

	env = gym.make('Stochastic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsOrdered = value_iteration_async_ordered(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Stochastic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policyOrdered.reshape(size,size)))
	print("numStateUpdates: {}".format( numValueIterationsOrdered))

	env = gym.make('Stochastic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsOrdered = value_iteration_async_ordered(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("8X8: Stochastic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	print("policy: \n{}".format(policyOrdered.reshape(size,size)))
	print("numStateUpdates: {}".format( numValueIterationsOrdered))

if(args.case== 10): #  Asynchronized value iteration with randPerm heuristic
	env = gym.make('Deterministic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsRandomPerm = value_iteration_async_randperm(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Deterministic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	# print("policy: \n{}".format(policy.reshape(size,size)))
	print("numStateUpdates: {}".format( numValueIterationsRandomPerm))

	env = gym.make('Deterministic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsRandomPerm = value_iteration_async_randperm(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("8X8: Deterministic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	# print("policy: \n{}".format(policy.reshape(size,size)))
	print("numStateUpdates: {}".format( numValueIterationsRandomPerm))

	env = gym.make('Stochastic-4x4-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsRandomPerm = value_iteration_async_randperm(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("4X4: Stochastic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	# print("policy: \n{}".format(policy.reshape(size,size)))
	print("numStateUpdates: {}".format( numValueIterationsRandomPerm))

	env = gym.make('Stochastic-8x8-FrozenLake-v0')
	gamma = 0.9
	time1 = time.clock()
	policyOrdered, numValueIterationsRandomPerm = value_iteration_async_randperm(env, gamma, max_iterations=int(1e3), tol=1e-3)
	time2 = time.clock()

	size = int(env.nS**0.5)

	print("8X8: Stochastic Asynchronous Value Iteration: \ntime: {}".format((time2-time1)))
	# print("policy: \n{}".format(policy.reshape(size,size)))
	print("numStateUpdates: {}".format( numValueIterationsRandomPerm))


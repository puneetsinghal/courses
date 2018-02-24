# coding: utf-8
from __future__ import division, absolute_import
from __future__ import print_function, unicode_literals

import numpy as np
from math import *
from copy import copy, deepcopy
import queue as Queue


def print_policy(policy, action_names):
		"""Print the policy in human-readable format.

		Parameters
		----------
		policy: np.ndarray
			Array of state to action number mappings
		action_names: dict
			Mapping of action numbers to characters representing the action.
		"""
		str_policy = policy.astype('str')
		for action_num, action_name in action_names.items():
				np.place(str_policy, policy == action_num, action_name)

		print(str_policy)


def value_function_to_policy(env, gamma, value_function):
	"""Output action numbers for each state in value_function.

	Parameters
	----------
	env: gym.core.Environment
		Environment to compute policy for. Must have nS, nA, and P as
		attributes.
	gamma: float
		Discount factor. Number in range [0, 1)
	value_function: np.ndarray
		Value of each state.

	Returns
	-------
	np.ndarray
		An array of integers. Each integer is the optimal action to take
		in that state according to the environment dynamics and the
		given value function.
	"""  
	
	policy = np.zeros(env.nS, dtype='int')
	
	for state in range(env.nS):
		q = np.zeros(4)
		for action in range(env.nA):
			# prob, nextState, reward, is_terminal = (env.P[state][action])[0]
			# q[action] = prob*(reward+gamma*value_func[nextState])
			q[action] = sum(1.0*prob*(reward + gamma*value_function[nextState]) 
										for (prob, nextState, reward, is_terminal) in env.P[state][action])
		policy[state] = np.argmax(q)

	return False, policy


def evaluate_policy_sync(env, gamma, policy, max_iterations=int(1e3), tol=1e-3):
	"""Performs policy evaluation.
	
	Evaluates the value of a given policy.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	policy: np.array
		The policy to evaluate. Maps states to actions.
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	np.ndarray, int
		The value for the given policy and the number of iterations till
		the value function converged.
	"""

	# value = np.zeros(env.nS)
	# for itr in range (max_iterations):
	# 	newValue = np.copy(value)
	# 	for state in range(env.nS):
	# 		action = policy[state]
	# 		prob, nextState, reward, is_terminal = (env.P[state][action])[0]
	# 		if not is_terminal:
	# 			newValue[state] = 1.0*prob*(reward + gamma*value[nextState])
	# 		else:
	# 			newValue[state] = 1.0*prob*(reward)
	# 	if(np.sum(np.fabs(newValue-value)) < tol):
	# 		return value, itr
	# 	value = np.copy(newValue)

	# return value, itr

	value_func = np.zeros(env.nS)
	for itr in range (max_iterations):
		new_value_func = np.copy(value_func)
		delta = 0
		for state in range(env.nS):
			old_value = value_func[state]
			action = policy[state]
			# prob, nextState, reward, is_terminal = (env.P[state][action])[0]
			# if not is_terminal:
			# 	# newValue[state] = 1.0*prob*(reward + gamma*value_func[nextState])
			# 	new_value_func[state] = 1.0*prob*(reward + gamma*value_func[nextState])
			# else:
			# 	# newValue[state] = 1.0*prob*(reward)
			# 	new_value_func[state] = 1.0*prob*(reward)

			new_value_func[state] = sum(1.0*prob*(reward + gamma*value_func[nextState]) 
										for (prob, nextState, reward, is_terminal) in env.P[state][action])

			delta = max(delta, fabs(old_value - new_value_func[state]))
		value_func = np.copy(new_value_func)
		if(delta<tol):
			break

	return value_func, itr+1


def evaluate_policy_async_ordered(env, gamma, policy, max_iterations=int(1e3), tol=1e-3):
	"""Performs policy evaluation.
	
	Evaluates the value of a given policy by asynchronous DP.  Updates states in
	their 1-N order.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	policy: np.array
		The policy to evaluate. Maps states to actions.
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	np.ndarray, int
		The value for the given policy and the number of iterations till
		the value function converged.
	"""
	value_func = np.zeros(env.nS)
	for itr in range (max_iterations):
		delta = 0
		for state in range(env.nS):
			old_value = value_func[state]
			action = policy[state]
			# prob, nextState, reward, is_terminal = (env.P[state][action])[0]
			# if not is_terminal:
			# 	# newValue[state] = 1.0*prob*(reward + gamma*value_func[nextState])
			# 	value_func[state] = 1.0*prob*(reward + gamma*value_func[nextState])
			# else:
			# 	# newValue[state] = 1.0*prob*(reward)
			# 	value_func[state] = 1.0*prob*(reward)
			value_func[state] = sum(1.0*prob*(reward + gamma*value_func[nextState]) 
										for (prob, nextState, reward, is_terminal) in env.P[state][action])
			delta = max(delta, fabs(old_value - value_func[state]))
		if(delta<tol):
			break

	return value_func, itr+1


def evaluate_policy_async_randperm(env, gamma, policy, max_iterations=int(1e3), tol=1e-3):
	"""Performs policy evaluation.
	
	Evaluates the value of a policy.  Updates states by randomly sampling index
	order permutations.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	policy: np.array
		The policy to evaluate. Maps states to actions.
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	np.ndarray, int
		The value for the given policy and the number of iterations till
		the value function converged.
	"""
	# value = np.zeros(env.nS)
	# for itr in range (max_iterations):
	# 	# newValue = np.copy(value)
	# 	randomPermStates = np.random.permutation(env.nS)
	# 	for state in randomPermStates:
	# 		action = policy[state]
	# 		prob, nextState, reward, is_terminal = (env.P[state][action])[0]
	# 		if not is_terminal:
	# 			# newValue[state] = 1.0*prob*(reward + gamma*value[nextState])
	# 			value[state] = 1.0*prob*(reward + gamma*value[nextState])
	# 		else:
	# 			# newValue[state] = 1.0*prob*(reward)
	# 			value[state] = 1.0*prob*(reward)
	# 	if(np.sum(np.fabs(newValue-value)) < tol):
	# 		return value, itr
	# 	# value = np.copy(newValue)

	# return value, itr

	value_func = np.zeros(env.nS)
	for itr in range (max_iterations):
		delta = 0
		randomPermStates = np.random.permutation(env.nS)
		for state in randomPermStates:
			old_value = value_func[state]
			action = policy[state]
			# prob, nextState, reward, is_terminal = (env.P[state][action])[0]
			# if not is_terminal:
			# 	# newValue[state] = 1.0*prob*(reward + gamma*value_func[nextState])
			# 	value_func[state] = 1.0*prob*(reward + gamma*value_func[nextState])
			# else:
			# 	# newValue[state] = 1.0*prob*(reward)
			# 	value_func[state] = 1.0*prob*(reward)
			value_func[state] = sum(1.0*prob*(reward + gamma*value_func[nextState]) 
										for (prob, nextState, reward, is_terminal) in env.P[state][action])
			delta = max(delta, fabs(old_value - value_func[state]))
		if(delta<tol):
			break

	return value_func, itr+1


def evaluate_policy_async_custom(env, gamma, policy, max_iterations=int(1e3), tol=1e-3):
	"""Performs policy evaluation.
	
	Evaluate the value of a policy. Updates states by a student-defined
	heuristic. 

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	policy: np.array
		The policy to evaluate. Maps states to actions.
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	np.ndarray, int
		The value for the given policy and the number of iterations till
		the value function converged.
	"""
	return np.zeros(env.nS), 0


def improve_policy(env, gamma, value_func, policy):
	"""Performs policy improvement.

	Given a policy and value function, improves the policy.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	value_func: np.ndarray
		Value function for the given policy.
	policy: dict or np.array
		The policy to improve. Maps states to actions.

	Returns
	-------
	bool, np.ndarray
		Returns true if policy changed. Also returns the new policy.
	"""

	policyStable = True

	for state in range(env.nS):
		oldAction = policy[state]
		q = np.zeros(4)
		for action in range(env.nA):
			# prob, nextState, reward, is_terminal = (env.P[state][action])[0]
			# q[action] = prob*(reward+gamma*value_func[nextState])
			q[action] = sum(1.0*prob*(reward + gamma*value_func[nextState]) 
										for (prob, nextState, reward, is_terminal) in env.P[state][action])
		# qmax = np.amax(q)
		# maxIndices = (q==qmax)
		# maxIndices = maxIndices/sum(maxIndices)
		# policy[state] = np.random.choice(4, size=1, p=maxIndices.tolist())
		policy[state] = np.argmax(q)
		if not (oldAction == policy[state]): 
			policyStable = False
	return policyStable, policy


def policy_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3):
	"""Runs policy iteration.

	See page 85 of the Sutton & Barto Second Edition book.

	You should use the improve_policy() and evaluate_policy_sync() methods to
	implement this method.
	
	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	(np.ndarray, np.ndarray, int, int)
		 Returns optimal policy, value function, number of policy
		 improvement iterations, and number of value iterations.
	"""

	policy = np.zeros(env.nS, dtype='int')
	value_func = np.zeros(env.nS)
	numValueIterations = 0
	for numPolicyIterations in range(max_iterations):
		value_func, iterations = evaluate_policy_sync(env, gamma, policy, tol = tol)
		# print(value_func, iterations)
		numValueIterations += iterations
		policyStable, policy = improve_policy(env, gamma, value_func, policy)
		# print(policyStable, policy)
		if(policyStable):
			return policy, value_func, numPolicyIterations+1, numValueIterations

	return policy, value_func, numPolicyIterations+1, numValueIterations


def policy_iteration_async_ordered(env, gamma, max_iterations=int(1e3), tol=1e-3):
	"""Runs policy iteration.

	You should use the improve_policy and evaluate_policy_async_ordered methods
	to implement this method.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	(np.ndarray, np.ndarray, int, int)
		 Returns optimal policy, value function, number of policy
		 improvement iterations, and number of value iterations.
	"""
	policy = np.zeros(env.nS, dtype='int')
	value_func = np.zeros(env.nS)
	numValueIterations = 0
	for numPolicyIterations in range(max_iterations):
		value_func, iterations = evaluate_policy_async_ordered(env, gamma, policy, tol = tol)
		# print(value_func, iterations)
		numValueIterations += iterations
		policyStable, policy = improve_policy(env, gamma, value_func, policy)
		# print(policyStable, policy)
		if(policyStable):
			return policy, value_func, numPolicyIterations+1, numValueIterations

	return policy, value_func, numPolicyIterations+1, numValueIterations


def policy_iteration_async_randperm(env, gamma, max_iterations=int(1e3), tol=1e-3):
	"""Runs policy iteration.

	You should use the improve_policy and evaluate_policy_async_randperm methods
	to implement this method.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	(np.ndarray, np.ndarray, int, int)
		 Returns optimal policy, value function, number of policy
		 improvement iterations, and number of value iterations.
	"""
	policy = np.zeros(env.nS, dtype='int')
	value_func = np.zeros(env.nS)
	numValueIterations = 0
	for numPolicyIterations in range(max_iterations):
		value_func, iterations = evaluate_policy_async_randperm(env, gamma, policy, tol = tol)
		# print(value_func, iterations)
		numValueIterations += iterations
		policyStable, policy = improve_policy(env, gamma, value_func, policy)
		# print(policyStable, policy)
		if(policyStable):
			return policy, value_func, numPolicyIterations+1, numValueIterations

	return policy, value_func, numPolicyIterations+1, numValueIterations


def policy_iteration_async_custom(env, gamma, max_iterations=int(1e3), tol=1e-3):
	"""Runs policy iteration.

	You should use the improve_policy and evaluate_policy_async_custom methods
	to implement this method.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	(np.ndarray, np.ndarray, int, int)
		 Returns optimal policy, value function, number of policy
		 improvement iterations, and number of value iterations.
	"""
	policy = np.zeros(env.nS, dtype='int')
	value_func = np.zeros(env.nS)

	return policy, value_func, 0, 0


def value_iteration_sync(env, gamma, max_iterations=int(1e3), tol=1e-3):
	"""Runs value iteration for a given gamma and environment.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	np.ndarray, iteration
		The value function and the number of iterations it took to converge.
	"""
	policy = np.zeros(env.nS, dtype='int')
	value_func = np.zeros(env.nS)
	new_value_func = np.copy(value_func)

	for numValueIterations in range (max_iterations):
		delta = 0
		for state in range(env.nS):
			temp = 0
			for action in range(env.nA):
				temp = max(temp , sum(1.0*prob*(reward + gamma*value_func[nextState]) 
										for (prob, nextState, reward, is_terminal) in env.P[state][action]))
			new_value_func[state] = temp
			delta = max(delta, fabs(value_func[state] - new_value_func[state]))

		value_func = np.copy(new_value_func)
		if(delta<tol):
			break

	# print(value_func.reshape(int(env.nS**0.5),int(env.nS**0.5)))
	return value_func, numValueIterations+1
	# return np.zeros(env.nS), 0


def value_iteration_async_ordered(env, gamma, max_iterations=int(1e3), tol=1e-3):
	"""Runs value iteration for a given gamma and environment.
	Updates states in their 1-N order.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	np.ndarray, iteration
		The value function and the number of iterations it took to converge.
	"""
	policy = np.zeros(env.nS, dtype='int')
	value_func = np.zeros(env.nS)

	for numValueIterations in range (max_iterations):
		delta = 0
		for state in range(env.nS):
			old_value = value_func[state]
			temp = 0
			for action in range(env.nA):
				temp = max(temp , sum(1.0*prob*(reward + gamma*value_func[nextState]) 
										for (prob, nextState, reward, is_terminal) in env.P[state][action]))
			value_func[state] = temp
			delta = max(delta, fabs(old_value - value_func[state]))
		if(delta<tol):
			break
	# print(value_func.reshape(int(env.nS**0.5),int(env.nS**0.5)))
	_, policy = value_function_to_policy(env, gamma, value_func)

	return policy, numValueIterations+1
	# return np.zeros(env.nS), 0


def value_iteration_async_randperm(env, gamma, max_iterations=int(1e3), tol=1e-3):
	"""Runs value iteration for a given gamma and environment.
	Updates states by randomly sampling index order permutations.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	np.ndarray, iteration
		The value function and the number of iterations it took to converge.
	"""
	policy = np.zeros(env.nS, dtype='int')
	value_func = np.zeros(env.nS)

	for numValueIterations in range (max_iterations):
		delta = 0
		randomPermStates = np.random.permutation(env.nS)
		for state in randomPermStates:
			old_value = value_func[state]
			temp = 0
			for action in range(env.nA):
				temp = max(temp , sum(1.0*prob*(reward + gamma*value_func[nextState]) 
										for (prob, nextState, reward, is_terminal) in env.P[state][action]))
			value_func[state] = temp
			delta = max(delta, fabs(old_value - value_func[state]))
		if(delta<tol):
			break
	# print(value_func.reshape(int(env.nS**0.5),int(env.nS**0.5)))
	_, policy = value_function_to_policy(env, gamma, value_func)

	return policy, numValueIterations+1
	# return np.zeros(env.nS), 0


def value_iteration_async_custom(env, gamma, max_iterations=int(1e3), tol=1e-3):
	"""Runs value iteration for a given gamma and environment.
	Updates states by student-defined heuristic.

	Parameters
	----------
	env: gym.core.Environment
		The environment to compute value iteration for. Must have nS,
		nA, and P as attributes.
	gamma: float
		Discount factor, must be in range [0, 1)
	max_iterations: int
		The maximum number of iterations to run before stopping.
	tol: float
		Determines when value function has converged.

	Returns
	-------
	np.ndarray, iteration
		The value function and the number of iterations it took to converge.
	"""
	policy = np.zeros(env.nS, dtype='int')
	value_func = np.zeros(env.nS)
	
	openQueue = Queue.PriorityQueue()
	numStateEvaluations = 0
	numStableValues = 0

	for state in range(env.nS):
		terminalValue = 0
		if len(env.P[state]) == 4:
			for action in range(env.nA):
				_, _, _, is_terminal = env.P[state][action][0]
				terminalValue += is_terminal
		if(terminalValue < 4):
			openQueue.put((0., 1, state))

	while (numStableValues<openQueue.qsize()):
		numStateEvaluations +=1
		priorityValue, entry_count, state = openQueue.get()
		old_value = value_func[state]
		temp = 0
		for action in range(env.nA):
			temp = max(temp , sum(1.0*prob*(reward + gamma*value_func[nextState]) 
									for (prob, nextState, reward, is_terminal) in env.P[state][action]))
		value_func[state] = temp
		delta = fabs(old_value - temp)
		if(delta>tol):
			openQueue.put((-delta, entry_count+1, state))
			numStableValues = 0	
		else:
			openQueue.put((0, entry_count + 1, state))
			numStableValues +=1

	# for state in range(env.nS):
	# 	openQueue.put((0., state))

	# for numIterations in range(max_iterations):
	# # while (numStableValues<openQueue.qsize()):
	# 	# numStateEvaluations +=1
	# 	newQueue = Queue.PriorityQueue()
	# 	delta = 0
	# 	for s in range(env.nS):
	# 		priorityValue, state = openQueue.get()
	# 		old_value = value_func[state]
	# 		temp = 0
	# 		for action in range(env.nA):
	# 			temp = max(temp , sum(1.0*prob*(reward + gamma*value_func[nextState]) 
	# 									for (prob, nextState, reward, is_terminal) in env.P[state][action]))
	# 		value_func[state] = temp
	# 		delta = max(delta, fabs(old_value - temp))
	# 		newQueue.put((-fabs(old_value - temp), state))
	# 	openQueue = copy(newQueue)
	# 	if(delta<tol):
	# 		break
	# numStateEvaluations = env.nS * (numIterations+1)
	
	# print(value_func.reshape(int(env.nS**0.5),int(env.nS**0.5)))
	
	_, policy = value_function_to_policy(env, gamma, value_func)

	return policy, numStateEvaluations
	# return np.zeros(env.nS), 0


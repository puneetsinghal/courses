import math
import numpy as np
from copy import copy

# The logProd function takes a vector of numbers in logspace 
# (i.e., x[i] = log p[i]) and returns the product of those numbers in logspace
# (i.e., logProd(x) = log(product_i p[i]))
def logProd(x):
	## Inputs ## 
	# x - 1D numpy ndarray
	
	## Outputs ##
	# log_product - float

	log_product = np.sum(x)

	return log_product

# The NB_XGivenY function takes a training set XTrain and yTrain and
# Beta parameters beta_0 and beta_1, then returns a matrix containing
# MAP estimates of theta_yw for all words w and class labels y
def NB_XGivenY(XTrain, yTrain, beta_0, beta_1):
	## Inputs ## 
	# XTrain - (n by V) numpy ndarray
	# yTrain - 1D numpy ndarray of length n
	# alpha - float
	# beta - float
	
	## Outputs ##
	# D - (2 by V) numpy ndarray

	D = np.zeros([2, XTrain.shape[1]])
	positiveReviews = yTrain==1
	numPositiveReviews = np.sum(yTrain)

	negativeReviews = yTrain==0
	numNegativeReviews = yTrain.size - np.sum(yTrain)

	for i in range(XTrain.shape[1]):
		numAppearancesPositive = np.sum(XTrain[positiveReviews,i])
		numAppearancesNegative = np.sum(XTrain[negativeReviews,i])

		D[0][i] = 1.0*(beta_0 + numAppearancesNegative - 1.)/(beta_0 + beta_1 + numNegativeReviews -2.)
		D[1][i] = 1.0*(beta_0 + numAppearancesPositive - 1.)/(beta_0 + beta_1 + numPositiveReviews -2.)

	return D
	
# The NB_YPrior function takes a set of training labels yTrain and
# returns the prior probability for class label 0
def NB_YPrior(yTrain):
	## Inputs ## 
	# yTrain - 1D numpy ndarray of length n

	## Outputs ##
	# p - float

	p = 1.0 - 1.0*np.sum(yTrain)/yTrain.size
	return p

# The NB_Classify function takes a matrix of MAP estimates for theta_yw,
# the prior probability for class 0, and uses these estimates to classify
# a test set.
def NB_Classify(D, p, XTest):
	## Inputs ## 
	# D - (2 by V) numpy ndarray
	# p - float
	# XTest - (m by V) numpy ndarray
	
	## Outputs ##
	# yHat - 1D numpy ndarray of length m

	yHat = np.ones(XTest.shape[0])
	
	for i in range(XTest.shape[0]):
		XProbability = copy(D[0])
		XProbability[XTest[i]==0] = 1. - XProbability[XTest[i]==0]
		probabilityNegative = logProd(np.log(XProbability)) + np.log(p)

		XProbability = copy(D[1])
		XProbability[XTest[i]==0] = 1. - XProbability[XTest[i]==0]
		probabilityPositive = logProd(np.log(XProbability)) + np.log(1-p)

		# print(probabilityNegative, probabilityPositive)
		if probabilityNegative > probabilityPositive:
			yHat[i] = 0 

	return yHat

# The classificationError function takes two 1D arrays of class labels
# and returns the proportion of entries that disagree
def classificationError(yHat, yTruth):
	## Inputs ## 
	# yHat - 1D numpy ndarray of length m
	# yTruth - 1D numpy ndarray of length m
	
	## Outputs ##
	# error - float

	error = 1.0 - 1.0*np.sum(yHat==yTruth)/yHat.size

	return error

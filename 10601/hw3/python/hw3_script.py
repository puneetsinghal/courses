import os
import csv
import numpy as np
import NB
import matplotlib.pyplot as plt
# Point to data directory here
# By default, we are pointing to '../data/'
data_dir = os.path.join('..','data')

# Read vocabulary into a list
# You will not need the vocabulary for any of the homework questions.
# It is provided for your reference.
with open(os.path.join(data_dir, 'vocabulary.csv'), 'rb') as f:
    reader = csv.reader(f)
    vocabulary = list(x[0] for x in reader)



# XTrain = np.array(([1, 0, 0, 1, 1],[0, 1, 0, 1, 0], [0, 1, 1, 1, 1], [1, 1, 0, 1, 0],[1, 1, 1, 0, 0]))
# yTrain = np.array(([0, 1, 1, 0, 1]))
# XTest = np.array(([1, 0, 1, 0, 1],[0, 1, 0, 0, 0], [0, 1, 1, 1, 1]))
# yTruth = np.array(([ 0, 1, 1]))

# Load numeric data files into numpy arrays
print("Loading XTrain")
XTrain = np.genfromtxt(os.path.join(data_dir, 'XTrain.csv'), delimiter=',')
print("Loading yTrain")
yTrain = np.genfromtxt(os.path.join(data_dir, 'yTrain.csv'), delimiter=',')
print("Loading XTrainSmall")
XTrainSmall = np.genfromtxt(os.path.join(data_dir, 'XTrainsmall.csv'), delimiter=',')
print("Loading yTrainSmall")
yTrainSmall = np.genfromtxt(os.path.join(data_dir, 'yTrainsmall.csv'), delimiter=',')
print("Loading XTest")
XTest = np.genfromtxt(os.path.join(data_dir, 'XTest.csv'), delimiter=',')
print("Loading yTest")
yTruth = np.genfromtxt(os.path.join(data_dir, 'yTest.csv'), delimiter=',')
 
beta_01 = 5
beta_11 = 7

# TODO: Test logProd function, defined in NB.py
error = 0 
for i in range(100):
	x = np.random.uniform(0.1, 10000, 10)
	error += abs(NB.logProd(np.log(x)) - np.sum(np.log(x)))>1e-10
print("\nerror in logProd = {}\n".format(error))


# TODO: Test NB_XGivenY function, defined in NB.py
D = NB.NB_XGivenY(XTrainSmall, yTrainSmall, beta_01, beta_11)
print("value of MAP: \n {} \n".format(D))


# TODO: Test NB_YPrior function, defined in NB.py
p = NB.NB_YPrior(yTrain)
print("The value of yTrain prior = {} \n".format(p))


# TODO: Test NB_Classify function, defined in NB.py
yHat = NB.NB_Classify(D, p, XTest)
print("Test results are: {}".format(yHat))
a = np.sum(yHat)
print("Number of 1 is {} and total numbe is {}\n".format(a, yHat.size))


# TODO: Test classificationError function, defined in NB.py
error = NB.classificationError(yHat, yTruth)
print("Error in test 1 results is: {} \n".format(error))

# TODO: Run experiments outlined in HW2 PDF
# XTrainNew = np.vstack((XTrainSmall, XTrain))
# yTrainNew = np.hstack((yTrainSmall, yTrain))

D1 = NB.NB_XGivenY(XTrain, yTrain, beta_01, beta_11)
p1 = NB.NB_YPrior(yTrain)
yHat1 = NB.NB_Classify(D1, p1, XTest)
error1 = NB.classificationError(yHat1, yTruth)
print("Error in test 2 results is: {} \n".format(error1))

beta_02 = 7
beta_12 = 5
D2 = NB.NB_XGivenY(XTrain, yTrain, beta_02, beta_12)
p2 = NB.NB_YPrior(yTrain)
yHat2 = NB.NB_Classify(D2, p2, XTest)
error2 = NB.classificationError(yHat2, yTruth)
print("Error in test 3 results is: {} \n".format(error2))

plt.figure()

plt.subplot(2, 2, 1)
plt.plot(np.arange(1,XTrain.shape[1]+1,1),D1[0,:])
plt.title('beta_0 = 5, beta_1 = 7')
plt.grid()

plt.subplot(2, 2, 2)
plt.plot(np.arange(1,XTrain.shape[1]+1,1),D1[1,:])
plt.title('beta_0 = 5, beta_1 = 7')
plt.grid()

plt.subplot(2, 2, 3)
plt.plot(np.arange(1,XTrain.shape[1]+1,1),D2[0,:])
plt.title('beta_0 = 7, beta_1 = 5')
plt.grid()

plt.subplot(2, 2, 4)
plt.plot(np.arange(1,XTrain.shape[1]+1,1),D2[1,:])
plt.title('beta_0 = 7, beta_1 = 5')
plt.grid()

a = D1[1,:]>0.1 and D1[1,:]<0.3
b = D2[1,:]>0.1 and D2[1,:]<0.3

print(np.sum(a))
print(np.sum(b))

plt.show()
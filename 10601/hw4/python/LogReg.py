#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
from copy import copy
class LogisticRegression:

    def __init__(self, alpha = 0.01, regLambda=0.01, epsilon=0.0001, maxNumIters = 50):
        '''
        Initializes Parameters of the  Logistic Regression Model
        '''
        self.alpha = alpha
        self.regLambda = regLambda
        self.epsilon = epsilon
        self.maxNumIters = maxNumIters
    
    
    def calculateGradient(self, weight, X, Y, regLambda):
        '''
        Computes the gradient of the objective function
        Arguments:
        
            X is a n-by-(d+1) numpy matrix
            Y is an n-by-1 dimensional numpy matrix
            weight is (d+1)-by-1 dimensional numpy matrix
            regLambda is the scalar regularization constant
        Returns:
            the gradient, an (d+1)-by-1 dimensional numpy matrix
        '''
        # hypothesis = 1.0/(1+np.exp(-np.dot(weight.T, X.T)))
        Z = np.dot(X, weight)
        hypothesis = self.sigmoid(Z)
        Gradient = np.dot(X.T, (hypothesis - Y)) + regLambda*weight
        Gradient[0,0] = Gradient[0,0] - regLambda*weight[0,0]
        return Gradient

    def sigmoid(self, Z):
        '''
        Computes the Sigmoid Function  
        Arguments:
            A n-by-1 dimensional numpy matrix
        Returns:
            A n-by-1 dimensional numpy matrix
       
        '''
        # sigmoid = Z/np.linalg.norm(Z)
        sigmoid = 1.0/(1.0+np.exp(-Z))
        return sigmoid

    def update_weight(self,X,Y,weight):
        '''
        Updates the weight vector.
        Arguments:
            X is a n-by-(d+1) numpy matrix
            Y is an n-by-1 dimensional numpy matrix
            weight is a d+1-by-1 dimensional numpy matrix
        Returns:
            updated weight vector : (d+1)-by-1 dimensional numpy matrix
        '''
        Gradient = self.calculateGradient(weight,X,Y,self.regLambda)
        new_weight = weight - self.alpha*Gradient
        return new_weight
    
    def check_conv(self,weight,new_weight,epsilon):
        '''
        Convergence Based on Tolerance Values
        Arguments:
            weight is a (d+1)-by-1 dimensional numpy matrix
            new_weights is a (d+1)-by-1 dimensional numpy matrix
            epsilon is the Tolerance value we check against
        Return : 
            True if the weights have converged, otherwise False

        '''
        weightDifference = np.linalg.norm(weight - new_weight)
        if (weightDifference <= epsilon):
            return True
        return False
        
    def train(self,X,Y):
        '''
        Trains the model
        Arguments:
            X is a n-by-d numpy matrix
            Y is an n-by-1 dimensional numpy matrix
        Return:
            Updated Weights Vector: (d+1)-by-1 dimensional numpy matrix
        '''
        # Read Data
        n,d = X.shape
        
        #Add 1's column
        X = np.c_[np.ones((n,1)), X]
        self.weight = self.new_weight = np.zeros((d+1,1))

        for i in range(self.maxNumIters):
            self.new_weight = self.update_weight(X,Y,self.weight)
            if (self.check_conv(self.weight,self.new_weight,self.epsilon)):
                return self.weight
            else:
                self.weight = copy(self.new_weight)

        return self.weight

    def predict_label(self, X, weight):
        '''
        Used the model to predict values for each instance in X
        Arguments:
            X is a n-by-d numpy matrix
            weight is a d+1-by-1 dimensional matrix
        Returns:
            an n-by-1 dimensional matrix of the predictions 0 or 1
        '''
        #data
        n=X.shape[0]
        #Add 1's column
        X = np.c_[np.ones((n,1)), X]
        
        temp = np.dot(X, weight)
        probability = self.sigmoid(-temp)
        result = np.ones((n,1), dtype=np.int)
        result[probability[:,0]>0.5] = 0

        return result
    
    def calculateAccuracy (self, Y_predict, Y_test):
        '''
        Computes the Accuracy of the model
        Arguments:
            Y_predict is a n-by-1 dimensional matrix (Predicted Labels)
            Y_test is a n-by-1 dimensional matrix (True Labels )
        Returns:
            Scalar value for accuracy in the range of 0 - 100 %
        '''
        Accuracy = 100.0*(np.sum(Y_predict==Y_test))/Y_predict.size
        return Accuracy
    
        
import numpy as np
import pickle
import matplotlib.pyplot as plt

numSamples = 100
costResults = pickle.load(open('costResults.p', 'rb'))
accuracyResults = pickle.load(open('accuracyResults.p', 'rb'))

plt.figure()
plt.plot(np.linspace(1, numSamples-1, numSamples-1), costResults[0,0:numSamples-1], color='k', linewidth=3, label='train cost')
plt.plot(np.linspace(1, numSamples-1, numSamples-1), costResults[1,0:numSamples-1], color='r', linewidth=3,  linestyle=':', label='test cost')
plt.title('cost comparison')
plt.legend()
plt.xlabel('iteration/100')
plt.ylabel('cost')
plt.grid()
plt.show()

plt.figure()
plt.plot(np.linspace(1, numSamples-1, numSamples-1), accuracyResults[0,0:numSamples-1], color='k', linewidth=3, label='train accuracy')
plt.plot(np.linspace(1, numSamples-1, numSamples-1), accuracyResults[1,0:numSamples-1], color='r', linewidth=3,  linestyle=':', label='test accuracy')
plt.title('accuracy comparison')
plt.legend()
plt.xlabel('iteration/100')
plt.ylabel('accuracy')
plt.grid()
plt.show()
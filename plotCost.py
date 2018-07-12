import matplotlib.pyplot as plt
import numpy as np

X = np.loadtxt('./path/cost.txt')

if X.ndim == 1 or X.shape[0] < 2:
    print('No cost date.')
    exit()

fig = plt.figure(1)
ax1 = fig.add_subplot(1, 2, 1)
ax1.plot(X[:,0], X[:,1], 'k-', label='Cost')
ax1.set_xlabel('Time')
ax1.set_ylabel('Cost')
ax1.set_ylim([0, np.max(X[:,1])])

ax2 = fig.add_subplot(1, 2, 2)
ax2.plot(X[:,0], X[:,2], 'k-', label='Probability')
ax2.set_xlabel('Time')
ax2.set_ylabel('Probability')
ax2.set_ylim([0, np.max(X[:,2])])

plt.show()
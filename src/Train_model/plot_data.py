import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
#from matplotlib import cm
#from matplotlib.ticker import LinearLocator, FormatStrFormatter
#from matplotlib import rcParams
import numpy.ma as ma
from numpy import genfromtxt
fig = plt.figure()
ax = fig.gca(projection='3d')

data = np.genfromtxt('/home/vibek/Human_intention/OutputData/Trajectory_RightHand_1.csv', delimiter=',')
x = data[:,0]
y = data[:,1]
z = data[:,2]
ax.scatter(x, z, y, c='r', marker='o')
ax.plot(x, z, y, label='Motion trajectory of left_hand')
ax.legend()
ax.set_xlabel('X (m)')
ax.set_ylabel('Z (m)')
ax.set_zlabel('Y (m)')
plt.show()


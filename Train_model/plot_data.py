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

data = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Prediction_accuracy/Trajectory_LeftHand_2016-12-12_14_35_49.csv', delimiter=',')
data1 = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Prediction_accuracy/Trajectory_LeftHand_2016-12-12_14_35_49.csv', delimiter=',')
x = data[:,0]
y = data[:,1]
z = data[:,2]
x1 = data1[:,0]
y1 = data1[:,1]
z1 = data1[:,2]
ax.scatter(x, z, y, c='r', marker='o')
ax.plot(x, z, y, label='Observed trajectory')
ax.plot(x1, z1, y1, c='k', label='Predicted trajectory')
ax.legend(prop={'size':15,'weight':'bold' })
plt.title("Prediction plot", size=25,fontweight='bold')
ax.set_xlabel('X (m)', size =15, fontweight='bold')
ax.set_ylabel('Z (m)', size =15, fontweight='bold')
ax.set_zlabel('Y (m)', size =15, fontweight='bold')
plt.tick_params(labelsize=10)

plt.show()


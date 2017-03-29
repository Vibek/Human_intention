"Plot Prediction data"

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
#from matplotlib.mlab import griddata
from scipy.interpolate import griddata
from matplotlib import cm
#from matplotlib.ticker import LinearLocator, FormatStrFormatter
from numpy import genfromtxt

  
fig = plt.figure()

ax =  fig.gca(projection='3d')
ax1 = fig.gca(projection='3d')
ax3 = fig.gca(projection='3d')
ax4 = fig.gca(projection='3d')

ax.set_title('Drinking', fontsize=14, fontweight='bold', color='red')

data = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Potential_map/predict_drink_w.csv', delimiter=',')
x = data[:,0]
y = data[:,1]
z = data[:,2]

data3 = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Potential_map/RightHand_drink_w.csv', delimiter=',')
x3 = data3[:,0]
y3 = data3[:,1]
z3 = data3[:,2]
   
data2 = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Potential_map/LeftHand_drink_w.csv', delimiter=',')
x2 = data2[:,0]
y2 = data2[:,1]
z2 = data2[:,2]
   
xi = np.linspace(x2.min(),x2.max())
yi = np.linspace(y2.min(),y2.max())

# VERY IMPORTANT, to tell matplotlib how is your data organized
zi = griddata((x2, y2), z2, (xi[None,:], yi[:,None]), method='cubic')

ii = -1 #index of variable or spot point
xm = 0.01 ##boxe size left corner
xp = 0.02 #box size right conner
ym = 0.04
yp = 0.004
zm = 0.004
zp = 0.03
bd = 5
bs = 0.1

xx = np.linspace(x[ii] - xm, x[ii] + xp, bd)
yy = np.linspace(y[ii] - ym, y[ii] + yp, bd)
zz = np.linspace(z[ii] - zm, z[ii] + zp, bd)
   
#xx = np.linspace(x[ii] - bs, x[ii] + bs, bd)
#yy = np.linspace(y[ii] - bs, y[ii] + bs, bd)
#zz = np.linspace(z[ii] - bs, z[ii] + bs, bd)

# Make the grid    
X,Y,Z = np.meshgrid(xx, yy, zz)

u = -(X - x[ii])
v = -(Y - y[ii])
w = -(Z - z[ii])
   
#ax.scatter(x2, z2, y2, c='g', marker='*', label = 'Heat-map')
ax.scatter(x, y, z, c='r', marker='o')
ax.plot(x, y, z, c='k', linewidth = 2,label='Predicted trajectory')
ax1.scatter(x, y, z, c='b', marker='+')
ax1.plot(x2, y2, z2, c='r', linewidth = 1.5, label='right hand trajectory')
ax3.plot(x3, y3, z3, c='g', label='left hand trajectory')

ax4.contourf(xi, yi, zi, zdir='z', cmap=cm.gist_rainbow)

ax4.quiver(X,Y,Z,u,v,w, length=0.01, normalize=True)

ax.legend(fontsize= 15)
ax.set_xlabel('X (m)', fontsize=10, fontweight='bold')
ax.set_ylabel('Z (m)', fontsize=10, fontweight='bold')
ax.set_zlabel('Y (m)', fontsize=10, fontweight='bold')
plt.show()

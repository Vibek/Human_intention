"Plot Prediction data"

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata
from matplotlib import cm
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
   

data1 = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Potential_map/predict_drink_contour.csv', delimiter=',')
x1 = data1[:,0]
y1 = data1[:,1]
z1 = data1[:,2]
   
xi = np.linspace(x1.min(),x1.max())
yi = np.linspace(y1.min(),y1.max())

# VERY IMPORTANT, to tell matplotlib how is your data organized
zi = griddata((x1, y1), z1, (xi[None,:], yi[:,None]), method='cubic')

ii = -1 #index of variable or spot point
xm = 0.01 ##boxe size left corner
xp = 0.02 #box size right conner
ym = 0.04
yp = 0.004
zm = 0.004
zp = 0.03
bd = 5
bs = 0.1

#xx = np.linspace(x1[ii] - xm, x1[ii] + xp, bd)
#yy = np.linspace(y1[ii] - ym, y1[ii] + yp, bd)
#zz = np.linspace(z1[ii] - zm, z1[ii] + zp, bd)
   
xx = np.linspace(x1[ii] - bs, x1[ii] + bs, bd)
yy = np.linspace(y1[ii] - bs, y1[ii] + bs, bd)
zz = np.linspace(z1[ii] - bs, z1[ii] + bs, bd)

# Make the grid    
#X, Y, Z = np.meshgrid(xx, yy, zz)

#u = -(X - x1[ii])
#v = -(Y - y1[ii])
#w = -(Z - z1[ii])
   
ax.plot(x,   y,  z,  c='r', linewidth = 3,   label='Predicted trajectory')
ax1.plot(x2, y2, z2, c='K', linewidth = 1.5, label='right hand trajectory')
ax3.plot(x3, y3, z3, c='g', linewidth = 1.5, label='left hand trajectory')

ax4.contourf(xi, yi, zi, zdir='z', cmap=cm.gist_rainbow)

#ax4.quiver(X,Y,Z,u,v,w, length=0.01, normalize=True)

ax.legend()
ax.set_xlabel('X (m)', fontsize=10, fontweight='bold')
ax.set_ylabel('Z (m)', fontsize=10, fontweight='bold')
ax.set_zlabel('Y (m)', fontsize=10, fontweight='bold')
plt.show()

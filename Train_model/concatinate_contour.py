"""Contours with overlap control.
This script generates widely spaced black contours, with a gray patch in a
subregion where more finely-spaced colored contours are visible.  The gray
patch covers the black contours.
"""

from __future__ import division
import matplotlib
import numpy as np
import matplotlib.cm as cm
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import scipy.interpolate
import time
import scipy.stats as stats
import numpy.random as random
import scipy.interpolate

plt.close('all')
# plot with various axes scales
plt.figure(1)

with open('/home/vibek/Human_intention/src/Train_model/Potential_map/RightHand_drink.csv','r') as in_file, open('/home/vibek/Human_intention/src/Train_model/Potential_map/RightHand_drink_w.csv','w') as out_file:
 seen = set()
 for line in in_file:
	if line in seen: continue
	seen.add(line)
	out_file.write(line)

matplotlib.rcParams['xtick.direction'] = 'out'
matplotlib.rcParams['ytick.direction'] = 'out'

data = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Potential_map/RightHand_drink_w.csv', delimiter=',')
X = data[:,0]
Y = data[:,1]
Z = data[:,2]
xi = np.linspace(X.min(), X.max(), 8, endpoint=True)
yi = np.linspace(Y.min(), Y.max(), 8, endpoint=False)
xi, yi = np.meshgrid(xi, yi)   
rbf = scipy.interpolate.Rbf(X,Y,Z, function='linear')
zi = rbf(xi,yi)
# I control the range of my colorbar by removing data 
# outside of my range of interest
plt.contourf(xi, yi, zi, cmap=plt.cm.hot, aspect='auto')
plt.colorbar()  
cp = plt.contour(xi, yi, zi)
plt.clabel(cp, inline=True, fontsize=12)
plt.title('Potential Map (Drinking)', size=20)
plt.xlabel("X", size=15)
plt.ylabel("Y", size=15)
plt.tick_params(labelsize=15)
plt.show()

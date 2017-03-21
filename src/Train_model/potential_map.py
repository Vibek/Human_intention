"Isoclines map for the predicted trajectory"
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

plt.close('all')
# plot with various axes scales
plt.figure(1)

#with open('/home/vibek/Human_intention/src/Train_model/chair_predict.csv','r') as in_file, open('/home/vibek/Human_intention/src/Train_model/chair_predict_w.csv','w') as out_file:
# seen = set()
# for line in in_file:
#	if line in seen: continue
#	seen.add(line)
#	out_file.write(line)

matplotlib.rcParams['xtick.direction'] = 'out'
matplotlib.rcParams['ytick.direction'] = 'out'

data = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/door_left_w.csv', delimiter=',')
X = data[:,0]
Y = data[:,1]
Z = data[:,2]
xi = np.linspace(X.min(), X.max(), 8, endpoint=True)
yi = np.linspace(Y.min(), Y.max(), 8, endpoint=False)
xi, yi = np.meshgrid(xi, yi)   

# Interpolate
rbf = scipy.interpolate.Rbf(X, Y, Z, function='thin_plate')
zi = rbf(xi, yi)
plt.subplot(221)
plt.imshow(zi, vmin=Z.min(), vmax=Z.max(), origin='upper',
         extent=[X.min(), X.max(), Y.min(), Y.max()], cmap=plt.cm.hot, aspect='auto')
cp = plt.contour(xi, yi, zi)
plt.clabel(cp, inline=True, fontsize=10)
plt.scatter(X, Y, c=Z)
plt.colorbar()
plt.title("Contour Plot (probability)", size=15)
plt.xlabel("X (m)", size=12)
plt.ylabel("Y (m)", size=12)

"Q-Q plot X"

X.sort()
norm=random.normal(0,2,len(X))
norm.sort()
#plt.figure(figsize=(8,6),facecolor='1.0')
plt.subplot(222)
plt.plot(norm,X,"o")
z = np.polyfit(norm, X, 1)
p = np.poly1d(z)
plt.plot(norm,p(norm),"k--", linewidth=2)
plt.title("Q-Q plot (X-value)", size=15)
plt.xlabel("Expected quantiles", size=12)
plt.ylabel("Expreimental quantiles", size=12)
plt.tick_params(labelsize=10)


"Q-Q plot Y"

Y.sort()
norm=random.normal(0,2,len(Y))
norm.sort()
#plt.figure(figsize=(8,6),facecolor='1.0')
plt.subplot(223)
plt.plot(norm,Y,"o")
z = np.polyfit(norm, Y, 1)
p = np.poly1d(z)
plt.plot(norm,p(norm),"k--", linewidth=2)
plt.title("Q-Q plot (Y-value)", size=15)
plt.xlabel("Expected quantiles", size=12)
plt.ylabel("Expreimental quantiles", size=12)
plt.tick_params(labelsize=10)


"Q-Q plot Z"

Z.sort()
norm=random.normal(0,2,len(Z))
norm.sort()
#plt.figure(figsize=(8,6),facecolor='1.0')
plt.subplot(224)
plt.plot(norm,Z,"o")
z = np.polyfit(norm, Z, 1)
p = np.poly1d(z)
plt.plot(norm,p(norm),"k--", linewidth=2)
plt.title("Q-Q plot (Z-value)", size=15)
plt.xlabel("Expected quantiles", size=12)
plt.ylabel("Expreimental quantiles", size=12)
plt.tick_params(labelsize=10)

#k = stats.shapiro(X)
#plt.plot(k)
#if (k<0.05):
#	print "Variable not normal"
#else:
#	print "Variable is normal"

plt.show()

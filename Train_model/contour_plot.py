"""
Comparison of griddata and tricontour for an unstructured triangular grid.
"""
from __future__ import print_function
import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import numpy.random as rnd
import matplotlib.mlab as mlab
import time

with open('/home/vibek/Human_intention/src/Train_model/chair_predict.csv','r') as in_file, open('/home/vibek/Human_intention/src/Train_model/chair_predict_w.csv','w') as out_file:
 seen = set()
 for line in in_file:
	if line in seen: continue
	seen.add(line)
	out_file.write(line)
rnd.seed(0)
npts = 200
ngridx = 100
ngridy = 200
data = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/chair_right_w.csv', delimiter=',')

x = data[:,0]
y = data[:,1]
z = data[:,2]

#z = x*np.exp(-x**2 - y**2)

# tricontour.
start = time.clock()
#plt.subplot(212)
triang = tri.Triangulation(x, y)
plt.triplot(x,y, data, 'go-',lw=1.0)
plt.tricontour(x, y, z, 35, linewidths=0.5, colors='k')
plt.tricontourf(x, y, z, 35)
               # norm=plt.Normalize(vmax=abs(z).max(), vmin=-abs(z).max()))
plt.colorbar()
plt.plot(x, y, 'ko', ms=3)
plt.xlim(0, 0.4)
plt.ylim(-0.3, 0)
plt.title('tricontour (%d points)' % npts)
print('tricontour seconds: %f' % (time.clock() - start))

#plt.subplots_adjust(hspace=0.5)

plt.show()

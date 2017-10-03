import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
#from matplotlib import cm
#from matplotlib.ticker import LinearLocator, FormatStrFormatter
#from matplotlib import rcParams
import numpy.ma as ma
from numpy import genfromtxt
from scipy import interpolate

rownums = [];
colnums =[];
aa = [];

for i in range(0,1):
	name = '/home/vibek/Human_intention/OutputData/Trajectory_RightHand_0.csv'
	aa.append(np.loadtxt(name, delimiter=','))
        rownums.append(aa[i].shape[0]);
	colnums.append(aa[i].shape[1]);
maxrownum = max(rownums);
sums = np.ndarray((maxrownum,aa[0].shape[1]));
x = np.genfromtxt('/home/vibek/Human_intention/OutputData/Trajectory_RightHand_0.csv', usecols=(0,), delimiter=',',dtype=None)
y = np.genfromtxt('/home/vibek/Human_intention/OutputData/Trajectory_RightHand_0.csv', usecols=(1,), delimiter=',',dtype=None)
z = np.genfromtxt('/home/vibek/Human_intention/OutputData/Trajectory_RightHand_0.csv', usecols=(2,), delimiter=',',dtype=None)
t = range(0, maxrownum)
fx = interpolate.interp1d(t,x, kind='cubic')
fy = interpolate.interp1d(t,y, kind='cubic')
fz = interpolate.interp1d(t,z, kind='cubic')
tnew = np.arange(0,maxrownum-1,0.1)
xnew = fx(tnew)
ynew = fy(tnew)
znew = fz(tnew)
np.savetxt('colums_data_right.csv', xnew);
#print (xnew)
for i in range(0,1):
	plt.plot(range(0, aa[i].shape[0]), aa[i].transpose()[0], 'o', range(0, aa[i].shape[0]), aa[i].transpose()[1], 'o', range(0, aa[i].shape[0]), aa[i].transpose()[2], 'o', tnew, xnew, '-', tnew, ynew, '-', tnew, znew, '-')
	plt.show()

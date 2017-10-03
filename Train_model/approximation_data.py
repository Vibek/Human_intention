import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
aa = [];
rownums = [];
colnums =[];

for i in range(0,10):
	name = '/home/vibek/Human_intention/OutputData/Trajectory_RightHand_'+str(i)+'.csv'
	aa.append(np.loadtxt(name, delimiter=','))
        rownums.append(aa[i].shape[0]);
	colnums.append(aa[i].shape[1]);
	bx = aa[i].transpose()[0];
	by = aa[i].transpose()[1];
	bz = aa[i].transpose()[2];
	#maxrownum = max(rownums);
	#tr = range(0, aa[i].shape[0])
	#funx = interpolate.interp1d(tr,aa[i].transpose()[0], kind='cubic')
	#funx = interpolate.interp1d(tr,aa[i].transpose()[1], kind='cubic')
	#funx = interpolate.interp1d(tr,aa[i].transpose()[2], kind='cubic')
	#trnew = np.arange(0, aa[i].shape[0]-1, 0.1)
	#bxnew = funx(trnew)
	#bynew = funx(trnew)
	#bznew = funx(trnew)
	#np.savetxt(name, bxnew)
	#np.savetxt(name, bynew)
	#np.savetxt(name, bznew)
	
maxrownum = max(rownums);
sums = np.ndarray((maxrownum,aa[0].shape[1]));
	
for ri in range(0,sums.shape[0]):
	for ci in range(0,sums.shape[1]):
		sums[ri][ci] = 0.0;
		for fi in range(0,10):
			if(ri < aa[fi].shape[0]):
				sums[ri][ci] += aa[fi][ri][ci]/10.0;
np.savetxt('sums_right.csv', sums);
x = np.genfromtxt('sums_right.csv', names=['x'], usecols=(0,), delimiter=' ',dtype=None)
y = np.genfromtxt('sums_right.csv', names=['y'], usecols=(1,), delimiter=' ',dtype=None)
z = np.genfromtxt('sums_right.csv', names=['z'], usecols=(2,), delimiter=' ',dtype=None)
t = range(0, maxrownum)
fx = interpolate.interp1d(t,x, kind='cubic')
fy = interpolate.interp1d(t,y, kind='cubic')
fz = interpolate.interp1d(t,z, kind='cubic')
tnew = np.arange(0,maxrownum-1,0.1)
xnew = fx(tnew)
np.savetxt('colums_data_right.csv', xnew);
#print (xnew)
for i in range(0,10):
	plt.plot(range(0, aa[i].shape[0]), aa[i].transpose()[0], 'o', tnew, xnew, '-', t, x, '*')
	plt.show()

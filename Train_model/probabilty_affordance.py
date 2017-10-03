"Evaluate the prediction Value"
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
from StringIO import StringIO

plt.close('all')
# plot with various axes scales
plt.figure(1)
N = np.random.uniform(0,1,100)
#print(N)
np.savetxt('random.csv', N);
index =0
data = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/prediction.csv', delimiter=',', dtype='|S12')

#collect the row number and data with the name "Openebale"#
listNum=[];listRow=[];
for num, row in enumerate(data):
	if row[0] == "Graspable":
		listNum.append(num);
		listRow.append(row[1]);
		#print num, row[1]
		
#collect the row number and data with the name "Graspable"#
listNum1=[];listRow1=[];
for num1, row1 in enumerate(data):
	if row1[0] == "Openable":
		listNum1.append(num1);
		listRow1.append(row1[1]);
		#print num1, row1[1]

#collect the row number and data with the name "Reachable"#
listNum2=[];listRow2=[];
for num2, row2 in enumerate(data):
	if row2[0] == "Reachable":
		listNum2.append(num2);
		listRow2.append(row2[1]);
		#print num2, row2[1]

plt.plot(listNum1, listRow1, 'g+:', label='Openable') 
plt.plot(listNum, listRow, 'b+:', label='Graspbale')
plt.plot(listNum2, listRow2, 'r+:', label='Reachable')
plt.legend(loc='upper left' )
plt.title("Prediction plot", size=15)
plt.xlabel("Number of Frames", size=12)
plt.ylabel("Probability", size=12)
plt.tick_params(labelsize=10)
plt.show()












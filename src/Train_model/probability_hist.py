import numpy as np
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

plt.close('all')
# plot with various axes scales
plt.figure(1)

#read the data
data_R = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_R_ch.csv', delimiter=',')
data_G = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_G_ch.csv', delimiter=',')
data_O = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_P_ch.csv', delimiter=',')

# use the number of bins (means Frequecy)
numbins=10

#remove all the NAN values in the data-set
data_R = data_R[~np.isnan(data_R)]
data_G = data_G[~np.isnan(data_G)]
data_O = data_O[~np.isnan(data_O)]

#plot the histogram of each data-set in one plot
plt.hist([data_R, data_G, data_O], numbins, normed=True, color=['red','green','blue'], alpha=1, label=['Reachable','Graspable','Pullable'])
plt.legend(loc='upper left' )
plt.title('Probability Graph of Affrodances (Pulling a Chair) ', size=15)
plt.xlabel("Probability value (0-1)", size=12)
plt.ylabel("Frequency", size=12)
plt.tick_params(labelsize=10)
plt.show()

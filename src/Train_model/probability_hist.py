import numpy as np
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

plt.close('all')
# plot with various axes scales
plt.figure(1)

#read the data
data_G_ch = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_G_ch.csv', delimiter=',')
data_R_ch = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_R_ch.csv', delimiter=',')
data_P_ch = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_P_ch.csv', delimiter=',')
#data_M = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_M.csv', delimiter=',')
# use the number of bins (means Frequecy)
numbins=12

#remove all the NAN values in the data-set
data_G_ch =  data_G_ch[~np.isnan(data_G_ch)]
data_R_ch =  data_R_ch[~np.isnan(data_R_ch)]
data_P_ch =  data_P_ch[~np.isnan(data_P_ch)]
#data_M =  data_M[~np.isnan(data_M)]

#plot the histogram of each data-set in one plot
plt.hist([data_G_ch, data_R_ch, data_P_ch], numbins, normed = False, color=['red','green','blue'], alpha=1, label=['Reachable','Graspable','Pullable'], orientation='horizontal')
#plt.set_ylim([0, 1])
plt.legend(loc='lower right' )
plt.title('Probability Graph of Affrodances (Pulling a Chair) ', size=15)
plt.xlabel("Frequency", size=12)
plt.ylabel("Probability values (0-1)", size=12)
plt.tick_params(labelsize=10)
plt.show()

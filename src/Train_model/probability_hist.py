import numpy as np
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

plt.close('all')
# plot with various axes scales
plt.figure(1)

#read the data
data_D = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_D.csv', delimiter=',')
data_P = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_P.csv', delimiter=',')
data_Pa = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_Pa.csv', delimiter=',')
data_M = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_M.csv', delimiter=',')
# use the number of bins (means Frequecy)
numbins=12

#remove all the NAN values in the data-set
data_D =  data_D[~np.isnan(data_D)]
data_P =  data_P[~np.isnan(data_P)]
data_Pa = data_Pa[~np.isnan(data_Pa)]
data_M =  data_M[~np.isnan(data_M)]

#plot the histogram of each data-set in one plot
plt.hist([data_D, data_P, data_Pa, data_M], numbins, normed=True, color=['red','green','blue', 'deeppink'], alpha=1, label=['Drinkable','Pourable','Placeable', 'Moveable'])
plt.legend(loc='upper left' )
plt.title('Probability Graph of Affrodances (Drinking Water from a Bottle) ', size=15)
plt.xlabel("Probability value (0-1)", size=12)
plt.ylabel("Frequency", size=12)
plt.tick_params(labelsize=10)
plt.show()

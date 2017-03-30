import numpy as np
#from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
#from scipy.interpolate import griddata
import matplotlib.mlab as mlab

#plt.close('all')
# plot with various axes scales
#plt.figure(1)

np.random.seed(0)


#read the data
data_G_ch = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_D.csv', delimiter=',')
data_R_ch = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_P.csv', delimiter=',')
data_P_ch = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_Pa.csv', delimiter=',')
data_M = np.genfromtxt('/home/vibek/Human_intention/src/Train_model/Afforance_Probability/data_M.csv', delimiter=',')
# use the number of bins (means Frequecy)
numbins=50

#remove all the NAN values in the data-set
data_G_ch =  data_G_ch[~np.isnan(data_G_ch)]
data_R_ch =  data_R_ch[~np.isnan(data_R_ch)]
data_P_ch =  data_P_ch[~np.isnan(data_P_ch)]
data_M =  data_M[~np.isnan(data_M)]

#plot the histogram of each data-set in one plot
n, numbins, patches = plt.hist([data_G_ch, data_R_ch, data_P_ch, data_M], numbins, color=['red','green','blue','hotpink'], alpha=1, label=['Drinkable','Placeable','Pourable', 'Moveable'])


plt.legend(loc='upper left' )
plt.xlabel("Input data (0-irrelevant, 1-relevent)", size=12)
plt.ylabel("Frequency (number of times occures)", size=12)
plt.title('Histogram plot of a test sample (Drinking Water from a Bottle)', size=15)
#plt.tick_params(labelsize=10)


plt.show()

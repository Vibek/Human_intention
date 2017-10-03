import numpy as np
import matplotlib.pyplot as plt

conf_arr = np.array([[0.96,0.08,0,0,0,0,0,0,0], 
            [0.02,1.00,0,0,0,0,0,0,0], 
            [0,0.02,0.99,0,0,0,0,0,0],
	    [0,0,0,0.99,0,0,0,0,0],
	    [0,0,0.012,0,0.95,0,0,0,0],
            [0,0,0,0,0,0.954,0,0,0],
	    [0,0,0,0,0,0,0.97,0,0],
	    [0,0,0,0,0,0,0.02,0.961,0],
	    [0,0,0,0,0,0,0,0,0.939]])

norm_conf = []
for i in conf_arr:
    a = 0
    tmp_arr = []
    a = sum(i, 0)
    for j in i:
        tmp_arr.append(float(j)/float(a))
    norm_conf.append(tmp_arr)

fig = plt.figure()
plt.clf()
ax = fig.add_subplot(111)
ax.set_aspect(1)
res = ax.imshow(np.array(norm_conf), cmap=plt.cm.Blues, 
                interpolation='nearest')

width, height = conf_arr.shape

for x in xrange(width):
    for y in xrange(height):
        ax.annotate(str(conf_arr[x][y]), xy=(y, x), 
                    horizontalalignment='center',
                    verticalalignment='center')

cb = fig.colorbar(res)
alphabet = ['10','15', '20', '25', '30', '35', '40', '45', '50']
plt.xticks(range(width), alphabet[:width], rotation=25)
plt.yticks(range(height), alphabet[:height])
plt.ylabel('Actual value (degree)')
plt.xlabel('Observed value (degree)')
plt.title('Accuracy measurement of QR-landmark orientataion taking into account keystone correction')
plt.show()

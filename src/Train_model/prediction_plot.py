"Plot Prediction data"

    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib import cm
    import matplotlib.pyplot as plt
    from matplotlib.mlab import griddata
    from scipy.interpolate import griddata
    import numpy as np
    from matplotlib import cm
    from matplotlib.ticker import LinearLocator, FormatStrFormatter
    from matplotlib import rcParams
    from numpy import genfromtxt
    from scipy import interpolate
    from numpy.random import uniform, seed
    import numpy.ma as ma
  
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax1 = fig.gca(projection='3d')
    ax3 = fig.gca(projection='3d')
    ax2 = fig.gca(projection='3d')
    ax4 = fig.gca(projection='3d')
    ax.set_title('Pouring', fontsize=14, fontweight='bold', color='red')
    data = np.genfromtxt('/home/vibek/Human_intention/OutputData/Chair_left.csv', delimiter=',')
    x = data[:,0]
    y = data[:,1]
    z = data[:,2]

    data3 = np.genfromtxt('/home/vibek/Human_intention/OutputData/chair_right.csv', delimiter=',')
    x3 = data3[:,0]
    y3 = data3[:,1]
    z3 = data3[:,2]
   
    data2 = np.genfromtxt('/home/vibek/Human_intention/OutputData/chair_right.csv', delimiter=',')\n",
    x2 = data2[:,0]
    y2 = data2[:,1]
    z2 = data2[:,2]
   
    xi = np.linspace(x2.min(),x2.max())
    yi = np.linspace(y2.min(),y2.max())
    # VERY IMPORTANT, to tell matplotlib how is your data organized
    zi = griddata((x2, y2), z2, (xi[None,:], yi[:,None]), method='cubic')

    ii = -1 #index of variable or spot point
    xm = 0.01 ##boxe size left corner
    xp = 0.02 #box size right conner
    ym = 0.04
    yp = 0.004
    zm = 0.004
    zp = 0.03
    bd = 5
    bs = 0.1
    "#xx = np.linspace(x[ii] - xm, x[ii] + xp, bd)\n",
    "#yy = np.linspace(y[ii] - ym, y[ii] + yp, bd)\n",
    "#zz = np.linspace(z[ii] - zm, z[ii] + zp, bd)\n",
    "\n",
    xx = np.linspace(x[ii] - bs, x[ii] + bs, bd)
    yy = np.linspace(y[ii] - bs, y[ii] + bs, bd)
    zz = np.linspace(z[ii] - bs, z[ii] + bs, bd)
    
    (X,Y,Z) = np.meshgrid(xx,yy,zz)
    u = -(X - x[ii])
    v = -(Y - y[ii])
    w = -(Z - z[ii])
   
    #ax.scatter(x2, z2, y2, c='g', marker='*', label = 'Heat-map')
    ax.scatter(x, y, z, c='r', marker='o')
    ax.plot(x, y, z, c='k', linewidth = 2,label='Predicted trajectory')
    ax1.scatter(x, y, z, c='b', marker='+')
    ax1.plot(x1, y1, z1, c='r', linewidth = 1.5, label='Observed trajectory')
    ax3.plot(x3, y3, z3, c='g', label='right trajectory')
    ax2.plot(x4, y4, z4, c='b', label='right trajectory')
    ax4.contourf(xi, yi, zi, zdir='z', cmap=cm.gist_rainbow)
    ax4.quiver(X,Y,Z,u,v,w, length=0.01)
    ax.legend(fontsize= 15)
    ax.set_xlabel('X (m)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Z (m)', fontsize=10, fontweight='bold')
    ax.set_zlabel('Y (m)', fontsize=10, fontweight='bold')
    plt.show()"

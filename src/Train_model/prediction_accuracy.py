import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

x =[0.33, 0.51, 0.63, 0.59, 0.75, 0.72, 0.81];
x1=[0.14, 0.21, 0.18, 0.25, 0.30, 0.28, 0.28];
x2=[0.14, 0.14, 0.31, 0.33, 0.41, 0.47, 0.56];
x3=[0.12, 0.12, 0.12, 0.24, 0.27, 0.35, 0.44];
y =[50, 100, 150, 200, 250, 300, 350];

x1 =[0.29, 0.38, 0.51, 0.58, 0.53, 0.70, 0.75];
x11=[0.19, 0.27, 0.22, 0.28, 0.30, 0.26, 0.26];
x21=[0.17, 0.22, 0.25, 0.25, 0.33, 0.41, 0.46];
x31=[0.17, 0.17, 0.30, 0.30, 0.30, 0.32, 0.35];

x2 =[0.41, 0.43, 0.61, 0.63, 0.79, 0.83, 0.83];
x21=[0.20, 0.37, 0.44, 0.38, 0.38, 0.47, 0.58];
x22=[0.19, 0.19, 0.42, 0.39, 0.36, 0.48, 0.64];
x23=[0.17, 0.22, 0.25, 0.29, 0.26, 0.30, 0.34];

x3 =[0.36, 0.44, 0.56, 0.53, 0.71, 0.79, 0.85];
x31=[0.22, 0.27, 0.56, 0.59, 0.59, 0.55, 0.61];
x32=[0.20, 0.20, 0.38, 0.43, 0.43, 0.47, 0.56];
x33=[0.18, 0.22, 0.25, 0.28, 0.35, 0.40, 0.44];

plt.figure(1)
plt.plot(y,x,  '-bs', lw= 2, label='Our Method');
plt.plot(y,x1, '-g^', lw= 2, label='HMM');
plt.plot(y,x2, '-ro', lw= 2, label='linear SVM');
plt.plot(y,x3, '-m*', lw= 2, label='chance');
plt.axis([0, 350, 0, 1])
plt.legend(loc='upper left');
plt.title("# WUT1 dataset", size=20);
plt.xlabel('frames', size=15);
plt.ylabel('accuracy', size=15);
plt.tick_params(labelsize=13);
#plt.grid(True, which='both');

plt.figure(2)
plt.plot(y,x1, '-bs',  lw= 2, label='Our Method');
plt.plot(y,x11, '-g^', lw= 2, label='HMM');
plt.plot(y,x21, '-ro', lw= 2, label='linear SVM');
plt.plot(y,x31, '-m*', lw= 2, label='chance');
plt.axis([0, 350, 0, 1])
plt.legend(loc='upper left');
plt.title("# WUT2 dataset", size=20);
plt.xlabel('frames', size=15);
plt.ylabel('accuracy', size=15);
plt.tick_params(labelsize=13);

plt.figure(3)
plt.plot(y,x2, '-bs',  lw= 2, label='Our Method');
plt.plot(y,x21, '-g^', lw= 2, label='HMM');
plt.plot(y,x22, '-ro', lw= 2, label='linear SVM');
plt.plot(y,x23, '-m*', lw= 2, label='chance');
plt.axis([0, 350, 0, 1])
plt.legend(loc='upper left');
plt.title("# CAD1 dataset", size=20);
plt.xlabel('frames', size=15);
plt.ylabel('accuracy', size=15);
plt.tick_params(labelsize=13);

plt.figure(4)
plt.plot(y,x3, '-bs', lw= 2, label='Our Method');
plt.plot(y,x31, '-g^', lw= 2, label='HMM');
plt.plot(y,x32, '-ro', lw= 2, label='linear SVM');
plt.plot(y,x33, '-m*', lw= 2, label='chance');
plt.axis([0, 350, 0, 1])
plt.legend(loc='upper left');
plt.title("# CAD2 dataset", size=20);
plt.xlabel('frames', size=15);
plt.ylabel('accuracy', size=15);
plt.tick_params(labelsize=13);
plt.show()

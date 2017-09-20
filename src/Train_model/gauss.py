from matplotlib import pyplot as plt
import numpy as np

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

for mu, sig in [(-2.8, 1)]:
    plt.plot(gaussian(np.linspace(-2, 2, 11), mu, sig), c = 'b', label='$\mu = 0.78, \sigma =1.0$', lw = 3)
plt.xlabel('$d (m)$', size = 15)
plt.ylabel(r'$f(dist_{pref})$', size =15)
plt.title('Gaussian Distribution of distance prefernce', size = 18)

plt.legend()
plt.show()

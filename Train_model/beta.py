import numpy as np
from scipy.stats import beta
from matplotlib import pyplot as plt

alpha_values = [0.77]
beta_values = [1.46]

x = np.linspace(0, 1, 1002)[1:-1]
# plot the distributions
fig, ax = plt.subplots(figsize=(5, 3.75))

for a, b in zip(alpha_values, beta_values):
    dist = beta(a, b)
    plt.plot(x, dist.pdf(x), c='b',lw =3, label=r'$\alpha = 0.78, \beta =1.46$')

plt.xlim(0, 1)
plt.ylim(0, 6)

plt.xlabel('$\overline{d}t_{i}$', size =15)
plt.ylabel(r'$f(edge_{prf})$', size =15)
plt.title('Beta Distribution of edge preference', size =18)

plt.legend(loc=0, prop={'size':18,'weight':'bold' })
plt.show()

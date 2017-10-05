import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt


# Generate some data for this demonstration.
data = norm.rvs(5.0, 1.5, size=500)

# Fit a normal distribution to the data:
mu, std = norm.fit(data)

# Plot the histogram.
plt.hist(data, bins=55, normed=True, alpha=0.6, color='g')

# Plot the PDF.
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mu, std)
plt.plot(x, p, 'k', linewidth=2)
title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
plt.title(title)

plt.show()


#import scipy.stats as ss
#import numpy as np
#import matplotlib.pyplot as plt

#x = np.arange(-10, 11)
#xU, xL = x + 0.5, x - 0.5 
#prob = ss.norm.cdf(xU, scale = 3) - ss.norm.cdf(xL, scale = 3)
#prob = prob / prob.sum() #normalize the probabilities so their sum is 1
#nums = np.random.choice(x, size = 10000, p = prob)
##plt.hist(nums, bins = len(x))

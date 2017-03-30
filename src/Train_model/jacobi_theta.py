import numpy as np
from scipy.linalg import solve

'''____Define function____'''

def jacobi(mu, var, x, k):

    D = np.diag(mu)
    R = mu - np.diagflat(D)
    
    for i in range(n):
        x = (var - np.dot(R,x))/ D
        print str(i).zfill(3),
        print(x)
    return x

'''___Main function___'''

mu = np.array([[4.0, -2.0, 1.0], [1.0, -3.0, 2.0], [-1.0, 2.0, 6.0]])
car = [1.0, 2.0, 3.0]
x = [1.0, 1.0, 1.0]
k = 25

print("\n\ninit"),
print(x)
print("")
x = jacobi(mu, var, x, k)
print("\nSol "),
print(x)
print("Act "),
print solve(A, b)
print("\n")

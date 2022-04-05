

import numpy as np

m = 0.61732
b = 40.1
a = 1.0/m

Kp = 200.0
Ki = 150.0

coeff = [1.0, b/m, Kp*a, Ki*a]
roots = np.roots(coeff)
print("Roots: {}".format(roots))

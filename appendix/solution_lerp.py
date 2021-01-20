import time
import numpy as np

p0 = np.random.rand(3)
p1 = np.random.rand(3)

for t in np.arange(0, 1, .01):
    p = p0 * (1 - t) + p1 * t
    viz.applyConfiguration('world/box', list(p) + list(quat.coeffs()))
    time.sleep(.01)

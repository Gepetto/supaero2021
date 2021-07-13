import crocoddyl
import numpy as np
import matplotlib.pylab as plt
from utils.unicycle_utils import plotUnicycle

x0 = np.array([-1., -1., 1.])# initial state (x,y,theta)
T = 20  			  # number of knots

# Creating an action model for the unicycle system
model = crocoddyl.ActionModelUnicycle()
model.costWeights = np.array([10., 1.])

# Creating a particular terminal action model
model_term = crocoddyl.ActionModelUnicycle()
model_term.costWeights = np.array([
    100,   # state weight
    1  # control weight
])

# Formulating the optimal control problem
problem = crocoddyl.ShootingProblem(x0, [model] * T, model_term)

# Creating the DDP solver for this OC problem
ddp = crocoddyl.SolverFDDP(problem)

# Solving it with the DDP algorithm
done = ddp.solve()
assert(done)

plt.clf()
for x in ddp.xs: plotUnicycle(x)
plt.axis([-2,2,-2,2])
plt.show()

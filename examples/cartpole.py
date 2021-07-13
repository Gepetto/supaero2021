# Display the solution
import numpy as np
from IPython.display import HTML

from utils.cartpole_utils import animateCartpole
import crocoddyl

# Hyperparameters
x0 = np.array([0., 3.14, 0., 0.])
DT = 5e-2
T = 50

class DifferentialActionModelCartpole(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self):
        crocoddyl.DifferentialActionModelAbstract.__init__(self, crocoddyl.StateVector(4), 1, 6)  # nu = 1; nr = 6
        self.unone = np.zeros(self.nu)

        self.m1 = 1.
        self.m2 = .1
        self.l = .5
        self.g = 9.81
        self.costWeights = [1., 1., 0.1, 0.001, 0.001, 1.]  # sin, 1-cos, x, xdot, thdot, f

    def calc(self, data, x, u=None):
        if u is None: u = self.unone
        # Getting the state and control variables
        y, th, ydot, thdot = x
        f = u[0]

        # Shortname for system parameters
        m1, m2, l, g = self.m1, self.m2, self.l, self.g
        s, c = np.sin(th), np.cos(th)

        # Defining the equation of motions
        m = m1 + m2
        mu = m1 + m2 * s**2
        xddot = (f + m2 * c * s * g - m2 * l * s * thdot**2) / mu
        thddot = (c * f / l + m * g * s / l - m2 * c * s * thdot**2) / mu
        data.xout = np.array([xddot, thddot])

        # Computing the cost residual and value
        data.r = np.array(self.costWeights * np.array([s, 1 - c, y, ydot, thdot, f]))
        data.cost = .5 * sum(data.r**2)

    def calcDiff(self, data, x, u=None):
        # Advance user might implement the derivatives
        pass


# Creating the DAM for the cartpole, use numdiff to compute derivative, and integrate
cartpoleDAM = DifferentialActionModelCartpole()
cartpoleND = crocoddyl.DifferentialActionModelNumDiff(cartpoleDAM, True)
cartpoleIAM = crocoddyl.IntegratedActionModelEuler(cartpoleND, DT)

# Creating the terminal model with different costs
terminalCartpole = DifferentialActionModelCartpole()
terminalCartpoleDAM = crocoddyl.DifferentialActionModelNumDiff(terminalCartpole, True)
terminalCartpoleIAM = crocoddyl.IntegratedActionModelEuler(terminalCartpoleDAM)
terminalCartpole.costWeights[0] = 100
terminalCartpole.costWeights[1] = 100
terminalCartpole.costWeights[2] = 1.
terminalCartpole.costWeights[3] = 0.1
terminalCartpole.costWeights[4] = 0.01
terminalCartpole.costWeights[5] = 0.0001

# Formulate the optimal control problem and solve
problem = crocoddyl.ShootingProblem(x0, [cartpoleIAM] * T, terminalCartpoleIAM)
ddp = crocoddyl.SolverDDP(problem)
ddp.setCallbacks([crocoddyl.CallbackVerbose()])
ddp.solve([], [], 300)

# Plot
import matplotlib.pylab as plt
plt.plot(ddp.xs)
plt.show()

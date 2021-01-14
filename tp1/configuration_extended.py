'''
Stand-alone program to optimize the placement of a 2d robot, where the decision variables
are the placement of the 3 bodies of the robot. BFGS and SLSQP solvers are used.
'''

import time
import numpy as np
from scipy.optimize import fmin_bfgs,fmin_slsqp
import meshcat
from numpy.linalg import norm,inv,pinv,svd,eig
from meshcat.geometry import Cylinder,Box,Sphere
from transfo import t2d,translation
import colors

viz = meshcat.Visualizer()
viz['joint1'].set_object(Sphere(.1),colors.red)
viz['joint2'].set_object(Sphere(.1),colors.red)
viz['joint3'].set_object(Sphere(.1),colors.red)
viz['arm1'].set_object(Cylinder(.75,.05),colors.grey)
viz['arm2'].set_object(Cylinder(.75,.05),colors.grey)
viz['target'].set_object(Sphere(.1001),colors.green)

def display_9(ps):
    '''Display the robot in Gepetto Viewer. '''
    assert (ps.shape == (9, ))
    x1, y1, t1, x2, y2, t2, x3, y3, t3 = ps
    viz['joint1'].set_transform(t2d(x1,                  y1,                  t1))
    viz['arm1'  ].set_transform(t2d(x1 + np.cos(t1) / 2, x1 + np.sin(t1) / 2, t1))
    viz['joint2'].set_transform(t2d(x2,                  y2,                  t2))
    viz['arm2'  ].set_transform(t2d(x2 + np.cos(t2) / 2, y2 + np.sin(t2) / 2, t2))
    viz['joint3'].set_transform(t2d(x3,                  y3,                  t3))

def endeffector_9(ps):
    assert (ps.shape == (9, ))
    x1, y1, t1, x2, y2, t2, x3, y3, t3 = ps
    return np.array([x3, y3])

target = np.array([.5, .5])
viz['target'].set_transform(translation(0,target[0],target[1]))

def cost_9(ps):
    eff = endeffector_9(ps)
    return norm(eff - target)**2

def constraint_9(ps):
    assert (ps.shape == (9, ))
    x1, y1, t1, x2, y2, t2, x3, y3, t3 = ps
    res = np.zeros(6)
    res[0] = x1 - 0
    res[1] = y1 - 0
    res[2] = x1 + np.cos(t1) - x2
    res[3] = y1 + np.sin(t1) - y2
    res[4] = x2 + np.cos(t2) - x3
    res[5] = y2 + np.sin(t2) - y3
    return res

def penalty(ps):
    return cost_9(ps) + 10 * sum(np.square(constraint_9(ps)))

def callback_9(ps):
    display_9(ps)
    time.sleep(.5)

x0 = np.array([ 0.0,] * 9)

with_bfgs = 0
if with_bfgs:
    xopt = fmin_bfgs(penalty, x0, callback=callback_9)
else:
    xopt = fmin_slsqp(cost_9, x0, callback=callback_9, f_eqcons=constraint_9, iprint=2, full_output=1)[0]
print('\n *** Xopt = %s\n\n\n\n' % xopt)

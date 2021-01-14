'''
Stand-alone program to optimize the configuration q=[q1,q2] of a 2-R robot with
scipy BFGS.
'''

import time
import numpy as np
from scipy.optimize import fmin_bfgs
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

def display(q):
    '''Display the robot in Gepetto Viewer. '''
    assert (q.shape == (2,))
    c0 = np.cos(q[0])
    s0 = np.sin(q[0])
    c1 = np.cos(q[0] + q[1])
    s1 = np.sin(q[0] + q[1])
    viz['joint1'].set_transform(t2d(0,           0,           0))
    viz['arm1'  ].set_transform(t2d(c0 / 2,      s0 / 2,      q[0]))
    viz['joint2'].set_transform(t2d(c0,          s0,          q[0]))
    viz['arm2'  ].set_transform(t2d(c0 + c1 / 2, s0 + s1 / 2, q[0] + q[1]))
    viz['joint3'].set_transform(t2d(c0 + c1,     s0 + s1,     q[0] + q[1]))

def endeffector(q):
    '''Return the 2D position of the end effector of the robot at configuration q. '''
    assert (q.shape == (2,))
    c0 = np.cos(q[0])
    s0 = np.sin(q[0])
    c1 = np.cos(q[0] + q[1])
    s1 = np.sin(q[0] + q[1])
    return np.array([c0 + c1, s0 + s1])

target = np.array([.5, .5])
viz['target'].set_transform(translation(0,target[0],target[1]))
                            
def cost(q):
    eff = endeffector(q)
    return np.linalg.norm(eff - target)**2

def callback(q):
    display(q)
    time.sleep(.5)

x0 = np.array([0.0, 0.0])
xopt_bfgs = fmin_bfgs(cost, x0, callback=callback)
print('\n *** Xopt in BFGS = %s \n\n\n\n' % xopt_bfgs)

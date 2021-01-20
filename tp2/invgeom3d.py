'''
Stand-alone inverse geom in 3D.  Given a reference translation <target> ,
it computes the configuration of the UR5 so that the end-effector position (3D)
matches the target. This is done using BFGS solver. While iterating to compute
the optimal configuration, the script also display the successive candidate
solution, hence moving the robot from the initial guess configuaration to the
reference target.
'''

import math
import time
import numpy as np
import pinocchio as pin
import example_robot_data as robex
from scipy.optimize import fmin_bfgs
from numpy.linalg import norm,inv,pinv,svd,eig

from tp2.meshcat_viewer_wrapper import MeshcatVisualizer

# --- Load robot model
robot = robex.load('ur5')
NQ = robot.model.nq
NV = robot.model.nv

# Open the viewer
viz = MeshcatVisualizer(robot,url='classical')

# Define an init config
robot.q0 = np.array([0, -3.14/2, 0, 0, 0, 0])
viz.display(robot.q0)
time.sleep(1)
print("Let's go to pdes.")

# --- Add ball to represent target
viz.addSphere("world/ball", .05, 'green')
viz.addBox("world/blue", [.08]*3, [.2, .2, 1., .5])

#
# OPTIM 3D #########################################################
#

def cost(q):
    '''Compute score from a configuration'''
    p = robot.placement(q, 6).translation
    return norm(p - target)**2

def callback(q):
    viz.applyConfiguration('world/ball', target.tolist()+[0,1,0,0])
    viz.applyConfiguration('world/blue', robot.placement(q, 6))
    viz.display(q)
    time.sleep(1e-1)

target = np.array([0.5, 0.1, 0.2])  # x,y,z
qopt = fmin_bfgs(cost, robot.q0, callback=callback)

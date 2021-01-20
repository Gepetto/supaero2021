'''
Load 4 times the UR5 model, plus a plate object on top of them, to feature a simple parallel robot.
No optimization, this file is just an example of how to load the models.
'''

import math
import time
import numpy as np
import pinocchio as pin
from load_ur5_parallel import load as loadUR5s

from scipy.optimize import fmin_bfgs
from numpy.linalg import norm,inv,pinv,svd,eig

from tp2.meshcat_viewer_wrapper import MeshcatVisualizer

# Load 4 Ur5 robots, placed at 0.3m from origin in the 4 directions x,y,-x,-y.
robot = loadUR5s()

# Open the viewer
viz = MeshcatVisualizer(url='classical')

# Add a new object featuring the parallel robot tool plate.
[w, h, d] = [0.5, 0.5, 0.005]
color = [red, green, blue, transparency] = [1, 1, 0.78, .3]
viz.addBox('world/robot0/toolplate', [w, h, d], color)
Mtool = pin.SE3(pin.utils.rotate('z', 1.268), np.array([0, 0, .75]))
viz.applyConfiguration('world/robot0/toolplate', Mtool)

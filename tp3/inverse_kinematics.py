'''
Inverse kinematics (close loop / iterative) for a mobile manipulator.
Template of the program for TP3
'''

import pinocchio as pin
import numpy as np
import time
from numpy.linalg import pinv,inv,norm,svd,eig
from tiago_loader import loadTiago
import matplotlib.pylab as plt; plt.ion()
from tp3.meshcat_viewer_wrapper import MeshcatVisualizer

robot = loadTiago()

viz = MeshcatVisualizer(robot,url='classical')

NQ = robot.model.nq
NV = robot.model.nv
IDX_TOOL = robot.model.getFrameId('frametool')
IDX_BASIS = robot.model.getFrameId('framebasis')
IDX_GAZE = robot.model.getFrameId('framegaze')

# Goal placement, and integration in the viewer of the goal.
oMgoal = pin.SE3(pin.Quaternion(-0.5, 0.58, -0.39, 0.52).normalized().matrix(),
                np.array([1.2, .4, .7]))
viz.addBox('goal', [.1,.1,.1], [ .1,.1,.5, .6] )
viz.applyConfiguration('goal',oMgoal)

# Integration step.
DT = 1e-2

# Robot initial configuration.
q0 = np.array([ 0.  ,  0.  ,  1.  ,  0.  ,  0.18,  1.37, -0.24, -0.98,  0.98,
                0.  ,  0.  ,  0.  ,  0.  , -0.13,  0.  ,  0.  ,  0.  ,  0.  ])

q = q0.copy()
herr = [] # Log the value of the error between tool and goal.
# Loop on an inverse kinematics for 200 iterations.
for i in range(200):  # Integrate over 2 second of robot life

    # Run the algorithms that outputs values in robot.data
    pin.framesForwardKinematics(robot.model,robot.data,q)
    pin.computeJointJacobians(robot.model,robot.data,q)

    # Placement from world frame o to frame f oMtool
    oMtool = robot.data.oMf[IDX_TOOL]

    # 3D jacobian in world frame
    o_Jtool3 = pin.computeFrameJacobian(robot.model,robot.data,q,IDX_TOOL,pin.LOCAL_WORLD_ALIGNED)[:3,:]

    # vector from tool to goal, in world frame
    o_TG = oMtool.translation-oMgoal.translation
    
    # Control law by least square
    vq = -pinv(o_Jtool3)@o_TG

    q = pin.integrate(robot.model,q, vq * DT)
    viz.display(q)
    time.sleep(1e-3)

    herr.append(o_TG) 

q = q0.copy()
herr = []
for i in range(200):  # Integrate over 2 second of robot life

    # Run the algorithms that outputs values in robot.data
    pin.framesForwardKinematics(robot.model,robot.data,q)
    pin.computeJointJacobians(robot.model,robot.data,q)

    # Placement from world frame o to frame f oMtool  
    oMtool = robot.data.oMf[IDX_TOOL]

    # 6D error between the two frame
    tool_nu = pin.log(oMtool.inverse()*oMgoal).vector

    # Get corresponding jacobian
    tool_Jtool = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_TOOL)

    # Control law by least square
    vq = pinv(tool_Jtool)@tool_nu

    q = pin.integrate(robot.model,q, vq * DT)
    viz.display(q)
    time.sleep(1e-3)

    herr.append(tool_nu)


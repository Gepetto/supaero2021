'''
# In this example test, we will solve the reaching-goal task with the Talos arm.
# For that, we use the forward dynamics (with its analytical derivatives)
# developed inside crocoddyl; it describes inside DifferentialActionModelFullyActuated class.
# Finally, we use an Euler sympletic integration scheme.
'''

import sys
WITHDISPLAY = 'display' in sys.argv
WITHPLOT = 'plot' in sys.argv

import crocoddyl
import pinocchio
import numpy as np
import example_robot_data

# First, let's load the Pinocchio model for the Talos arm.
robot = example_robot_data.load('talos_arm')
robot_model = robot.model
robot_model.armature =np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.])*5
robot_model.q0 = robot_model.referenceConfigurations['half_sitting']
robot_model.x0 = np.concatenate([robot_model.q0, pinocchio.utils.zero(robot_model.nv)])

# Configure task
FRAME_TIP = robot_model.getFrameId("gripper_left_fingertip_3_link")
GOAL = np.array([.2,0.5,.5])
DT = 1e-2
T = 100

# Configure viewer
from utils.meshcat_viewer_wrapper import MeshcatVisualizer
viz = MeshcatVisualizer(robot,'classical')
viz.display(robot_model.q0)
viz.addBox('world/box',[.1,.1,.1], [1.,0,0,1])
viz.addBox('world/goal',[.1,.1,.1],[0,1,0,1])
viz.applyConfiguration('world/goal',[0.2,0.5,.5,0,0,0,1])

# Create a cost model per the running and terminal action model.
state = crocoddyl.StateMultibody(robot_model)
runningCostModel = crocoddyl.CostModelSum(state)
terminalCostModel = crocoddyl.CostModelSum(state)

# Reaching cost term
pref = crocoddyl.FrameTranslation(FRAME_TIP,GOAL)
goalTrackingCost = crocoddyl.CostModelFrameTranslation(state, pref)
#Mref = crocoddyl.FramePlacement(FRAME_TIP,pinocchio.SE3(np.eye(3), GOAL))
#goalTrackingCost = crocoddyl.CostModelFramePlacement(state, Mref)
runningCostModel.addCost("gripperPose", goalTrackingCost, .001)
terminalCostModel.addCost("gripperPose", goalTrackingCost, 10)

# Regularization cost term
weights=crocoddyl.ActivationModelWeightedQuad(np.array([1,1,1,1,1,1,1, 1,1,1,1,2,2,2.]))
xRegCost = crocoddyl.CostModelState(state,weights,robot_model.x0)
uRegCost = crocoddyl.CostModelControl(state)
runningCostModel.addCost("xReg", xRegCost, 1e-3)
runningCostModel.addCost("uReg", uRegCost, 1e-6)

# Next, we need to create an action model for running and terminal knots. The
# forward dynamics (computed using ABA) are implemented
# inside DifferentialActionModelFullyActuated.
actuationModel = crocoddyl.ActuationModelFull(state)
runningModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, runningCostModel), DT)
runningModel.differential.armature = robot_model.armature
terminalModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, terminalCostModel), 0.)
terminalModel.differential.armature = robot_model.armature

# For this optimal control problem, we define 250 knots (or running action
# models) plus a terminal knot
T = 100
problem = crocoddyl.ShootingProblem(robot_model.x0, [runningModel] * T, terminalModel)

# Creating the DDP solver for this OC problem, defining a logger
ddp = crocoddyl.SolverDDP(problem)
ddp.setCallbacks([
    crocoddyl.CallbackLogger(),
    crocoddyl.CallbackVerbose(),
])

# Solving it with the DDP algorithm
ddp.solve([],[],1000)  # xs_init,us_init,maxiter

# Plotting the solution and the DDP convergence
if WITHPLOT:
    log = ddp.getCallbacks()[0]
    crocoddyl.plotOCSolution(log.xs, log.us, figIndex=1, show=False)
    crocoddyl.plotConvergence(log.costs, log.u_regs, log.x_regs, log.grads, log.stops, log.steps, figIndex=2)

# Visualizing the solution in gepetto-viewer
if WITHDISPLAY:
    import utils.croco_utils as crocutils
    crocutils.displayTrajectory(viz,ddp.xs,ddp.problem.runningModels[0].dt,12)

###########################################################################
################## TODO: Create an IAM with from the DAM ##################
###########################################################################
# Hint:
# Use IntegratedActionModelEuler
timeStep = 5e-2
cartpoleIAM = crocoddyl.IntegratedActionModelEuler(cartpoleND, timeStep)

model_term = crocoddyl.ActionModelUnicycle()
x0 = np.array([-1,-1,1])
problem = crocoddyl.ShootingProblem(x0, [ model ] * T, model_term)
ddp = crocoddyl.SolverDDP(problem)

model_term.costWeights = np.matrix([
    100,   # state weight
    1  # control weight
]).T
done = ddp.solve()
assert(done)

plt.clf()
for x in ddp.xs: plotUnicycle(x)
plt.axis([-2,2,-2,2])

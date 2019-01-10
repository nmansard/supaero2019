tauq = rand(robot.model.nv)
aq = np.linalg.inv(M)*(tauq-b)

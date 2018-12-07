def penalty(x): return cost(x) + 100 * np.linalg.norm(constraint(x))**2
xopt_bfgs = fmin_bfgs(penalty, x0, callback=callback_9)
print '\n *** Xopt BFGS = ',xopt_bfgs,'\n\n\n\n'

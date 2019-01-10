dt = 1e-2
q = robot.q0.copy()
vq = zero(robot.model.nv)

for i in range(10000):

    M = pinocchio.crba(robot.model,robot.data,q)
    b = pinocchio.rnea(robot.model,robot.data,q,vq,zero(robot.model.nv))

    tauq = zero(robot.model.nv)

    aq  = np.linalg.inv(M)*(tauq-b)
    vq += aq*dt
    q   = pinocchio.integrate(robot.model,q,vq*dt)

    if not i % 3: # Only display once in a while ... 
        robot.display(q)
        time.sleep(1e-4)


        

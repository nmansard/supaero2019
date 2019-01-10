Kp = 10.
Kv = 2*np.sqrt(Kp)

q = robot.q0.copy()
vq = zero(robot.model.nv)

dt = 1e-3

def qdes(t):
    '''Compute a reference position for time <t>.'''
    qdes    = robot.q0.copy()
    qdes[2] = 1.5*cos(-t)-1
    qdes[3] = 1.5*cos(-t)-1
    qdes[5] = 1.5*cos(-t/1.5)-1
    qdes[6] = 1.5*cos(-t/1.5)-1
    qdes[8] = 1.5*cos(-t/2.5)-1
    qdes[9] = 1.5*cos(-t/2.5)-1
    return qdes

for i in range(10000):

    M = pinocchio.crba(robot.model,robot.data,q)
    b = pinocchio.rnea(robot.model,robot.data,q,vq,zero(robot.model.nv))

    tauq = -Kp*(q-qdes(i*dt)) - Kv*vq

    aq  = np.linalg.inv(M)*(tauq-b)
    vq += aq*dt
    q   = pinocchio.integrate(robot.model,q,vq*dt)

    if not i % 3: # Only display once in a while ... 
        robot.display(q)
        time.sleep(1e-4)        

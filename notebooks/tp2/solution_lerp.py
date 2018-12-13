import time

p0 = pinocchio.utils.rand(3)
p1 = pinocchio.utils.rand(3)

for t in np.arange(0,1,.01):
    p = p0*(1-t)+p1*t
    gv.applyConfiguration('world/box', p.T.tolist()[0]+quat.coeffs().T.tolist()[0])
    gv.refresh()
    time.sleep(.01)

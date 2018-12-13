'''
Stand-alone slerp example.
'''
import pinocchio
from pinocchio import SE3,Quaternion,AngleAxis
import numpy as np
from numpy.linalg import norm
import time
from gviewserver import GepettoViewerServer
gv = GepettoViewerServer()

def _lerp(p0,p1,t):
    return (1-t)*p0+t*p1

def slerp(q0,q1,t):
    assert(t>=0 and t<=1)
    a = AngleAxis(q0.inverse()*q1)
    return Quaternion(AngleAxis(a.angle*t,a.axis))

def nlerp(q0,q1,t):
    q0 = q0.coeffs()
    q1 = q1.coeffs()
    l  = _lerp(q0,q1,t); l/= norm(l)
    return Quaternion(l[3,0],*l[:3].T.tolist()[0])
    
q0 = Quaternion(SE3.Random().rotation)
q1 = Quaternion(SE3.Random().rotation)
gv.applyConfiguration('world/box', [0,0,0]+q0.coeffs().T.tolist()[0])
time.sleep(.1)
for t in np.arange(0,1,.01):
    q = nlerp(q0,q1,t)
    gv.applyConfiguration('world/box', [0,0,0]+q.coeffs().T.tolist()[0])
    gv.refresh()
    time.sleep(.01)
time.sleep(.1)
gv.applyConfiguration('world/box', [0,0,0]+q1.coeffs().T.tolist()[0])

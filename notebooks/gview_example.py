import time
import numpy as np
import pinocchio as se3
from pinocchio.utils import *
import pinocchio.utils as utils


import gviewserver
gv = gview.GepettoViewerServer()

gv.addSphere ('world/ball',    .1,         [1 ,0 ,0,1])  # radius, color=[r,g,b,1]
gv.addCapsule('world/capsule', .05,.75,    [1 ,1 ,1,1])  # radius, length, color = [r,g,b,1]
gv.addBox    ('world/world/box',     .2,.05,.5,  [.5,.5,1,1]) # depth(x),length(y),height(z), color

gv.applyConfiguration('world/box',  [.1,.1,.1,  1,0,0,0 ]) # x,y,z, quaternion
gv.refresh()

time.sleep(1)
M = se3.SE3(eye(3),np.matrix([.2,.2,.2]).T)
gv.applyConfiguration('world/box',  se3ToXYZQUAT(M))
gv.refresh()

# gv.deleteNode('world', True)

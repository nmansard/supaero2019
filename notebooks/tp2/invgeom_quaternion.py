'''
This script is the various of previous invgeom6d.py, that must be red first.
Instead of optimizing the configuration of a (fixed-basis) manipulator robot, it optimizes
the configuration of a (free-foating-basis) humanoid robot.
Hence the configuration space contains a quaternion.
'''

import pinocchio
from pinocchio.romeo_wrapper import RomeoWrapper
from pinocchio.utils import *
from pinocchio.explog import log
from pinocchio import SE3
import time
from numpy.linalg import inv,norm
from math import pi
from scipy.optimize import fmin_bfgs, fmin_slsqp

# --- Load robot model
pkg   = 'models/romeo/'
urdf  = pkg + 'urdf/romeo.urdf'
robot = RomeoWrapper( urdf, [ pkg, ] )
robot.initDisplay( loadModel = True)
if 'viewer' not in robot.__dict__: robot.initDisplay()

NQ = robot.model.nq
NV = robot.model.nv

# --- Add ball to represent target
robot.viewer.gui.addSphere("world/red", .05, [1.,.2,.2,.5]) # .1 is the radius
robot.viewer.gui.addSphere("world/blue", .05, [.2,.2,1.,.5]) # .1 is the radius
robot.viewer.gui.addSphere("world/green", .05, [.2,1.,2.,.5]) # .1 is the radius
robot.viewer.gui.addSphere("world/yellow", .05, [1.,.2,1.,.5]) # .1 is the radius
robot.viewer.gui.addSphere("world/white", .05, [.9,.9,.9,.5]) # .1 is the radius

# Shortcut function to convert SE3 to 7-dof vector.
M2gv      = lambda M: se3ToXYZQUAT(M)
def place(objectId,M):
     robot.viewer.gui.applyConfiguration(objectId, M2gv(M))
     robot.viewer.gui.refresh() # Refresh the window.

### 
### OPTIM 3D #########################################################
###

class Cost:
     '''Functor class computing the distance of robot effector to target.'''
     def __init__(self,rh_des,rf_des,lf_des,com_des):
          self.rh  = rh_des
          self.rf  = rf_des
          self.lf  = lf_des
          self.com = com_des
          self.weightFeet = 100.
          self.weightCom  = 100.
          self.weightQuat = 1000.
     def __call__(self,q):
          '''Compute score from a configuration'''
          q = np.matrix(q).T
          error  = 0.0
          error += norm(robot.placement(q, robot.rh).translation - self.rh )**2
          error += norm(robot.placement(q, robot.rf).translation - self.rf )**2 * self.weightFeet
          error += norm(robot.placement(q, robot.lf).translation - self.lf )**2 * self.weightFeet
          error += norm(robot.com(q)                            - self.com)**2 * self.weightCom
          error += (norm(q[3:7])-1)**2                                         * self.weightQuat
          return error

class CallbackLogger:
     def __init__(self, sleeptime=1e-2):
          """
          nfevl: iteration number
          dt: animation time pause
          """
          self.nfeval = 1
          self.dt = sleeptime
     def __call__(self, q):
          print '===CBK=== {0:4d}   {1: 3.2f}   {2: 3.2f}'.format(self.nfeval, q[0], q[1])
          q = np.matrix(q).T
          robot.display(q)
          #place('world/blue',robot.placement(q, robot.rh))
          self.nfeval += 1
          time.sleep(self.dt)

## Question 1-2-3
rh  = np.matrix([1.5, 0.1, 1.5 ]).T  # x,y,z
rf  = np.matrix([0., -0.1, 0.0 ]).T  # x,y,z
lf  = np.matrix([0.,  0.1, 0.0 ]).T  # x,y,z
com = np.matrix([0.0, 0.0, 0.5 ]).T  # x,y,z

place('world/red',   SE3(eye(3),rh ))
place('world/blue',  SE3(eye(3),rf ))
place('world/green', SE3(eye(3),lf ))
place('world/yellow',SE3(eye(3),com))

cost = Cost(rh,rf,lf,com)

# If the quaternion is not unitary, it does not correspond a valid rotation. The
# behavior is then not mathematically valid and is implementation dependant. In
# our implementation, quaternions with norm larger than one are interpreted as 
# rotation+scaling.
print 'Optimize with any (non unitary) quaternion'
cost.weightQuat = 0.0   
qopt = fmin_bfgs(cost, robot.q0, callback=CallbackLogger(.1))

# We implement the constraint as a cost with high weight. The quaternion is now unitary.
# The objective is not feasible (red ball too far). The solution respects the priority 
# set by imposing weights on the objectives.
print 'Optimize with constrained quaternion'
cost.weightQuat = 1000.0   
qopt = fmin_bfgs(cost, robot.q0, callback=CallbackLogger(.1))


'''
Inverse kinematics (close loop / iterative) for a mobile manipulator.
Template of the program for TP3
'''

import pinocchio
from mobilerobot import MobileRobotWrapper
from pinocchio.utils import *
import time
from numpy.linalg import pinv

### Load robot model.
pkg = '../models/'
urdf = pkg + 'ur_description/urdf/ur5_gripper.urdf'
                     
robot = MobileRobotWrapper(urdf,[pkg,])
robot.initDisplay(loadModel=True)
gv=robot.viewer.gui
gv.setCameraTransform(0,[-8,-8,2,.6,-0.25,-0.25,.7])

NQ = robot.model.nq
NV = robot.model.nv
IDX_TOOL  = 24
IDX_BASIS = 23

### Set up display environment.
def place(name,M):
    robot.viewer.gui.applyConfiguration(name,se3ToXYZQUAT(M))
    robot.viewer.gui.refresh()

def Rquat(x,y,z,w):
    q = pinocchio.Quaternion(x,y,z,w)
    q.normalize()
    return q.matrix()

Mgoal = pinocchio.SE3(Rquat(0.4,0.02, -0.5,0.7),
                np.matrix([.2,-.4,.7]).T)
try:
    robot.viewer.gui.addXYZaxis('world/framegoal',[1.,0.,0.,1.],.015,.2)
    robot.viewer.gui.addCylinder('world/yaxis',.01,20,[0.1,0.1,0.1,1.])
except:
    pass

place('world/framegoal',Mgoal)
place('world/yaxis',pinocchio.SE3(rotate('x',np.pi/2),
                            np.matrix([0,0,.1]).T))

# Define robot initial configuration
q  = robot.rand()
q[:2] = 0  # Basis at the center of the world.

DT = 1e-2  # Integration step.

# Loop on an inverse kinematics for 200 iterations.
for i in range(200):   # Integrate over 1 second of robot life
      pinocchio.forwardKinematics(robot.model,robot.data,q)     # Compute joint placements
      pinocchio.updateFramePlacements(robot.model,robot.data)   # Also compute operational frame placements
      Mtool = robot.data.oMf[IDX_TOOL]                          # Get placement from world frame o to frame f oMf
      J  = pinocchio.frameJacobian(robot.model,robot.data,q,IDX_TOOL,pinocchio.ReferenceFrame.LOCAL) # Get corresponding jacobian
      ### ... YOUR CODE HERE
      vq    = rand(NV)   #   .... REPLACE THIS LINE BY YOUR CODE ...
      ### ... END OF YOUR CODE HERE
      q = robot.integrate(q,vq*DT)
      robot.display(q)
      time.sleep(DT)


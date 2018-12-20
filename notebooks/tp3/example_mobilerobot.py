'''
This is a simple example showing how to define a mobile manipulator using the class MobileRobotWrapper.
'''

import pinocchio
from mobilerobot import MobileRobotWrapper
from pinocchio.utils import *

pkg = '/home/student/models/'
urdf = pkg + 'ur_description/urdf/ur5_gripper.urdf'
                     
robot = MobileRobotWrapper(urdf,[pkg,])
robot.initDisplay(loadModel=True)
#robot.viewer.gui.addFloor('world/floor')

NQ = robot.model.nq
NV = robot.model.nv

q = robot.rand()
vq = rand(NV)

robot.display(q)

from time import sleep
for i in range(10000):
    q = robot.integrate(q,vq/100)
    robot.display(q)
    sleep(.01)
    print q.T

IDX_TOOL  = 24
IDX_BASIS = 23

se3.framesKinematics(robot.model,robot.data)
Mtool = robot.data.oMf[IDX_TOOL]
Mbasis = robot.data.oMf[IDX_BASIS]

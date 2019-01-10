from pinocchio.utils import *
from robot_hand import Robot
from time import sleep
from numpy import cos
import pinocchio

robot = Robot()

robot.collisionPairs.append([2,8])
robot.collisionPairs.append([2,11])
robot.collisionPairs.append([2,14])
robot.collisionPairs.append([2,16])

dq = rand(robot.model.nq)*12

for i in range(1000):
    #q  = rand(robot.model.nq)*12-6
    q = dq*i/1000
    pinocchio.forwardKinematics(robot.model,robot.data,q)
    dists = [ robot.checkCollision(idx) for idx,pair in enumerate(robot.collisionPairs) ]
    if min(dists)<1e-3: break
    robot.display(q)
    for idx,pair in enumerate(robot.collisionPairs):
        robot.displayCollision(idx,idx)
    

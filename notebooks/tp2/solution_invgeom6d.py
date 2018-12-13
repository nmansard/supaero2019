from scipy.optimize import fmin_bfgs
from pinocchio.utils import se3ToXYZQUAT
import time

# Define an init config
robot.q0 = np.matrix([0, -1.5, 0, 0, 0, 0]).T

Mtarget = pinocchio.SE3(pinocchio.utils.eye(3),np.matrix([0.5, 0.1, 0.2 ]).T)  # x,y,z
def cost(q):
    q = np.matrix(q).T
    M = robot.placement(q, 6)
    return np.linalg.norm(pinocchio.log(M.inverse()*Mtarget).vector)

def callback(q):
    q = np.matrix(q).T
    robot.display(q)
    gv.applyConfiguration('world/blue',se3ToXYZQUAT(robot.placement(q, 6)))
    time.sleep(.1)

robot.display(robot.q0)
qopt = fmin_bfgs(cost, robot.q0, callback=callback)

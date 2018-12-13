from numpy.linalg import norm

robot = loadRomeo()
robot.initDisplay(loadModel=True)

target_rh  = np.matrix([1.5, 0.1, 1.5 ]).T  # x,y,z
target_rf  = np.matrix([0., -0.1, 0.0 ]).T  # x,y,z
target_lf  = np.matrix([0.,  0.1, 0.0 ]).T  # x,y,z
target_com = np.matrix([0.0, 0.0, 0.5 ]).T  # x,y,z

def cost(q):
    q = np.matrix(q).T
    error  = 0.0
    error += norm(robot.placement(q, robot.rh).translation - target_rh )**2
    error += norm(robot.placement(q, robot.rf).translation - target_rf )**2 * weightFeet
    error += norm(robot.placement(q, robot.lf).translation - target_lf )**2 * weightFeet
    error += norm(robot.com(q)                             - target_com)**2 * weightCom
    return error

def callback(q):
    q = np.matrix(q).T
    robot.display(q)
    time.sleep(.1)

robot.display(robot.q0)
qopt = fmin_bfgs(cost, robot.q0, callback=callback)

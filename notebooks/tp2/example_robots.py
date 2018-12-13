WITH_UR5   = True
WITH_ROMEO = False


if WITH_UR5:
    from pinocchio.robot_wrapper import RobotWrapper

    path  = 'models/'
    urdf = path + '/ur_description/urdf/ur5_gripper.urdf'
    robot = RobotWrapper(urdf,[path,])
    
if WITH_ROMEO:
    from pinocchio.romeo_wrapper import RomeoWrapper

    path = 'models/romeo/'
    urdf = path + 'urdf/romeo.urdf'

    # Explicitly specify that the first joint is a free flyer.
    robot = RomeoWrapper(urdf,[path,]) # Load urdf model

robot.initDisplay(loadModel=True)
robot.display(robot.q0)

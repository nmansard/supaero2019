'''
Drive Cozmo's wheels, lift and head motors directly.
'''

import time
import numpy as np
import cozmo

def cozmo_program(robot: cozmo.robot.Robot):
    robot.world.request_nav_memory_map(1)
    N = 4000
    w = 1e-2*np.pi/4

    # Loop following sinusoidal control and printing position and visible cubes.
    for t in range(N):
        robot.drive_wheels( 20*np.sin(w*t),20*np.cos(w*t))
        position = robot.pose.position.x_y_z
        landmarks = [ (i,c.is_visible) for i,c in robot.world.light_cubes.items() ]
        print(position, landmarks)
        time.sleep(1e-1)

cozmo.run_program(cozmo_program,use_viewer=True,use_3d_viewer=True)

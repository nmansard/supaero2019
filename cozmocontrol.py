'''
This program is to control the real robot cozmo using the Q-net trained in qlearn.

The program implements a QValueNetwork, load the network weights that have been computed 
by qlearn.py, implements a cozmo object and start a control loop evaluating the
network from sensor state estimation.

The network should be used to directly evaluate the policy and send it to the robot.
However, the robot API delay makes it difficult. A by-pass is used: the policy network is used
to predict the future 1-second state, and the robot is send to this position using 
the API high-level robot.go_to_pose method. This command is canceled after 1 seconds and
the robot is sent to another position evaluated from sensor and network.
'''

from cozmomodel import Cozmo1 as Env
from qnetwork import *
import time
import cozmo
import sys

### --- Hyper paramaters
NH1 = NH2               = 32            # Hidden layer size

### --- Environment
env                 = Env(discretize_u=True)
NX                  = env.nx            # ... training converges with q,qdot with 2x more neurones.
NU                  = env.nu            # Control is dim-1: joint torque

### --- Tensor flow initialization
qvalue          = QValueNetwork(NX=NX,NU=NU,nhiden1=NH1,nhiden2=NH2,randomSeed=0)
# Useless, just to have the same "restore" action than in qlearn.py.
qvalueTarget    = QValueNetwork(NX=NX,NU=NU,nhiden1=NH1,nhiden2=NH2,randomSeed=0)
sess            = tf.InteractiveSession()
tf.global_variables_initializer().run()
tf.train.Saver().restore(sess, "netvalues/qlearn_cozmo1.ckpt")

### --- QValue evaluation
def policy(x):
    ui = sess.run(qvalue.policy,feed_dict={ qvalue.x: x }).flat[0]
    return env.decode_u(ui)

def preview(env,NSTEPS=20):
    hx = []
    for i in range(NSTEPS):
        x,r = env.step(sess.run(qvalue.policy,feed_dict={ qvalue.x: env.reshape_x(env.x) }))
        hx.append(x)
    return x

def cozmo2net(xc):
    x,y,z = xc.position.x_y_z; az = xc.rotation.angle_z.radians
    return np.array([[ x/1000,y/1000,np.cos(az),np.sin(az) ]])
def net2cozmo(xn):
    a,b,c,s = xn[0,:4]
    az = np.arctan2(s,c)
    return cozmo.util.Pose(x=a*1000,y=b*1000,z=0,angle_z=cozmo.util.radians(az))
c2n = cozmo2net
n2c = net2cozmo

### --- Main Loop
def cozmo_program(robot: cozmo.robot.Robot):
    cozmo.logger.setLevel('WARN')
    hx=[]
    hu=[]
    # Start a first action (this one is trivial, just to start).
    a = robot.go_to_pose(cozmo.util.Pose(x=0,y=0,z=0,angle_z=cozmo.util.radians(0)),
                         relative_to_robot=True)

    # Let's run the loop for 10 seconds.
    for i in range(10):
        x  = env.reset(cozmo2net(robot.pose))           # Get current state from sensors
        xn = preview(env,10)                            # Predict next state 
        hx.append(x);         hu.append(xn)             # Log ...
        print(x,xn)                                     # ... and print 
        if a.is_running: a.abort()                      # Abort preview action if not achieved
        a = robot.go_to_pose(net2cozmo(xn))             # Send the robot to next passing point
        time.sleep(1)
    # \endfor main action loop

    # Just in case, stop the robot motors.
    if a.is_running: a.abort()
    robot.drive_wheel_motors(0,0)
    # Start a ipyshell if you want to debug ... copy it from cozmoshell
    # ipyshell(usage)
        
# Start the main loop    
if __name__ == "__main__":
    cozmo.run_program(cozmo_program, use_viewer=False)


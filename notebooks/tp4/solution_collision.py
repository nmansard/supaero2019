dq = rand(robot.model.nq)*12

for i in range(1000):
    q = dq*i/1000

    # Update the fwd kinematics.
    pinocchio.forwardKinematics(robot.model,robot.data,q)

    # Compute the collision distances.
    dists = [ robot.checkCollision(idx) for idx,pair in enumerate(robot.collisionPairs) ]

    # Stop at collision.
    if min(dists)<1e-3: break

    # Display the robot and the witness points.
    robot.display(q)
    for idx,pair in enumerate(robot.collisionPairs):
        robot.displayCollision(idx,idx)
    

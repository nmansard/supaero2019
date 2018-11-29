Introduction to robotics with Q-learning on Cozmo.

The repository contains 3 sets of python scripts:

* Models
  - pendulum models in pendulummodel.py (for N-d continiuous pendulum and 1-d discrete pendulum)
  - cozmo model in velocity or acceleration, with or with discretization, in cozmomodel.py

* Q-learning algorithm
  - Simple Q-table for discrete Cozmo in qtable.py
  - Deep Q-table, working for the pendulum but failing for Cozmo (dimension) in deeptable.py
  - Deep Q-learn, in qlearn.py, with the control model in qnetwork.py

* Application on Cozmo
  - simple example in cozmo_example.py
  - Cozmo interactive shell in cozmoshell.py
  - Control of cozmo using the Q-network learned above, in cozmocontrol.py

Read install.txt for the list of dependencies on Ubuntu 16.04.

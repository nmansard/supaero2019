# Shooting problem:
# - search for initial velocity
# - so that a free-fall trajectory is generated
# - minimizing a integral+terminal cost over the trajectory
# - knowing terminal time

from pyomo.environ import *
from pyomo.dae import *
from math import pi

model = ConcreteModel()

model.PlaneR2   = RangeSet(0,1)                 # Variables defined in the plane R^2 = {x1,x2}
model.UnitTime  = ContinuousSet(bounds=(0.,1.)) # Time scaled between 0,1 (for var-time problem)

model.target    = Param(model.PlaneR2, initialize={0:1.0,1: .0})  # Target to reach
model.targetang = Param(initialize=pi/4)                           # Target angle to reach

model.x         = Var(model.PlaneR2,model.UnitTime)           # Position 
model.th        = Var(model.UnitTime,bounds=(-1.5*pi,1.5*pi)) # Angle
model.v         = Var(model.PlaneR2,model.UnitTime,bounds=(-5,5)) # Track velocity
model.u         = Var(model.PlaneR2,model.UnitTime,bounds=(-5,5)) # Track acceleration (control)

model.Tf        = Var(bounds=(0,None))                        # Terminal time

model.dx_ds     = DerivativeVar(model.x, wrt=model.UnitTime)  # abscissa-derivative of x
model.dth_ds    = DerivativeVar(model.th,wrt=model.UnitTime)  # abscissa-derivative of th
model.dv_ds     = DerivativeVar(model.v, wrt=model.UnitTime)  # abscissa-derivative of v

model.delta     = 0.1                                         # Space between the tracks.
model.x0        = Param(model.PlaneR2,initialize={0: 0,1:0},mutable=True)
model.th0       = Param(initialize=0,mutable=True)
model.v0        = Param(model.PlaneR2,initialize={0: 0,1:0},mutable=True)

# Objective Function: minimum time
def _obj(model):
    return model.Tf
model.obj = Objective( rule = _obj )

# x derivative
def _ode_x(model,k,s):
    v = (model.v[0,s]+model.v[1,s])/2
    if   k==0: return model.dx_ds[k,s] == model.Tf*v*cos(model.th[s])
    elif k==1: return model.dx_ds[k,s] == model.Tf*v*sin(model.th[s])
model.dx_con = Constraint(model.PlaneR2, model.UnitTime, rule=_ode_x)

# th derivative
def _ode_th(model,s):
    w = (model.v[0,s]-model.v[1,s])/model.delta
    return model.dth_ds[s] ==  model.Tf*w
model.dth_con = Constraint(model.UnitTime,rule=_ode_th)

# v derivative
def _ode_v(model,k,s):
    return model.dv_ds[k,s] ==  model.Tf*model.u[k,s]
model.dv_con = Constraint(model.PlaneR2,model.UnitTime,rule=_ode_v)

# Set initial and final conditions
def _init(model):
    # --- Initial 
    yield model.x[0,0] == model.x0[0]
    yield model.x[1,0] == model.x0[1]
    yield model.th [0] == model.th0
    yield model.v[0,0] == model.v0[0]
    yield model.v[1,0] == model.v0[1]

    # --- Terminal
    yield model.x[0,1] == model.target[0]
    yield model.x[1,1] == model.target[1]
    yield model.th [1] == model.targetang
    yield model.v[0,1] == 0.
    yield model.v[1,1] == 0.

    yield ConstraintList.End
model.init_conditions = ConstraintList(rule=_init)

# --- SOLVE ---
# --- SOLVE ---
# --- SOLVE ---

discretizer = TransformationFactory('dae.finite_difference')
discretizer.apply_to(model,nfe=10,scheme='BACKWARD')

solver=SolverFactory('ipopt')

results = solver.solve(model,tee=True)

# --- PLOT ---
# --- PLOT ---
# --- PLOT ---

import matplotlib.pylab as plt
plt.ion()

xval    = [ [value(model.x[0,s]),value(model.x[1,s])] for s in model.UnitTime ]
tval    = [ s*value(model.Tf)  for s in model.UnitTime ]
thval   = [ value(model.th[s]) for s in model.UnitTime ]
vval    = [ [value(model.v[0,s]),value(model.v[1,s])] for s in model.UnitTime ]
uval    = [ [value(model.u[0,s]),value(model.u[1,s])] for s in model.UnitTime ]

plt.subplot(131)
plt.plot([x[0] for x in xval],[x[1] for x in xval])
L=1.2*max([max([abs(x) for x in xy]) for xy in xval])
plt.axis([-L,L,-L,L])

plt.subplot(132)
plt.plot(tval,vval)

plt.subplot(133)
plt.plot(tval,uval)



# --- MPC style
from cozmomodel import Cozmo2
import numpy as np

env = Cozmo2()
env.reset()
env.x[:] = [ model.x0[0].value, model.x0[1].value,
             np.cos(model.th0.value), np.sin(model.th0.value),
             model.v0[0].value, model.v0[1].value ]
plt.figure(2)
env.umax*=100
env.umin*=100

for i in range(10):
    s1 = model.UnitTime[2]
    env.dt = s1*model.Tf.value
    env.step(np.array([ model.u[0,s1].value, model.u[1,s1].value ]))
    
    model.x0[0] = env.x[0]
    model.x0[1] = env.x[1]
    model.th0   = np.arctan2(env.x[3],env.x[2])
    model.v0[0] = env.x[4]
    model.v0[1] = env.x[5]

    results = solver.solve(model,tee=False)
    plt.plot([ model.x[0,s].value for s in model.UnitTime],
             [model.x[1,s].value for s in model.UnitTime])

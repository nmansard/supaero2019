'''
We denote by:
 - a,b the position in the plane
 - theta the orientation of the body, c=cos(theta), s=sin(theta)
 - v,w the forward velocity and angular velocity
 - vr and vl the linear velocity of the caterpilar
 - r the radius of the circle that the robot is following 
 - delta the space between the caterpilars.

Then 
vr = -w(r-delta) ; vl = -w(r+delta)
v = wr = (vr+vl)/2
w = (vr-vl)/delta

With the configuration a,b,c,s and the control u1=vr, u2=vl:
adot = v cos(theta) = (vr+vl)c
bdot = v sin(theta) = (vr+vl)s
cdot = d/dt cos(theta) = -sin(theta) w = -s (vr-vl)/delta
sdot = d/dt sin(theta) =  cos(theta) w =  c (vr-vl)/delta

'''
import numpy as np
from discretization import VectorDiscretization

# Because plt.pause raises a false warning (https://stackoverflow.com/questions/22873410/)
import warnings
warnings.filterwarnings("ignore",".*GUI is implemented.*")

class Cozmo1(object):
    '''
    Velocity-controled model of Cozmo.
    The main methods reset, step and render are used to roll-out Cozmo behaviours.
    The class can be built with continuous or discretized state and control (continuous
    by default).
    '''
    def __init__(self, discretize_x = False, discretize_u = False):
        '''
        Build the environment with or without discretize state and control.
        The state is continous in theory, and the main functions randomState, dyn, cost and
        display works with a continuous state and control. 
        However, a discretization can be added with the proper arguments.
        In that case, the stored state / control are int variables. Use self.decode_{x,u} and
        self.encode_{x,u} to go from int to continuous (decode) and reverse (encode).
        The reset, step and render methods are basically decoding state and control (if
        need be), calling the respective randomState, dyn-cost, display methods, and
        finally encoding state and control (if need be).
        '''
        # The state is stored as x_pos,y_pos,cos(theta),sin(theta), with c**2+s**2=1
        # The control is the velocities of right and left tracks u = vr,vl.
        self.xmax = np.array([ .5,  .5, 1., 1. ])       # Bounds of the state.
        self.xmin = -self.xmax                          # Lower bounds of the state
        self.umax = np.array([ .55, .55 ])              # Upper bound of control
        self.umin = -self.umax                          # Lower bound of control
        # Discretization of X and U
        if discretize_u:
            self.discretize_u = VectorDiscretization(2,vmax=self.umax,nsteps=21)
            self.encode_u = self.discretize_u.c2i
            self.decode_u = self.discretize_u.i2c
        else:
            self.discretize_u = None
            self.encode_u = lambda u:u
            self.decode_u = lambda u:u
        if discretize_x:
            XM = np.concatenate([self.xmax[:2],[np.pi]])
            xycs2sxth = lambda x: np.array([ x[0],x[1],np.arctan2(x[3],x[2])])
            xyth2sxcs = lambda x: np.array([ x[0],x[1],np.cos(x[2]),np.sin(x[2])])
            self.discretize_x = VectorDiscretization(3,vmax=XM,nsteps=11)
            self.encode_x = lambda xc: self.discretize_x.c2i(xycs2sxth(xc))
            self.decode_x = lambda xi: xyth2sxcs(self.discretize_x.i2c(xi))
        else:
            self.discretize_x = None
            self.encode_x = lambda x:x
            self.decode_x = lambda x:x
        # Model parameters
        self.delta = .06                                # Space between the two caterpillars
        self.dt               = .1                      # Duration of one step(x,u)
        self.integrationSteps = 10                      # Number of internal integration in one
                                                        # step (of time dt/steps)
        # Internal mutables
        self._ax = None
        self._plotobject_right = None; self._plotobject_left = None
        self.expected_shape = [1,4]
        self.reshape_x = (lambda x: np.reshape(x,self.expected_shape))\
                         if self.discretize_x is None \
                         else (lambda x:x)
        
    @property
    def nu(self): return 2 if self.discretize_u is None else self.discretize_u.nd
    @property
    def nx(self): return 4 if self.discretize_x is None else self.discretize_x.nd
    
    def render(self,sleep=.01,ion=True,newfig=False):
        '''
        This method internally calls self.display, but first recover the 
        robot state and build / clean the figure display.
        '''
        import matplotlib.pyplot as plt
        if ion: plt.ion()
        else: plt.ioff()
        if self._ax is None or len(self._ax.axes)==0:
            self._ax = plt.figure() if newfig else plt.gcf()
            self._plotobject_left = None
            self._plotobject_right = None
            if plt.axis() == (0,1,0,1):
                d = self.xmax[:2] - self.xmin[:2]
                plt.axis([ self.xmin[0]-.1*d[0],self.xmax[0]+.1*d[0],
                           self.xmin[1]-.1*d[1],self.xmax[1]+.1*d[1]])
        else:
            if self._plotobject_right is not None: self._plotobject_right.remove()
            if self._plotobject_left is not None: self._plotobject_left.remove()
        self._plotobject_left,self._plotobject_right = self.display(self.decode_x(self.x))
        if sleep>0: plt.pause(sleep)
    def reset(self,x=None):
        '''
        This method internally calls self.randomState() and stores the results. 
        '''
        x =  self.randomState() if x is None else np.reshape(self.decode_x(x),[4])
        self.x = self.encode_x(x)
        return self.reshape_x(self.x)
    def step(self,u):
        '''
        This method internally calls self.cost() and self.dyn() from internal state,
        stores the results and return xnext,reward.
        '''
        if self.discretize_u is not None and isinstance(u,np.ndarray): u = u.flat[0]
        self.u = u
        u = self.decode_u(u)
        if self.discretize_u is None: self.u = u = np.clip(u,self.umin,self.umax)
        x = self.decode_x(self.x).copy()
        for i in range(self.integrationSteps):
            x += self.dyn(x,u)*self.dt / self.integrationSteps
            x[2:4] /= np.linalg.norm(x[2:4])
            #x = np.clip(x,self.xmin,self.xmax)
        #assert(np.all(x<=self.xmax) and np.all(x>=self.xmin))
        self.x = self.encode_x(x)
        self.r = self.cost(x,u)
        return self.reshape_x(self.x),self.r

    # Internal methods corresponding to reset (randomState), step (cost and dyn) and
    # render (display). They are all to be used with continuous state and control, and are
    # read only (no change of internal state).
    def randomState(self):
        xy = np.random.rand(2)*(self.xmax[:2]-self.xmin[:2])+self.xmin[:2]
        th = np.random.rand()*np.pi*2
        return np.concatenate([xy,[np.cos(th), np.sin(th)]])
    def cost(self,x,u):
        '''
        Const method returning the reward of making one step (positive = reward, negative = cost).
        This method is called inside step(u).
        '''
        norm = np.linalg.norm
        return -norm(x[:2])     \
            -.1*norm(x[2:4]-[1,0]) \
            -0.1*norm(u) \
            + (20 if norm(x-[0,0,1,0])<1e-2 else 0)
    def dyn(self,x,u):
        '''
        Const method: return f(x,u) = dx/dt 
        This method is called inside step(u).
        '''
        a,b,c,s = x
        vr,vl = u
        v,w = (vr+vl)/2,(vr-vl)/self.delta
        dx_dt  = np.array([ v*c,v*s, -s*w, c*w ])
        return dx_dt
    def display(self,x):
        sc,delta = self.delta,self.delta
        import matplotlib.pyplot as plt
        a,b,c,s = x[:4]
        refs = []
        refs.append(plt.arrow(a-sc/2*c-delta*s,b-sc/2*s+delta*c,c*sc,s*sc,head_width=.03))
        refs.append(plt.arrow(a-sc/2*c+delta*s,b-sc/2*s-delta*c,c*sc,s*sc,head_width=.03))
        return refs

class Cozmo2(Cozmo1):
    '''
    Second order cozmo model, where controls are caterpillar accelerations.
    State x = [a,b,c,s,vr,vl], control u = [ar,al].
    This second model is strongly built upon Cozmo1. Basically, only the
    internal randomState, dyn and cost methods are changed.
    '''
    def __init__(self, discretize_x = False, discretize_u = False):
        Cozmo1.__init__(self)
        self.xmax = np.concatenate([self.xmax,self.umax])
        self.xmin = np.concatenate([self.xmin,self.umin])
        self.umax = np.array([ .5,.5 ])
        self.umin = -self.umax
        # Discretization of X and U
        if discretize_u:
            self.discretize_u = VectorDiscretization(2,vmax=self.umax,nsteps=5) #11
            self.encode_u = self.discretize_u.c2i
            self.decode_u = self.discretize_u.i2c
        else:
            self.discretize_u = None
            self.encode_u = lambda u:u
            self.decode_u = lambda u:u
        assert(not discretize_x) # Not implemented, not sure that discretize state dim=5 is relevant.
        self.discretize_x = None; self.decode_x = self.encode_x = lambda x:x
        self.expected_shape = [1,6]
        
    @property
    def nx(self): return 6
        
    def randomState(self):
        x = Cozmo1.randomState(self)
        v = np.random.rand(2)*(self.xmax[4:6]-self.xmin[4:6])+self.xmin[4:6]
        return np.concatenate([x,v])
    def dyn(self,x,u):
        return np.concatenate([ Cozmo1.dyn(self,x=x[:4],u=x[4:6]),u ])
    def cost(self,x,u):
        norm = np.linalg.norm
        return -norm(x[:4]-[0,0,1,0]) \
            -0.01*norm(x[4:]) \
            -0.1*norm(u)
       
    
if __name__ == "__main__":
    env = Cozmo1(discretize_u=False)
    env.reset()
    for i in range(100):
        assert( np.all(env.reset()>env.xmin) and np.all(env.reset()<env.xmax))

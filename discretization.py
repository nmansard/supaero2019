'''
Helper class to systematically convert a R^n vector of float into a 
N^n vector of integers, and to a single integer.
The main methods are flat2continuous and continuous2flat.
Build it from the size of the real vector space, the bounds in each directions and
the number of discretization steps.

See also numpy.digitalize.
'''

import numpy as np
from functools import reduce

class VectorDiscretization:
    '''
    Convertion between continuous R^N bounded vectors and their discretization
    as N^N bounded integer vectors and N*N integer.
    '''
    def __init__(self,nv,vmax=1.,vmin=None,nsteps=10):
        self.nv = nv  # Dimension of the vector space
        self.vmax = vmax if isinstance(vmax,np.ndarray) else np.array([vmax,]*nv)
        self.vmin = vmin if isinstance(vmin,np.ndarray) else -self.vmax if vmin is None else np.array([vmin,]*nv)
        self.nsteps = nsteps if isinstance(nsteps,np.ndarray) else np.array([nsteps,]*nv)
        self.nd = reduce(lambda i,j:i*j,self.nsteps)
        assert( self.nsteps.dtype == np.int32 or self.nsteps.dtype == np.int64)

    def continuous2discrete(self,vc):
        vc = np.clip(vc,self.vmin,self.vmax)
        return ((vc-self.vmin)/(self.vmax-self.vmin)*self.nsteps).astype(self.nsteps.dtype)

    def discrete2continuous(self,vd):
        return (vd+.5)*(self.vmax-self.vmin)/self.nsteps+self.vmin

    def flatten(self,vd):
        '''change an array of int to a flat int.'''
        #assert( np.all(vd<self.nsteps) )
        vd = np.clip( vd,0,self.nsteps-1 )
        # Return (((v1)n2+v2)n3+v3)n4+...
        # return reduce(lambda v1_n1,v2_n2: (v1_n1[0]*v2_n2[1]+v2_n2[0],v1_n1[1]*v2_n2[1]),
        #               zip(vd,self.nsteps))[0]
        res = 0
        for v,n in zip(vd,self.nsteps):            res = res*n+v
        return res
    
    def unflatten(self,i):
        '''change a flat int into an array of int.'''
        assert(i>=0 and i<self.nd)
        # res,check = reduce(lambda res_ii,n: (res_ii[0]+[res_ii[1]%n],res_ii[1]/n),
        #                    [([],i)]+self.nsteps.tolist())
        res = []
        for n in self.nsteps:
            res.append(i%n)
            i //= n
        assert(0==i)
        return np.array([ii for ii in reversed(res)])

    def flat2continuous(self,i):
        return self.discrete2continuous(self.unflatten(i))
    def continuous2flat(self,vc):
        return self.flatten(self.continuous2discrete(vc))

    i2c=flat2continuous
    c2i=continuous2flat

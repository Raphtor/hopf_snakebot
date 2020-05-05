
import numpy as np
from numpy import sin, cos, pi, square
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64
from utils import rk4, euler, hopf, rot2d



class OscillatorNode(object):
    def __init__(self, ix, iw, ip, il, si, isigma=1, method='euler'):
        self.estimator = euler if method=='euler' else rk4
        self.x = ix.copy()
        self.last_t = None
        self.set_hopf(iw, ip, il, isigma)
        self.s = si
    def set_hopf(self,w,p,l,sigma):
        self.f =  hopf(w,p,l,sigma)
        
    def step(self,t):
        
        if self.last_t is None:
            dt = 0.001
        else:
            dt = t - self.last_t
        self.last_t = t
        new_x = self.estimator(self.f, self.x, t, dt, s=self.s)
        self.x = new_x
        self._update_s()
        return self.x
    
    def _update_s(self):
        pass






class CoupledOscillator(OscillatorNode):
    '''
    A coupled oscillator of the form 4.c from Zang et al. (2017). 
    The assumption here is that the first n/2 oscillators are the left side of the coupled pair
    and last n/2 oscillators are the right side
    '''
    def __init__(self, dl=pi/5 ,k=1,a=0.4, b=1,c=0.9, d=1,*args, **kwargs):
        self.set_dl(dl)
        self.k = k
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        super(CoupledOscillator, self).__init__(*args, **kwargs)
    def set_dl(self,dl):
        self.dl = dl
    def _update_s(self):
        n = self.s.shape[1]
        a = self.a
        b = self.b
        c = self.c
        d = self.d
        k = self.k
        x = self.x
        n2 = int(n/2)
        r = rot2d(self.dl)
        ir = rot2d(-self.dl)
        r90 = rot2d(pi)
        rn90 = rot2d(-pi/2)
        s = self.s
        for i in range(n2):
            
            s[:,i] = -d*(x[:,i] - np.matmul(r90,x[:, n2+i]))
            if i < n2-1:
                s[:,i] += -a*(x[:,i] - np.matmul(r,x[:, i+1]))
            if i > 0:
                s[:,i] += -b*(x[:,i] - np.matmul(r,x[:, i-1] )) 
            s[:,i] *= k
        for i in range(n2,n):
            
            s[:,i] = -c*(x[:,i] - np.matmul(rn90,x[:,i-n2])) 
            if i < n-1:
                s[:,i] += -a*(x[:,i] - np.matmul(ir,x[:, i+1]))
            if i > n2:
                s[:,i] += -b*(x[:,i] - np.matmul(ir,x[:, i-1]))
            s[:,i] *= k
        self.s = s



    






class CoupledOscillatorController(CoupledOscillator):
    def __init__(self, publisher_fstring, *args, **kwargs):
        super(CoupledOscillatorController, self).__init__(*args, **kwargs)
        n = self.x.shape[1]
        rospy.init_node("CoupledOscillatorController")
        self.pubs = []
        for i in range(n):
            
            pub = rospy.Publisher(publisher_fstring.format(i+1), Float64, queue_size=10)
            self.pubs.append(pub)

    def step(self, *args, **kwargs):
        t = rospy.get_time()
        super(CoupledOscillatorController, self).step(t,*args, **kwargs)
        for i, pub in enumerate(self.pubs):
            pub.publish(self.x[0,i])




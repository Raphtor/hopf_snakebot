import numpy as np
from numpy import sin, cos, pi, square
def rk4(f,y,t,dt, *args, **kwargs):
    k1 = f(t,y, *args, **kwargs)
    k2 = f(t+dt/2, y + dt*k1/2, *args, **kwargs)
    k3 = f(t+dt/2, y + dt*k2/2, *args, **kwargs)
    k4 = f(t+dt, y+dt*k3, *args, **kwargs)
    ret = y + dt*(k1 + 2*k2 + 2*k3 + k4)/6
    
    return ret

def euler(f,y,t,dt, *args, **kwargs):
    ret = y+f(t,y, *args, **kwargs)*dt
    return ret
def hopf(w,p,l, sigma=1):
    def f(t,x,s):
        
        u = x[0,:]
        v = x[1,:]
        dx = np.zeros(x.shape)
        sqr = (square(u)+square(v))/square(p)
        dx[1,:] = -w*u - l*(sqr-sigma)*v + s[1,:]
        dx[0,:] = w*v - l*(sqr-sigma)*u + s[0,:]
        return dx
    return f

def rot2d(w):
    return np.array([[cos(w), -sin(w)],[sin(w), cos(w)]])
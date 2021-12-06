"""
Control.
"""
from part2 import *
import math as m
import numpy as np
from icecream import ic



def S(n):
    result = np.array([
	[ 0,          -float(n[2]),  float(n[1])] ,
	[ float(n[2]), 0,           -float(n[0])],
	[-float(n[1]), float(L[0]),  0 ]
	])
    return result

def control_orientation(Rd,Re,wd):

    Ko = 1

    nd = Rd[0:3,0]
    sd = Rd[0:3,1]
    ad = Rd[0:3,2]

    ne = Re[0:3,0]
    se = Re[0:3,1]
    ae = Re[0:3,2]

    eo = 1/2*(np.cross(ne,nd)+np.cross(se,sd)+np.cross(ae,ad))

    L = -1/2*(S(nd)*S(ne)+S(sd)*S(se)+S(sd)*S(se))

    Linv = np.linalg.inv(L)
    
    Lt = L.transpose()

    uo = Linv*(Lt*wd+Ko*eo)

    return uo

def control_position(Xd,X,D,r_dot):
    kp = 2
    ep = Xd - X
    Xd_dot = np.dot(r_dot,D)
    up = Xd_dot + kp*ep
    return up
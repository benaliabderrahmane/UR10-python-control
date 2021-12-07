"""
Control.
"""
from part2 import *
import math as m
import numpy as np
from icecream import ic

def inv(m): #compute inv of  sigular matrix
    a, b = m.shape
    if a != b:
        raise ValueError("Only square matrices are invertible.")

    i = np.eye(a, a)
    return np.linalg.lstsq(m, i)[0]

def S(n):
    result = np.array([
	[ 0,          -float(n[2]),  float(n[1])] ,
	[ float(n[2]), 0,           -float(n[0])],
	[-float(n[1]), float(n[0]),  0 ]
	])
    return result

def control_orientation(Rd,Re,wd, ko):

    nd = Rd[0:3,0]
    sd = Rd[0:3,1]
    ad = Rd[0:3,2]

    ne = Re[0:3,0]
    se = Re[0:3,1]
    ae = Re[0:3,2]

    eo = 1/2*(np.cross(ne,nd)+np.cross(se,sd)+np.cross(ae,ad))

    eo = np.array(eo).reshape(3,1)

    L = -1/2*(S(nd)*S(ne)+S(sd)*S(se)+S(ad)*S(ae)) 

    #Linv = np.linalg.inv(L+10)
    
    Linv = inv(L)

    Lt = L.transpose()

    uo = np.dot(Linv,(np.dot(Lt, wd) + np.dot(ko, eo)))

    return uo

def control_position(Xd,X,D,r_dot, kp):
    ep = Xd - X
    Xd_dot = np.dot(r_dot,D)
    up = Xd_dot + kp*ep
    return up
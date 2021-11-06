'''
Trajectory generation
issue with Rd, problem in rot(u,r theta) : r is 3x1 while in the formula should be 1x1
Xd is fine i think.
'''

import numpy as np
import math as m
from icecream import ic

#maximum joints velocities
v0 = 10
kv1 = v0
kv2 = v0
kv3 = v0
kv4 = v0
kv5 = v0
kv6 = v0
kv = [kv1,kv2,kv3,kv4,kv5,kv6]

#maximum joints accelerations
a0 = 5
ka1 = a0
ka2 = a0
ka3 = a0
ka4 = a0
ka5 = a0
ka6 = a0
ka = [ka1,ka2,ka3,ka4,ka5,ka6] 

def final_time_and_r(t,Xf,Xi): 
    tfj_x = []
    tfj_y = []
    tfj_z = []
    Dj = Xf-Xi
    for j in range(6):
        tfv = 15*np.abs(Dj)/(8*kv[j])
        tfa = np.sqrt(10*np.abs(Dj)/(np.sqrt(3)*ka[j]))
        tfj_x.append(max(tfv[0,0],tfa[0,0])) 
        tfj_y.append(max(tfv[1,0],tfa[1,0]))
        tfj_z.append(max(tfv[2,0],tfa[2,0]))

    tfx = max(tfj_x)
    tfy = max(tfj_y)
    tfz = max(tfj_z)

    rx = 10*(t/tfx)**3 - 15*(t/tfx)**4 + 6*(t/tfx)**5
    ry = 10*(t/tfy)**3 - 15*(t/tfy)**4 + 6*(t/tfy)**5
    rz = 10*(t/tfz)**3 - 15*(t/tfz)**4 + 6*(t/tfz)**5
    return rx, ry, rz

def generate_trajectory(Xi,Xf,Ri,Rf,t): #t ! 
    
    #compute rot (u,theta)
    rot_u_theta = np.array([])
    rot_u_theta = (np.transpose(Ri)).dot(Rf)
    c_theta = 0.5*(rot_u_theta[0,0] + rot_u_theta[1,1] + rot_u_theta[2,2] - 1 )
    s_theta = 0.5*m.sqrt( pow(rot_u_theta[1,2]-rot_u_theta[2,1],2) + pow(rot_u_theta[2,0]-rot_u_theta[0,2],2) + pow(rot_u_theta[0,1]-rot_u_theta[1,0],2) )
    s_theta+= 0.00000001 #not divide by 0
    #compute theta , u
    theta = np.arctan2(s_theta,c_theta)
    u = (1/(2*s_theta)) * np.array([
        [rot_u_theta[1,2]-rot_u_theta[2,1]],
        [rot_u_theta[2,0]-rot_u_theta[0,2]],
        [rot_u_theta[0,1]-rot_u_theta[1,0]]
        ])
    #compute rot(u,r(t)theta)
    rx, ry, rz = final_time_and_r(t,Xi,Xf)   
    #issue ! r must be 1x1 while r is 3x1
    r = rx;
    a=[u[0]*u[0]*(1-m.cos(r*theta))+m.cos(r*theta),    u[0]*u[1]*(1-m.cos(r*theta))-u[2]*m.sin(r*theta), u[0]*u[2]*(1-m.cos(r*theta))+u[1]*m.sin(r*theta)]
    b=[u[0]*u[1]*(1-m.cos(r*theta))+u[2]*m.sin(r*theta), u[1]*u[1]*(1-m.cos(r*theta))+m.cos(r*theta),      u[1]*u[2]*(1-m.cos(r*theta))-u[0]*m.sin(r*theta)]
    c=[u[0]*u[2]*(1-m.cos(r*theta))-u[1]*m.sin(r*theta), u[1]*u[2]*(1-m.cos(r*theta))+u[0]*m.sin(r*theta), u[2]*u[2]*(1-m.cos(r*theta))+m.cos(r*theta)]
    rot_u_r_theta = [a,b,c]
     
    #compute Rd(t) 
    Rd = Ri.dot(rot_u_r_theta)
    #compute Xd
    r = np.array([[rx],[ry],[rz]])
    Xd = np.array([[0],[0],[0]])
    Xd = Xi + r*(Xf-Xi);
    ic(Rd.reshape(3,3))
    ic(Xd)
    return Xd, Rd

if __name__ == "__main__":

    Xi = np.array([0,0,0]).reshape(3,1)
    Xf = np.array([2,2,2]).reshape(3,1)
    Ri = np.array([[10,10,10],[20,20,10],[10,5,1]])
    Rf = np.array([[100,10,10],[10,1,10],[10,0,10]])
    t=0.1
    generate_trajectory(Xi, Xf, Ri, Rf, t)

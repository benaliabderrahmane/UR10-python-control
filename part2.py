'''
Trajectory generation
verify relations
'''
import numpy as np
import math as m
from icecream import ic




def generate_r(t, tf):
    r = 10*(t/tf)**3 - 15*(t/tf)**4 + 6*(t/tf)**5
    r_dot = 30*(t**2/tf**3) - 60*(t**3/tf**4) + 30*(t**4/tf**5)
    return r, r_dot

def generate_trajectory(Xi, Xf, Ri, Rf, t, tf):  
    # compute rot (u,theta)
    rot_u_theta = np.array([])
    rot_u_theta = (np.transpose(Ri)).dot(Rf)
    c_theta = 0.5*(rot_u_theta[0, 0] + rot_u_theta[1, 1] + rot_u_theta[2, 2] - 1)
    s_theta = 0.5*m.sqrt(pow(rot_u_theta[1, 2]-rot_u_theta[2, 1], 2) + pow(rot_u_theta[2, 0]-rot_u_theta[0, 2], 2) + pow(rot_u_theta[0, 1]-rot_u_theta[1, 0], 2))
    #s_theta += 0.00000001  # not divide by 0
    # compute theta , u
    theta = np.arctan2(s_theta, c_theta)
    if s_theta != 0:
        u = (1/(2*s_theta)) * np.array([
            [rot_u_theta[1, 2]-rot_u_theta[2, 1]],
            [rot_u_theta[2, 0]-rot_u_theta[0, 2]],
            [rot_u_theta[0, 1]-rot_u_theta[1, 0]]
        ])
    else:
        u = np.array([
           0.0,
           0.0,
           0.0
        ])
    # compute rot(u,r(t)theta)
    r, r_dot = generate_r(t, tf)
    
    a = [u[0]*u[0]*(1-m.cos(r*theta))+m.cos(r*theta),    u[0]*u[1]*(1-m.cos(r*theta)) -
         u[2]*m.sin(r*theta), u[0]*u[2]*(1-m.cos(r*theta))+u[1]*m.sin(r*theta)]
    b = [u[0]*u[1]*(1-m.cos(r*theta))+u[2]*m.sin(r*theta), u[1]*u[1]*(1-m.cos(r*theta)) +
         m.cos(r*theta),      u[1]*u[2]*(1-m.cos(r*theta))-u[0]*m.sin(r*theta)]
    c = [u[0]*u[2]*(1-m.cos(r*theta))-u[1]*m.sin(r*theta), u[1]*u[2]*(1-m.cos(r*theta)
                                                                      )+u[0]*m.sin(r*theta), u[2]*u[2]*(1-m.cos(r*theta))+m.cos(r*theta)]
    rot_u_r_theta = [a, b, c]
    # compute Rd(t)
    Rd = Ri.dot(rot_u_r_theta)
    # compute Xd
    Xd = np.array([0.0, 0.0, 0.0])
    
    D = np.linalg.norm(Xf-Xi)

    for i in range(0,3):
        Xd[i] = Xi[i] + r*(Xf[i]-Xi[i])
    

    #compute wd
    wd = np.dot(Ri,r_dot)
    wd = np.dot(wd,theta)
    wd = np.dot(wd,u)
    return Xd, Rd, r, r_dot, wd, D


if __name__ == "__main__":

    Xi = np.array([0, 0, 0]).reshape(3, 1)
    Xf = np.array([2, 2, 2]).reshape(3, 1)
    Ri = np.array([[10, 10, 10], [20, 20, 10], [10, 5, 1]])
    Rf = np.array([[100, 10, 10], [10, 1, 10], [10, 0, 10]])
    t = 0.1
    tf = 1
    Xd, Rd, r, r_dot, wd, D = generate_trajectory(Xi, Xf, Ri, Rf, t, tf)
    Rd.reshape(3,3)
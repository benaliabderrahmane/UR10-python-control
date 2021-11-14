from part1 import * 
from part2 import *
from part3 import * 

#initialization
Xi = np.array([0, 0, 0]).reshape(3, 1)
Xf = np.array([2, 2, 2]).reshape(3, 1)
Ri = np.array([[10, 10, 10], [20, 20, 10], [10, 5, 1]])
Rf = np.array([[100, 10, 10], [10, 1, 10], [10, 0, 10]])

t  = np.linspace(0,20,1000)





for ti in range (t):
    Xd, Rd, r, r_dot, wd= generate_trajectory(Xi, Xf, Ri, Rf, ti, 20)

    #get from coppeliasim
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = 0
    theta6 = 0

    T01,T02,T03,T04,T05,T06,X,R06 = DKM(theta1,theta2,theta3,theta4,theta5,theta6,  0,0,d6)
    Re = R06

    up = control_position(Xd, Xi, X,r_dot)

    uo = control_orientation(Rd,Re,wd)

    u = up + uo

    J01,J02,J03,J04,J05,J06,J,D = DDKM(T01,T02,T03,T04,T05,T06)

    Jinv = IDKM(J)

    qdot = Jinv*u

    #send qdot to coppeliasim
    



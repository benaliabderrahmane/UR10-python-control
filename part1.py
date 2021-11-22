'''
DKM / direct kinematic model
DDKM / direct diff model
IDKM / inverse diff model
'''
import math as m
import numpy as np
from icecream import ic

###############################parameters#####################################
a2 = 0.612
a3 = 0.5723
d1 = 0.1273
d4 = 0.163941
d5 = 0.1157
d6 = 0.0922 

def T(alpha, a, theta, r):
	T = np.matrix([
		[m.cos(theta),             -m.sin(theta),               0,             a],
		[m.cos(alpha)*m.sin(theta), m.cos(alpha)*m.cos(theta), -m.sin(alpha), -r*m.sin(alpha)],
		[m.sin(alpha)*m.sin(theta), m.sin(alpha)*m.cos(theta),  m.cos(alpha),  r*m.cos(alpha)],
		[0,                         0,                          0,             1]
	])
	return T

def Ji(sigmai,zi,pi):
	return (np.array([sigmai*(zi)+(not sigmai)*(np.cross(zi,pi)),(not sigmai)*zi])).reshape([6,1])

###############################DKM#####################################
def DKM(theta1,theta2,theta3,theta4,theta5,theta6,xo,yo,zo):
	T01 = T(0, 0, theta1-np.pi/2, d1)
	T12 = T(m.pi/2, 0, theta2-np.pi/2, d4)
	T23 = T(0, -a2, theta3, 0)
	T34 = T(0, -a3, theta4-np.pi/2, 0)
	T45 = T(m.pi/2, 0, theta5, d5)
	T56 = T(-m.pi/2, 0, theta6, 0)
    
	T02 = T01.dot(T12)
	T03 = T02.dot(T23)
	T04 = T03.dot(T34)
	T05 = T04.dot(T45)
	T06 = T05.dot(T56)
	position = T06.dot([xo, yo, zo, 1])
    
    
	R06 = T06[0:3,0:3]

	return T01,T02,T03,T04,T05,T06,position,R06

###############################DDM#####################################
def DDKM(T01,T02,T03,T04,T05,T06):
	"""
	this function calculate the geometric J
	"""
	sigma = 0
	z = np.array([T01[0:3,2],T02[0:3,2],T03[0:3,2],T04[0:3,2],T05[0:3,2],T06[0:3,2]])
	#p0n = np.array([T01[0:3,3],T02[0:3,3],T03[0:3,3],T04[0:3,3],T05[0:3,3],T06[0:3,3]])
	
	p01 =  np.array([T01[0:3,3]-T01[0:3,3],T01[0:3,3]-T02[0:3,3],T01[0:3,3]-T03[0:3,3],T01[0:3,3]-T04[0:3,3],T01[0:3,3]-T05[0:3,3],T01[0:3,3]-T06[0:3,3]])
	p02 =  np.array([T02[0:3,3]-T01[0:3,3],T02[0:3,3]-T02[0:3,3],T02[0:3,3]-T03[0:3,3],T02[0:3,3]-T04[0:3,3],T02[0:3,3]-T05[0:3,3],T02[0:3,3]-T06[0:3,3]])
	p03 =  np.array([T03[0:3,3]-T01[0:3,3],T03[0:3,3]-T02[0:3,3],T03[0:3,3]-T03[0:3,3],T03[0:3,3]-T04[0:3,3],T03[0:3,3]-T05[0:3,3],T03[0:3,3]-T06[0:3,3]])
	p04 =  np.array([T04[0:3,3]-T01[0:3,3],T04[0:3,3]-T02[0:3,3],T04[0:3,3]-T03[0:3,3],T04[0:3,3]-T04[0:3,3],T04[0:3,3]-T05[0:3,3],T04[0:3,3]-T06[0:3,3]])
	p05 =  np.array([T05[0:3,3]-T01[0:3,3],T05[0:3,3]-T02[0:3,3],T05[0:3,3]-T03[0:3,3],T05[0:3,3]-T04[0:3,3],T05[0:3,3]-T05[0:3,3],T05[0:3,3]-T06[0:3,3]])
	p06 =  np.array([T06[0:3,3]-T01[0:3,3],T06[0:3,3]-T02[0:3,3],T06[0:3,3]-T03[0:3,3],T06[0:3,3]-T04[0:3,3],T06[0:3,3]-T05[0:3,3],T06[0:3,3]-T06[0:3,3]])
	
	z = z.reshape([6,3])
    
	#p0n = p0n.reshape([6,3])
	p01 = p01.reshape([6,3])
	p02 = p02.reshape([6,3])
	p03 = p03.reshape([6,3])
	p04 = p04.reshape([6,3])
	p05 = p05.reshape([6,3])
	p06 = p06.reshape([6,3])

	z = np.transpose(z)
	#p0n = np.transpose(p0n)
	p01 = np.transpose(p01)
	p02 = np.transpose(p02)
	p03 = np.transpose(p03)
	p04 = np.transpose(p04)
	p05 = np.transpose(p05)
	p06 = np.transpose(p06)

	J01 = np.array([])
	J02 = np.array([])
	J03 = np.array([])
	J04 = np.array([])
	J05 = np.array([])
	J06 = np.array([])
    
	for i in range(6): #why having each Jij of 6 columns ?
		J01 = np.append(J01,Ji(sigma,z[:,i],p01[:,i]))
		J02 = np.append(J02,Ji(sigma,z[:,i],p02[:,i]))
		J03 = np.append(J03,Ji(sigma,z[:,i],p03[:,i]))
		J04 = np.append(J04,Ji(sigma,z[:,i],p04[:,i]))
		J05 = np.append(J05,Ji(sigma,z[:,i],p05[:,i]))
		J06 = np.append(J06,Ji(sigma,z[:,i],p06[:,i]))


	J01 = J01.reshape(6,6)
	J02 = J02.reshape(6,6)
	J03 = J03.reshape(6,6)
	J04 = J04.reshape(6,6)
	J05 = J05.reshape(6,6)
	J06 = J06.reshape(6,6)

	J01 = np.transpose(J01)
	J02 = np.transpose(J02)
	J03 = np.transpose(J03)
	J04 = np.transpose(J04)
	J05 = np.transpose(J05)
	J06 = np.transpose(J06)

	#D calculation
	R06 = T06[0:3,0:3]
	
	L = R06.dot(T06[0:3,3])
	L = np.asarray(L)

	D = np.array([
	[0,		     d6*T06[2,2], -d6*T06[1,2]],
	[-d6*T06[2,2], 0,          d6*T06[0,2]],
	[ d6*T06[1,2],-d6*T06[0,2],  0        ]
	])

	JJ1 = np.concatenate((np.eye(3),D), axis=1)
	JJ2 = np.concatenate((np.zeros((3,3)),np.eye(3)), axis=1)
	JJn = np.concatenate((JJ1,JJ2), axis=0)
	
	J = JJn.dot(J06)
    
	return J01,J02,J03,J04,J05,J06,J

###############################IDM#####################################
def IDKM(J):
    if np.linalg.det(J) != 0:  
        return np.linalg.inv(J)
    else:
        k = 2
        return J.transpose()*np.linalg.inv(J*J.transpose()+k**2*np.eye(6))
        
            
###############################test#####################################
if __name__ == "__main__":
	
	theta = 0
	T01,T02,T03,T04,T05,T06,position,R06 = DKM(0,0,0,0,0,0,  0,0,d6)

	print(position)
	np.set_printoptions(precision=4)
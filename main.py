try:
    import sim
except:
    print ('----------------ERROR. CHECK simpleTest.py----------------------------------------------')
    ####doc
'''
    simxGetObjectHandle(clientID, objectName, operationMode)

    simxGetJointPosition(clientID, jointHandle, operationMode)
    simxSetJointPosition(clientID, jointHandle, position, operationMode)
    simxSetJointTargetPosition(clientID, jointHandle, targetPosition, operationMode)
    
    simxGetJointMatrix(clientID, jointHandle, operationMode) 
    
    simxGetObjectOrientation(clientID, objectHandle, relativeToObjectHandle, operationMode)
    simxGetObjectPosition(clientID, objectHandle, relativeToObjectHandle, operationMode)
    simxSetObjectOrientation(clientID, objectHandle, relativeToObjectHandle, eulerAngles, operationMode)
    simxSetObjectPosition(clientID, objectHandle, relativeToObjectHandle, position, operationMode)
    
    UR10_ref = sim.simxGetObjectHandle(clientID, 'UR10',opmode)
    UR10_gripper = sim.simxGetObjectHandle(clientID, 'UR10_connection',opmode)
'''
################################################################################################################################
from posixpath import join
from re import A
from part1 import *
from part2 import *
from part3 import *
import time
import math
import numpy as np
from icecream import ic

print ('Program started, Zackq')
sim.simxFinish(-1)

################################################################################################################################
clientID = sim.simxStart('127.0.0.1',19998,True,True,5000,5) # Connect to CoppeliaSim
if clientID!= -1:
    print ('Connected to remote API server')


##########################################################  CODE   ######################################################################

    opmode = sim.simx_opmode_blocking
    
    def joints_handle(ClientID = clientID, mode = opmode):
        joints_id = [0,0,0,0,0,0] 
        joints_names = ['UR10_joint1#','UR10_joint2#','UR10_joint3#','UR10_joint4#','UR10_joint5#','UR10_joint6#']
        for i in range(6):
            _,joints_id[i] = sim.simxGetObjectHandle(ClientID, joints_names[i],mode)
        return joints_id

    def get_joint_orientation(joints_id, ClientID = clientID, relativeTo=-1, mode = opmode):
        pos = sim.simxGetObjectPosition(ClientID, joints_id, relativeTo, mode)[1]
        ori = sim.simxGetObjectOrientation(ClientID, joints_id, relativeTo, mode)[1]
        return pos,ori

    def set_joint_orientation(joints_id, angle, ClientID = clientID, mode = opmode):
        sim.simxSetJointTargetPosition(ClientID, joints_id, angle, mode)
        

    ## main
    
    joints_id = joints_handle()

    ##go to configuration 0
    print("go to config 0")
    
    for i in range(6):   
        set_joint_orientation(joints_id[i], 0, mode=opmode) 
    Xi, _= get_joint_orientation(joints_id[5], ClientID = clientID, relativeTo=-1, mode = opmode)
    Xi = np.array(Xi).reshape(3, 1)
    _ , T = sim.simxGetJointMatrix(clientID, joints_id[5], opmode) 
    Ri = [[T[0],T[1],T[2]],[T[3],T[4],T[5]],[T[6],T[7],T[8]]]
    Ri = np.array(Ri)
    time.sleep(3)
    
    ##go to configuration 1
    print("go to config 1")
    theta = [0, 0, 0.5, 0, 0, 10]
    for i in range(6):   
        set_joint_orientation(joints_id[i], theta[i], mode=opmode) 
    Xf, _= get_joint_orientation(joints_id[5], ClientID = clientID, relativeTo=-1, mode = opmode)
    Xf = np.array(Xf).reshape(3, 1)
    _ , T = sim.simxGetJointMatrix(clientID, joints_id[5], opmode) 
    Rf = [[T[0],T[1],T[2]],[T[3],T[4],T[5]],[T[6],T[7],T[8]]]
    Rf = np.array(Rf)
    time.sleep(2)

    ##re go to configuration 0
    print("re go to config 0")
    
    for i in range(6):   
        set_joint_orientation(joints_id[i], 0, mode=opmode)
        
        

    #time.sleep(5)

    sim.simxSynchronous(clientID,True)
    sim.simxSynchronousTrigger(clientID) 
    sim.simxStartSimulation(clientID, opmode)
    
    t  = np.linspace(0,20,100)
    tf = t[-1]
    deltaT = t[1]-t[0]

    Xd_1 = Xi
    Rd_1 = Ri
    qpast = [0,0,0,0,0,0]
    d6 = 0.0922 

    for ti in t[:-1]:
        #time.sleep(deltaT*10)
        Xd, Rd, r, r_dot, wd, D = generate_trajectory(Xi, Xf, Ri, Rf, ti, tf) 
        ##try control
        Xd = Xd.reshape(3,1)
        kp = 1
        up = control_position(Xd, Xd_1, D,r_dot, kp)
        ko = 1
        uo = control_orientation(Rd, Rd_1, wd, ko)
        #uo = [0,0,0]
        
        #uo = np.array(uo).reshape(3,1)

        u = np.concatenate((up,uo))
        #Jinv
        T01,T02,T03,T04,T05,T06,Xr,R06 = DKM(qpast[0], qpast[1], qpast[2], qpast[3], qpast[4], qpast[5],  0,0,d6)
        J01,J02,J03,J04,J05,J06,J = DDKM(T01,T02,T03,T04,T05,T06)
        Jinv = IDKM(J)
        qdot = np.dot(Jinv,u)
        q = []
        #integrate qdot 
        for i in range(6):
            q.append(((qpast[i]+deltaT*qdot[i]) + np.pi) % (2 * np.pi) - np.pi)
        
        #send control
        for i in range(6):
            set_joint_orientation(joints_id[i], q[i], mode=opmode)
        
        qpast = q
        Xd_1 = Xd
        Rd_1 = Rd
    
    #close.
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)


else:
    print ('Failed connecting to remote API server')
print ('Program ended')


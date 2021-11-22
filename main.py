try:
    import sim
except:
    print ('----------------ERROR. CHECK simpleTest.py----------------------------------------------')
    ####doc
'''
    simxGetObjectHandle(clientID, objectName, operationMode)
    <
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

    opmode = sim.simx_opmode_oneshot_wait
    
    def joints_handle(ClientID = clientID, mode = opmode):
        joints_id = [0,0,0,0,0,0,0] 
        joints_names = ['UR10_joint1#','UR10_joint2#','UR10_joint3#','UR10_joint4#','UR10_joint5#','UR10_joint6#','UR10_connection']
        for i in range(7):
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
    orientations = [0,0,0,0,0,0]
    positions = [0,0,0,0,0,0]

    ##go to configuration 0
    for i in range(6):   
        #print('-------------------------------------------------------------')
        set_joint_orientation(joints_id[i], 0, mode=sim.simx_opmode_blocking) 
        
    
    
    sim.simxSynchronous(clientID,True)
    sim.simxSynchronousTrigger(clientID) 
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)



    

    #main loop
    #initialization


    t  = np.linspace(0,5,100)
    tf = t[-1]
    qpast = [0,0,0,0,0,0]
    deltaT = 20/100
    Xi, _ = get_joint_orientation(joints_id[6]) #to check
    ic(Xi)
    
    
    q=[]
    for i in range(6):
        _,temp = sim.simxGetJointPosition(clientID,joints_id[i] , sim.simx_opmode_blocking)
        q.append(temp)
  
    
 

    
    _ , T = sim.simxGetJointMatrix(clientID, joints_id[0], opmode) #to check


    Ri = [[T[0],T[1],T[2]],[T[3],T[4],T[5]],[T[6],T[7],T[8]]]
    Ri = np.array(Ri)
    
    _,_,_,_,_,_,Xf,_ = DKM(np.pi/2, 0, np.pi/2, 0, np.pi/2, 0,  0,0,d6)
    Xf = np.array(Xf[0,0:3])
    Xf = np.concatenate(Xf)

    Rf = Ri
    
    X = Xi




    

    for ti in t[0:-1]:
        #get Xi from coppelia
        X, _ = get_joint_orientation(joints_id[-1]) #to check
        ic(X)
        Xd, Rd, r, r_dot, wd, D = generate_trajectory(Xi, Xf, Ri, Rf, ti, tf) 
        
        
        T01,T02,T03,T04,T05,T06,Xr,R06 = DKM(qpast[0], qpast[1], qpast[2], qpast[3], qpast[4], qpast[5],  0,0,d6)

        Re = R06
        #try to get Ti from copp
        J01,J02,J03,J04,J05,J06,J = DDKM(T01,T02,T03,T04,T05,T06)
        
        Jinv = IDKM(J)
        #X = np.array(Xr[0,0:3])
        #X = np.concatenate(X)
        
        #ic(X)
        up = control_position(Xd, X, D,r_dot)

        #uo = control_orientation(Re,Re,wd)
        
        uo = [1,1,1]



        u = np.concatenate((up,uo))
        
        
        qdot = np.dot(Jinv,u)
        
        q = []

        #integrate qdot 
        for i in range(6):
            q.append(qpast[i]+deltaT*qdot[i])
        qpast = q
        
        #send control
        for i in range(6):
            set_joint_orientation(joints_id[i], q[i], mode=sim.simx_opmode_blocking)
        
        
    #ic(Xf)
    #close.
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)


else:
    print ('Failed connecting to remote API server')
print ('Program ended')
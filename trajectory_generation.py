import numpy as np


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


def calculate_final_time(qf,qi):
    tfj = np.zeros(1,6)
    Dj = qf-qi
    for j in range(6):
        tfv = 15*np.abs(Dj)/(8*kv[j])
        tfa = np.sqrt(10*np.abs(Dj)/(np.sqrt(3)*ka[j]))
        tfj[j] = np.max(tfv,tfa)
    tf = max(tfj)
    return tf

def generate_r(t,qi,qf):
    tf = calculate_final_time(qf,qi)
    r = 10*(t/tf)**3 - 15*(t/tf)**4 + 6*(t/tf)**5
    return r

def generate_trajectory(position_i,position_f,Ri,Rf):

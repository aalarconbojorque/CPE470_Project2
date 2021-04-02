# -----------------------------------------------------------------------------
# FILE NAME:         proj2.py
# USAGE:             python3 proj2.py
# NOTES:             Requires NumPy installation
#                    Requires Python3
#
# MODIFICATION HISTORY:
# Author             Date           Modification(s)
# ----------------   -----------    ---------------
# Andy Alarcon       04-01-2021     1.0 ... setup dev environment, imported NumPy
# -----------------------------------------------------------------------------

import numpy.matlib as m
import numpy as np
import math
import random
import matplotlib.pyplot as plt


def main():
    
    #Initalize paramters
    n = 2  #Dimesions
    delta_t = 0.05 #TimeStep
    Lambda = 8.5 #Scaling factor  of attractive potential field
    pr_max = 50 #Max robot velocity
    t = np.arange(0,10,delta_t) #Total simulation time
    
    #Set Virtual Target
    qv = np.zeros((len(t), n)) #Initial positions of virtual target
    pv = 1.2 #Velocity of virtual target
    theta_t = np.zeros((len(t), 1)) #Initial heading of virtual target
    
    #Set ROBOT
    qr = np.zeros((len(t), n)) #Initial positon of robot
    pr = np.zeros((len(t), 1)) #Intiial velocity of robot
    theta_r = np.zeros((len(t), 1)) #Initial heading of the robot 

    #Set Relative states between robot and target
    qrv = np.zeros((len(t), n)) #Relative position between robot and target
    prv = np.zeros((len(t), n)) #Realative velocity between robot and target 
    #qrv[0] = [9,4], row 0 

    #Compute initial relative states between robot and target
    qrv[0] = qv[0] - qr[0]
    #Compute initial relative velocity
    prv[0]  = np.array([pv*np.cos(theta_t[0]) - pr[0]*np.cos(theta_r[0]), 
        pv*np.sin(theta_t[0]) - pr[0]*np.sin(theta_r[0])]).reshape(2,)
 
    #Set noise mean and standard deviation 
    noise_mean = 0.5
    noise_std = 0.5

    #Set Virtual Target Data
    for index, item in enumerate(qv, start=0):   
        if index == 0:
            pass
        else :
            #Set circular target tragectory (position)
            qv_x = 60 - 15 * np.cos(t[index])
            qv_y = 30 + 15 * np.sin(t[index])
            qv[index] = np.array([qv_x, qv_y]).reshape(2,)

            #Set heading of virtual target
            theta_t[index] = np.arctan2(qv_y, qv_x)


            #print(qv[index][1], theta_t[index][0], index)
    
    #Compute relative data
    for index, item in enumerate(qrv, start=0):   
        if index == 0:
            pass
        else :
            #Compute relative y pos V - R
            yrv = qv[index][1] - qr[index][1]
            #Compute relative x pos V - R
            xrv = qv[index][0] - qr[index][0]
            #Compute relative heading
            Hrv = np.arctan2(yrv, xrv)
            #Compute relative position
            qrv[index] = qv[index] - qr[index]
            
            #Compute velocity of the robot
            V_Left = CompMagSqr(pv)
            V_Middle = 2*Lambda*CompMag(qrv[index])*CompMag(pv)*CosSub(theta_t[index], Hrv)
            V_Right = np.square(Lambda)*CompMagSqr(qrv[index])
            pr[index] = np.sqrt(V_Left + V_Middle + V_Right)
            pr[index] = np.minimum(pr[index], pr_max)

            #Computer orientation of robot
            arcSin = (CompMag(pv)*np.sin(theta_r[index]- Hrv)) / CompMag(pr[index])
            theta_r[index] = Hrv + np.arcsin(arcSin)

            #Update position of robot 
            # theta_r[index-1]
            qr[index] = qr[index - 1] + pr[index] * delta_t * np.array([np.cos(theta_r[index-1]), np.sin(theta_r[index-1])]).reshape(2,)
            prv[index] = pv - pr[index]


            #print(qv[index][1], theta_t[index][0], index)


    f = open("output.txt", "a+")
    
    for index, item in enumerate(qrv, start=0):   
        f.write(str(qrv[index][0]) + '|' + str(qrv[index][1]) + '|' + str(theta_r[index][0]) + '\n')
    
    f.close()   

            

  
    print(prv[0])


def CompMagSqr(vec):
    #Compute magnitued
    vec = np.linalg.norm(vec)
    #Compute squared
    vec = np.square(vec)
    return vec

def CompMag(vec):
    #Compute magnitued
    vec = np.linalg.norm(vec)
    return vec

def CosSub(theta_1, theta_2):
    theta_3 = np.cos(theta_1 - theta_2)
    theta_3 = np.absolute(theta_3)
    return theta_3
    

    


    


if __name__ == "__main__":
    main()

# -----------------------------------------------------------------------------
# FILE NAME:         proj2.py
# USAGE:             python3 proj2.py
# NOTES:             Requires NumPy installation
#                    Requires Python3
#
# MODIFICATION HISTORY:
# Author             Date           Modification(s)
# ----------------   -----------    ---------------
# Andy Alarcon       04-01-2021     1.0 ... Setup dev environment, imported NumPy
# Andy Alarcon       04-03-2021     1.1 ... Initalized variables
# Andy Alarcon       04-05-2021     1.2 ... Implemented Equations
# Andy Alarcon       04-06-2021     1.3 ... Fixed bug that resulted innaccurate meas
# Andy Alarcon       04-07-2021     1.4 ... Added error and file writing 
# Andy Alarcon       04-09-2021     1.4 ... Calcualted data sets
# -----------------------------------------------------------------------------

import numpy.matlib as m
import numpy as np
import math
import random


def main():
    
    #Initalize paramters
    n = 2  #Dimesions
    delta_t = 0.05 #TimeStep
    Lambda = 8.5 #Scaling factor  of attractive potential field
    pr_max = 50 #Max robot velocity
    t = np.arange(0,20,delta_t) #Total simulation time 0-10 but 0-20 for sine wave

    #Set Error
    err = np.zeros((len(t), n))
    
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

    #Compute initial relative states between robot and target
    qrv[0] = qv[0] - qr[0]
    #Compute initial relative velocity
    prv[0]  = np.array([pv*np.cos(theta_t[0]) - pr[0]*np.cos(theta_r[0]), 
        pv*np.sin(theta_t[0]) - pr[0]*np.sin(theta_r[0])]).reshape(2,)
 
    #Set noise mean and standard deviation 
    noise_mean = 0.5
    noise_std = 0.5

    #Setup Virtual Target Data
    for index, item in enumerate(qv, start=0):   
        if index == 0:
            pass
        else :
            # #Set circular target tragectory (position)
            # qv_x = 60 - 15 * np.cos(t[index])
            # qv_y = 30 + 15 * np.sin(t[index])

            #Set linear target tragectory (position) WITHOUT NOISE
            # qv_x = 60 - 15 * t[index]
            # qv_y = 30 + 15 * t[index]
            
            #Set sin wave target stragectory WITHOUT NOISE
            # qv_x = t[index] + 10
            # qv_y = 15* np.sin(qv_x) + 10

            #Set linear target tragectory (position) WITH NOISE
            # qv_x = 60 - 15 * t[index] + noise_std * np.random.randn() + noise_mean
            # qv_y = 30 + 15 * t[index] + noise_std * np.random.randn() + noise_mean

            #Set sin wave target stragectory WITH NOISE
            qv_x = t[index] + 10 + noise_std * np.random.randn() + noise_mean
            qv_y = 15* np.sin(qv_x) + 10 + noise_std * np.random.randn() + noise_mean

            #Set heading of virtual target
            qv[index] = np.array([qv_x, qv_y]).reshape(2,)
            theta_t[index] = np.arctan2(qv_y, qv_x)

    
    #Compute relative data
    for index, item in enumerate(qrv, start=0):   
        if index == 0:
            pass
        else :
            #Compute relative y pos V - R
            yrv = qv[index][1] - qr[index-1][1]
            #Compute relative x pos V - R
            xrv = qv[index][0] - qr[index-1][0]
            #Compute relative heading
            Hrv = np.arctan2(yrv, xrv)
            #Compute relative position
            qrv[index] = qv[index] - qr[index-1]
            
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
            qr[index] = qr[index - 1] + pr[index] * delta_t * np.array([np.cos(theta_r[index-1]), np.sin(theta_r[index-1])]).reshape(2,)
            prv[index] = pv - pr[index]

            #Compute error
            no = qv[index] - qr[index]
            err[index] = np.linalg.norm(no)


    #Write error data to a file
    f = open("error.txt", "w")  
    for index, item in enumerate(qrv, start=0):  
        #Error
        f.write(str(err[index][0]) + '\n')  
    f.close()
            
    #Write robot data to a file
    f = open("robotPath.txt", "w") 
    for index, item in enumerate(qrv, start=0):  
        #X|Y|HEADING|VELOCITY 
        f.write(str(qr[index][0]) + '|' + str(qr[index][1]) + '|' + str(theta_r[index][0]) + '|' + str(pr[index][0]) + '\n')  
    f.close()

    
    f = open("targetPath.txt", "w")    
    for index, item in enumerate(qrv, start=0):
        #X|Y|HEADING    
        f.write(str(qv[index][0]) + '|' + str(qv[index][1]) + '|' + str(theta_t[index][0]) + '\n')  
    f.close()    
  
    print("Data for target, robot and error written")

    

# ----------------------------------------------------------------------------
# FUNCTION NAME:     CompMagSqr()
# PURPOSE:           Computes the magnitude squared
# -----------------------------------------------------------------------------
def CompMagSqr(vec):
    #Compute magnitued
    vec = np.linalg.norm(vec)
    #Compute squared
    vec = np.square(vec)
    return vec
# ----------------------------------------------------------------------------
# FUNCTION NAME:     CompMag()
# PURPOSE:           Computes the magnitude 
# -----------------------------------------------------------------------------
def CompMag(vec):
    #Compute magnitued
    vec = np.linalg.norm(vec)
    return vec
# ----------------------------------------------------------------------------
# FUNCTION NAME:     CosSub()
# PURPOSE:           Subtracts two values, takes the cos and returns abs val
# -----------------------------------------------------------------------------
def CosSub(theta_1, theta_2):
    theta_3 = np.cos(theta_1 - theta_2)
    theta_3 = np.absolute(theta_3)
    return theta_3
    
if __name__ == "__main__":
    main()

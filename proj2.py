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
    #cos_theta = np.cos(self.odo_o)
    cos_t = np.cos(theta_t[0])
    cos_r = np.cos(theta_r[0])
    sin_t = np.sin(theta_t[0])
    sin_r = np.sin(theta_r[0])
    init_r = pr[0]
    left_s = pv*cos_t - init_r*cos_r
    right_s = pv*sin_t - init_r*sin_r
    #y = np.array([left_s, right_s]).reshape(2,)
    y = np.array([pv*np.cos(theta_t[0]) - pr[0]*np.cos(theta_r[0]), pv*np.sin(theta_t[0]) - pr[0]*np.sin(theta_r[0])]).reshape(2,)
    prv[0] = y
    print(prv[0])


    

    


    


if __name__ == "__main__":
    main()

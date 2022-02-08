# -*- coding: utf-8 -*-
"""
Created on Thu Feb  3 21:57:15 2022

@author: Zee Almusa
"""

import numpy as np
from manipulator import Manipulator
from armplot import ArmPlot
import matplotlib.pyplot as plt


def main():
    
    joints  = np.zeros((6,4))

    theta = np.deg2rad(0)
    twist = np.pi/2
    # numpy array [[theta, d, a, alpha], ...]
    a0 = 700
    d1 = 335
    a1 = 75
    a2 = 365
    a3 = 90
    d4 = 405
    d6 = 80
    
  
    
    dh_params = np.array([[theta, 0, a0, 0 ],
                          [theta, d1, a1, -twist],
                          [theta, 0, a2, 0],
                          [theta, 0, a3, -twist],
                          [theta, d4, 0, twist],
                          [theta, 0, 0, -twist],
                          [theta, d6, 0,  0]])
    
    dh_params2 = np.array([
                          [theta, d1, 0, -twist],
                          [theta, 0, a2, 0],
                          [theta, 0, a3, -twist],
                          [theta, d4, 0, twist],
                          [theta, 0, 0, -twist],
                          [theta, d6, 0,  0]])
    
    
    joints = dh_params
    
    
    arm = Manipulator(joints)
    
    # plotter = ArmPlot(arm)
    
    # plotter.show()
    
    t = np.linspace(-0.5*np.pi, 3/2*np.pi, 50)
    
 
    
    x = 100*np.cos(t) 
    y = 100*np.sin(t) 
    z = 50*np.sin(t*5 )
    
    trajectory = np.zeros((t.shape[0], 3))  
    trajectory[:, 0] = x
    trajectory[:, 1] = y
    trajectory[:, 2] = z
        
   
    solution = arm.solve_trajectory(trajectory)
    solution = solution % (2*np.pi)
 
    plotter = ArmPlot(arm)
   
    plotter.animate(solution, trajectory)
    
    
    
    
    

   
    
if __name__ == "__main__":
    
    main()
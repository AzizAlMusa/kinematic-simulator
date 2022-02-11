# -*- coding: utf-8 -*-
"""
Created on Thu Feb  3 21:57:15 2022

@author: Zee Almusa
"""

import numpy as np
from manipulator import Manipulator
from armplot import ArmPlot
import matplotlib.pyplot as plt
import pandas as pd
import pickle
import copy
from math import sin, cos, atan2

def main():
    

    ###########################################
    #####             ARM LOAD            #####
    ###########################################
    arm7 = pickle.load( open( "denso_7.arm", "rb" ) )
    arm6 = pickle.load( open( "denso_6.arm", "rb" ) )
   

    ###########################################
    #####           VISUAL CHECK          #####
    ###########################################
    plotter = ArmPlot(arm6)
    
    plotter.show()
    
    
    
    ###########################################
    #####            TRAJECTORY           #####
    ###########################################
    
    #define your trajectory here
    t = np.linspace(-0.5*np.pi, 3/2*np.pi, 50)
    
   
    
    x = 0.15*np.cos(t)
    y = 0.15*np.sin(t) 
    z = 0.1*np.sin(5*t) + 0.33
    
    trajectory = np.zeros((t.shape[0], 3))  
    trajectory[:, 0] = x
    trajectory[:, 1] = y
    trajectory[:, 2] = z
        
    
    ###########################################
    #####          WAYPOINT SOLVER        #####
    ###########################################
    
    solution = arm7.solve_trajectory(trajectory)
    
    
    df = pd.DataFrame((solution.T))
    
    print("Saving waypoints to 'waypoints.csv'")
    df.to_csv('waypoints.csv')
    print(pd.DataFrame(np.rad2deg(solution.T)))
    plotter = ArmPlot(arm7)
    
    plotter.animate7(solution, trajectory)
    
    
    
    ###########################################
    #####          TURNTABLE DEMO         #####
    ###########################################
    
    df = pd.read_csv('waypoints.csv')
  
    solution = df.to_numpy()
    
    solution = solution.T[1:,:]
    
    plotter = ArmPlot(arm6)

    plotter.animate6(solution, trajectory)
    
    

   
    
if __name__ == "__main__":
    
    main()
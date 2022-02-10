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
    
 

    theta = np.deg2rad(0)
    twist = np.pi/2
    
    # numpy array [[theta, d, a, alpha], ...]
    y1 = -0.9
    
    x2 = 0.075
    z2 = 0.335
    z3 = 0.365
    x4 = -0.09
    z4 = 0.218
    
    z5 = 0.187
    
    z6 = 0.08
    
    
    joint_limits = ( (-6.283185, 6.283185),
                      (-2.96706,  2.96706),
                      (-1.745329, 2.356194),
                      (-2.076942, 2.949606),
                      (-3.316126, 3.316126),
                      (-2.094395, 2.094395),
                      (-6.283185, 6.283185))
    
    
    joint_params = [[0, 0, 0, 0, np.array([0, 0, 1])],
                    [0, 0, y1, 0, np.array([0, 0, 1])],
                    [0, x2, 0, z2 , np.array([0, 1, 0])],
                    [0, 0, 0, z3, np.array([0, 1, 0])],
                    [0, x4, 0, z4, np.array([0, 0, 1])],
                    [0, 0, 0, z5, np.array([0, 1, 0])],
                    [0, 0, 0, z6, np.array([0, 0, 1])]]
    
   
    
    
    
    
    # joint_limits = (  (-2.96706,  2.96706),
    #                   (-1.745329, 2.356194),
    #                   (-2.076942, 2.949606),
    #                   (-3.316126, 3.316126),
    #                   (-2.094395, 2.094395),
    #                   (-6.283185, 6.283185))
    
    # joint_params = [[0, 0, y1, 0, np.array([0, 0, 1])],
    #                 [0, x2, 0, z2 , np.array([0, 1, 0])],
    #                 [0, 0, 0, z3, np.array([0, 1, 0])],
    #                 [0, x4, 0, z4, np.array([0, 0, 1])],
    #                 [0, 0, 0, z5, np.array([0, 1, 0])],
    #                 [0, 0, 0, z6, np.array([0, 0, 1])]]
    
    
    
   
   
 
    ########### ARM CREATION ################
    joints = np.array(joint_params, dtype=object)
    arm = Manipulator(joints, joint_limits)
    

    
    ##################################
    ############ SINGLE PLOT #########
    #################################
    # plotter = ArmPlot(arm)
    
    # plotter.show()
    
    
    
    #################################
    ########## TRAJECTORY PLOT ######
    #################################
    
    
    t = np.linspace(-0.5*np.pi, 3/2*np.pi, 50)
    
   
    
    x = 0.15*np.cos(t)
    y = 0.15*np.sin(t) 
    z = 0.1*np.sin(5*t) + 0.33
    
    trajectory = np.zeros((t.shape[0], 3))  
    trajectory[:, 0] = x
    trajectory[:, 1] = y
    trajectory[:, 2] = z
        
   
    solution = arm.solve_trajectory(trajectory)
    
    
    df = pd.DataFrame((solution.T))
   
    df.to_csv('angles.csv')
    print(pd.DataFrame(np.rad2deg(solution.T)))
    plotter = ArmPlot(arm)
    
    plotter.animate7(solution, trajectory)
    
    
    
    ###########################################
    #####          TURNTABLE DEMO         #####
    ###########################################
    
    # df = pd.read_csv('angles.csv')
  
    # solution = df.to_numpy()
    
    # solution = solution.T[1:,:]
    
    # plotter = ArmPlot(arm)
    
    # ones = np.ones((trajectory.shape[0], 1))
    
    # trajectory = np.hstack((trajectory, ones))
    
    # def rot_turntable(theta, trajectory):
    #     R = np.array([[cos(theta), -sin(theta),      0, 0],
    #                   [sin(theta),  cos(theta) ,     0, 0],
    #                   [0         , 0           ,     1, 0],
    #                   [0         , 0           ,     0, 1]])
        
    #     new_trajectory = R @ trajectory.T
        
    #     return new_trajectory.T
    # waypoint = solution[25, :]
    # turtnable_angle = waypoint[0]
    # arm_waypoint = waypoint[1:]
    # trajectory = rot_turntable(-turtnable_angle, trajectory)
    # arm.update_state(arm_waypoint)
    # plotter.show(trajectory)
    
    # original_trajectory = copy.deepcopy(trajectory)
    # for i, waypoint in enumerate(solution):
        
    #     waypoint = solution[i, :]
    #     turtnable_angle = waypoint[0]
    #     arm_waypoint = waypoint[1:]
    #     trajectory = rot_turntable(-turtnable_angle, original_trajectory)
    #     arm.update_state(arm_waypoint)
    #     plotter.show(trajectory)
    
    # plotter.animate6(solution, trajectory)
    
    

   
    
if __name__ == "__main__":
    
    main()
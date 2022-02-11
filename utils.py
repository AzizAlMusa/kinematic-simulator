# -*- coding: utf-8 -*-
"""
Created on Fri Feb 11 02:16:05 2022

@author: User
"""
import numpy as np
import manipulator
import pickle
#PICKLE SAVE AND LOAD
# filehandler = open('denso_6.arm', 'wb') 
# pickle.dump(arm, filehandler)

#arm = pickle.load( open( "denso_7.arm", "rb" ) )


## ARM SPECS ##
# y1 = -0.61
# x2 = 0.075
# z2 = 0.335
# z3 = 0.365
# x4 = -0.09
# z4 = 0.218
 
# z5 = 0.187
 
# z6 = 0.08
 
 
   # joint_limits = ( (-6.283185, 6.283185),
   #                   (-2.96706,  2.96706),
   #                   (-1.745329, 2.356194),
   #                   (-2.076942, 2.949606),
   #                   (-3.316126, 3.316126),
   #                   (-2.094395, 2.094395),
   #                   (-6.283185, 6.283185))
 
 
   # joint_params = [[0, 0, 0, 0, np.array([0, 0, 1])],
   #                 [0, 0, y1, 0, np.array([0, 0, 1])],
   #                 [0, x2, 0, z2 , np.array([0, 1, 0])],
   #                 [0, 0, 0, z3, np.array([0, 1, 0])],
   #                 [0, x4, 0, z4, np.array([0, 0, 1])],
   #                 [0, 0, 0, z5, np.array([0, 1, 0])],
   #                 [0, 0, 0, z6, np.array([0, 0, 1])]]
 

 
 
 
 
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

# joints = np.array(joint_params, dtype=object)
# arm = manipulator.Manipulator(joints, joint_limits)
  
# filehandler = open('denso_6_actual.arm', 'wb') 
# pickle.dump(arm, filehandler)
  
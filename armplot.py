# -*- coding: utf-8 -*-
"""
Created on Thu Feb  3 21:59:05 2022

@author: Zee Almusa
"""

import numpy as np

import matplotlib as mpl
#mpl.rc('text', usetex = False)
#mpl.rc('font', family = 'serif')
#mpl.rcParams.update(mpl.rcParamsDefault)

import matplotlib.pyplot as plt
# plt.rcParams.update({
#   "text.usetex": True,
#   "font.family": "Helvetica"
# })

import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import manipulator
from math import cos, sin

from datetime import datetime
import copy
import pdb

class ArmPlot:
    
    def __init__(self, arm):
        
        self.arm = arm
        
        self.colors = ['pink', 'red', 'darkorange', 'gold', 'blue', 'indigo', 'black']
        
        
        
    
    def transform_to_position(self, transform):
        
        return transform[:3, 3]
    
    
    #returns the end points of each link
    def get_endpoints(self):   
        
        T_chain = self.arm.get_forward_chain()
        
        n = self.arm.state.shape[0] 
        
        end_points = np.zeros((n, 3))
      
        for i in range(T_chain.shape[0]):
            
            end_points[i , :] = self.transform_to_position(T_chain[i, :, :])
        
        return end_points
    
    #formatting only
    def format_fig(self, axes):
        
        
        
        #plt.style.use(['ggplot','dark_background'])
       
        for i, ax in enumerate(axes):
            ax.axis('auto')
            
            ax.set_xlim(-1.0,1.0)
            ax.set_ylim(-1.0, 1.0)
            ax.set_zlim(-1.0, 1.0)
            ax.grid()
            ax.set_xlabel('x', fontsize=24)
            ax.set_ylabel('y', fontsize=24)
            ax.set_zlabel('z', fontsize=24)
            
            if i == 0:
                ax.view_init(azim=0, elev=90)
                
        
        
      
        
       
        
        
    
    #plot the arm
    def show(self, trajectory=None):
        
        end_points = self.get_endpoints()
        
        fig = plt.figure(figsize=(1920/72, 1080/72), dpi=72)
        fig.suptitle('Virtual Equivalent Trajectory', fontsize=38 )
        
        ax = plt.subplot(1,2,1, projection='3d')
       
        ax2 = plt.subplot(1,2,2, projection='3d')
        
        self.format_fig([ax, ax2])
        
        if type(trajectory) is np.ndarray:
            ax.plot3D(trajectory[:,0], trajectory[:, 1], trajectory[:,2], 'blue', linestyle="dashed")
            ax2.plot3D(trajectory[:,0], trajectory[:, 1], trajectory[:,2], 'blue', linestyle="dashed")
            
        for i in range(end_points.shape[0]-1):
            current = ax.plot3D(end_points[i:i+2, 0], end_points[i:i+2, 1], end_points[i:i+2, 2],self.colors[i])
            current[0].set_linewidth(10)
            
            current = ax2.plot3D(end_points[i:i+2, 0], end_points[i:i+2, 1], end_points[i:i+2, 2],self.colors[i])
            current[0].set_linewidth(10)
        
        plt.show()


    





    def animate7(self, solution, trajectory):
        
                           
        frame_count = solution.shape[0]
        end_points = self.get_endpoints()
        
        fig = plt.figure(figsize=(1920/72, 1080/72), dpi=72)
        fig.suptitle('Virtual Equivalent Trajectory', fontsize=44)
        ax = plt.subplot(1,2,1, projection='3d')
        ax2 = plt.subplot(1,2,2, projection='3d')
        
        #fig.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
        
        axes = [ax, ax2]
        self.format_fig(axes)
        
        N = end_points.shape[0]-1
        
        line = [ax.plot3D([], [], [])[0] for _ in range(N)] #lines to animate
        
        line2 = [ax2.plot3D([], [], [])[0] for _ in range(N)] #lines to animate
        
        lines = line + line2
        
        if type(trajectory) is np.ndarray:
            
            guide = ax.plot3D(trajectory[:,0], trajectory[:, 1], trajectory[:,2], 'blue', linestyle="dashed")
            guide2 = ax2.plot3D(trajectory[:,0], trajectory[:, 1], trajectory[:,2], 'blue', linestyle="dashed")
              
                     
        def update(i):
              
   
              current_state = solution[i, :]
              self.arm.update_state(current_state)
              end_points = copy.deepcopy(self.get_endpoints())
           
              
             
              
              for j in range(N):
                  
                  lines[j].set_data(end_points[j:j+2, 0], end_points[j:j+2, 1])
                  lines[j].set_3d_properties(end_points[j:j+2,2])
                  
                  lines[j].set_linewidth(10)
                  lines[j].set_color(self.colors[j])
                  
                  lines[j+N].set_data(end_points[j:j+2, 0], end_points[j:j+2, 1])
                  lines[j+N].set_3d_properties(end_points[j:j+2,2])
                  
                  lines[j+N].set_linewidth(10)
                  lines[j+N].set_color(self.colors[j])
  
                 
                  
              return lines
      
            
         
                
                
     
        anim = FuncAnimation(fig, update, frames=frame_count, interval=20, blit=True)
   
        writergif = animation.PillowWriter(fps=12) 
        
        now = datetime.now()
        stamp = str(now.date()) + str(now.hour) + str(now.minute) + str(now.second)
        anim.save('animations/7DOF ARM_' + stamp + '_.gif', writer=writergif)







    def animate6(self, solution, trajectory):
        
                           
        frame_count = solution.shape[0]
        end_points = self.get_endpoints()
        
        fig = plt.figure(figsize=(1920/72, 1080/72), dpi=72)
        fig.suptitle('Actual Trajectory', fontsize=16)
        ax = plt.subplot(1,2,1, projection='3d')
        ax2 = plt.subplot(1,2,2, projection='3d')
        
       
        
        axes = [ax, ax2]
        self.format_fig(axes)
        
        N = end_points.shape[0]-1
        
        line = [ax.plot3D([], [],[])[0] for _ in range(N)] #lines to animate
        
        line2 = [ax2.plot3D([], [],[])[0] for _ in range(N)] #lines to animate
        
        lines = line + line2
        
        if type(trajectory) is np.ndarray:
            
            guide = ax.plot3D(trajectory[:,0], trajectory[:, 1], trajectory[:,2], 'blue', linestyle="dashed")
            guide2 = ax2.plot3D(trajectory[:,0], trajectory[:, 1], trajectory[:,2], 'blue', linestyle="dashed")
      
        
        
        ones = np.ones((trajectory.shape[0], 1))
        trajectory = np.hstack((trajectory, ones))
        original_trajectory = copy.deepcopy(trajectory)
        
        def rot_turntable(theta, trajectory):
            
            R = np.array([[cos(theta), -sin(theta),      0, 0],
                          [sin(theta),  cos(theta) ,     0, 0],
                          [0         , 0           ,     1, 0],
                          [0         , 0           ,     0, 1]])
            
            new_trajectory = R @ trajectory.T
            
            return new_trajectory.T
            
            
        def update(i):
              
              waypoint = solution[i, :]
              turtnable_angle = waypoint[0]
              arm_waypoint = waypoint[1:]
              trajectory = rot_turntable(-turtnable_angle, original_trajectory)
           
              #self.arm.update_state(arm_waypoint)
              
       
              current_state = solution[i, :]
              self.arm.update_state(arm_waypoint)
              end_points = copy.deepcopy(self.get_endpoints())
              
              
              guide[0].set_data(trajectory[:,0], trajectory[:, 1])
              guide[0].set_3d_properties(trajectory[:,2])
              guide[0].set_linewidth(1)
              guide[0].set_color('blue')
              
              guide2[0].set_data(trajectory[:,0], trajectory[:, 1])
              guide2[0].set_3d_properties(trajectory[:,2])
              guide2[0].set_linewidth(1)
              guide2[0].set_color('blue')
              
              for j in range(N):
                  lines[j].set_data(end_points[j:j+2, 0], end_points[j:j+2, 1])
                  lines[j].set_3d_properties(end_points[j:j+2,2])
                  
                  lines[j].set_linewidth(10)
                  lines[j].set_color(self.colors[j+1])
                  
                  lines[j+N].set_data(end_points[j:j+2, 0], end_points[j:j+2, 1])
                  lines[j+N].set_3d_properties(end_points[j:j+2,2])
                  
                  lines[j+N].set_linewidth(10)
                  lines[j+N].set_color(self.colors[j+1])
  
                 
                  
              return lines
      
            
         
                
                
     
        anim = FuncAnimation(fig, update, frames=frame_count, interval=20, blit=True)
   
        writergif = animation.PillowWriter(fps=12) 
        
        now = datetime.now()
        stamp = str(now.date()) + str(now.hour) + str(now.minute) + str(now.second)
        anim.save('animations/6DOF ARM_' + stamp + '_.gif', writer=writergif)
        
        
        
            
           
            
            
            
            
     
           
        
        
        
        
        
        
        
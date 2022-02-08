# -*- coding: utf-8 -*-
"""
Created on Thu Feb  3 21:59:05 2022

@author: Zee Almusa
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import manipulator

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
        
        n = self.arm.state.shape[0] + 1
        
        end_points = np.zeros((n, 3))
      
        for i in range(T_chain.shape[0]):
            
            end_points[i + 1, :] = self.transform_to_position(T_chain[i, :, :])
        
        return end_points
    
    #formatting only
    def format_fig(self, ax):
        
        ax.axis('auto')
        ax.set_xlim(-500,500)
        ax.set_ylim(-500, 500)
        ax.set_zlim(-500, 500)
        ax.grid()
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        
        ax.view_init(azim=0, elev=90)
        
    
    #plot the arm
    def show(self, trajectory=None):
        
        end_points = self.get_endpoints()
        
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        
        self.format_fig(ax)
        
        if type(trajectory) is np.ndarray:
            ax.plot3D(trajectory[:,0], trajectory[:, 1], trajectory[:,2], 'blue', linestyle="dashed")
        
        for i in range(end_points.shape[0]-1):
            ax.plot3D(end_points[i:i+2, 0], end_points[i:i+2, 1], end_points[i:i+2, 2],self.colors[i])
           
        
        plt.show()




    def animate(self, solution, trajectory):
        
       
        
        frame_count = solution.shape[0]
        end_points = self.get_endpoints()
        
        fig = plt.figure(figsize=(1920/72, 1080/72), dpi=72)
        
        ax = plt.axes(projection='3d')     
        self.format_fig(ax)
        
        N = end_points.shape[0]-1
        lines = [ax.plot3D([], [],[])[0] for _ in range(N)] #lines to animate
        
        if type(trajectory) is np.ndarray:
            ax.plot3D(trajectory[:,0], trajectory[:, 1], trajectory[:,2], 'blue', linestyle="dashed")
                
        def update(i):
            
              current_state = solution[i, :]
              self.arm.update_state(current_state)
              end_points = copy.deepcopy(self.get_endpoints())
              
              
              for j in range(end_points.shape[0]-1):
                  lines[j].set_data(end_points[j:j+2, 0], end_points[j:j+2, 1])
                  lines[j].set_3d_properties(end_points[j:j+2,2])
                  
                  lines[j].set_linewidth(10)
                  lines[j].set_color(self.colors[j])
                  
              return lines
        
     
        anim = FuncAnimation(fig, update, frames=frame_count, interval=20, blit=True)
        
        writergif = animation.PillowWriter(fps=12) 
        anim.save('test.gif', writer=writergif)
        
        
        
            
           
            
            
            
            
     
           
        
        
        
        
        
        
        
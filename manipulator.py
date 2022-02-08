"""
Author: Zee Almusa
A simple revolute open chain manipulator
"""

import numpy as np
from math import sin, cos
import copy
import pdb


class Manipulator:
      
    def __init__(self, joints, state = None):
        
        
        # denavit-hartenberg structure of joints
        # numpy array [[theta, a, d, alpha], ...]
        self.joints = joints
        
        # numpy array of joint angles
        self.state = joints[:, 0]
        
        # kinematic chain
        self.transform_chain = self.get_forward_chain()
    
    
    # update angles on joints and corresponding position
    def update_state(self, new_state):
        #update state
        self.state = new_state
        
        #reflect in joints
        self.joints[:, 0] = new_state
       
        
        #reflect in chain
        self.transform_chain = self.get_forward_chain()
        
        
    # get the homogeneous transform for a joint (not cumulative transform)
    def joint_to_transform(self, joint):
       
        theta, d, a, alpha = joint[0], joint[1], joint[2], joint[3]
        
        # transform from frame i-1 to i: R_z*Trans_z*Trans_x*R_x
        T = np.array([[cos(theta), -sin(theta) * cos(alpha), sin(theta)*sin(alpha) , a*cos(theta)],
                      [sin(theta),  cos(theta) * cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                      [0         ,  sin(alpha)             , cos(alpha)            ,            d],
                      [0         ,  0                      , 0                     ,            1]])
        return T
    
    
    # cumulative homogoneous transform map for all the joints
    def get_forward_chain(self, joints=None):
        
        if not type(joints) is np.ndarray:
            joints = self.joints
            
        # shape: joints x [4 x 4] 
        # where i x [4 x 4] is the ith joint transform from origin up the chain
        transform_chain = np.zeros((len(joints), 4, 4))
        composite_transform = np.eye(4,4)
        
        for i, joint in enumerate(joints):
            
            composite_transform = composite_transform @ self.joint_to_transform(joint)
            transform_chain[i, :, :] = composite_transform
        
        return transform_chain
    

    # get end-effector transform
    # shape: 4 x 4
    def get_forward(self, joints=None):   
        if not type(joints) is np.ndarray:
            return self.transform_chain[-1, : , :]
   
        return self.get_forward_chain(joints)[-1, : , :]
 
    
    # get end-effector position
    # shape: 3 x 1
    def get_position(self, joints=None):        
        return self.get_forward(joints)[:3, 3].reshape(-1,1)
    
    # for RR arm only for debugging
    # theta velocities -> position velocities
    # shape: 3 x 1
    def get_true_jacobian(self):
        theta1, theta2 = self.joints[0,0],self.joints[1,0] 
        l1, l2 =  self.joints[0,1],self.joints[1,1] 
        
        J = np.array([[-l1 * sin(theta1) - l2*sin(theta1+theta2), -l2*sin(theta1 + theta2) ],
                      [l1 * cos(theta1) + l2*cos(theta1+theta2) ,     l2*cos(theta1+theta2)]])
        
        return J
        
    
    # numerical jacobian using forward difference
    # shape: 3 x 1
    def get_jacobian(self, joints=None, state=None):
        
        #if no joints/state are provided get the current jacobian of arm
        if not type(joints) is np.ndarray: 
            joints = copy.deepcopy(self.joints)
            
        if not type(state) is np.ndarray:
            state = copy.deepcopy(self.state)
            
        #else just get the current arm jacobian
        
        J = np.zeros((3, joints.shape[0]))
        
        d_theta = 0.001
        
        position_0 = self.get_position()
        
        
                             
        for i, theta in enumerate(state):
            
            # perturb angle in ith joint
            state_1 = copy.deepcopy(state)
            state_1[i] += d_theta

            # create corresponding virtual perturbed arm
            joints_1 = copy.deepcopy(joints)
            joints_1[:, 0] = state_1
            
            
            
            # get forward kinematics and position of the perturbed arm
            T_1 = self.get_forward(joints_1)
            position_1 = T_1[:3, 3].reshape(-1,1)
           
            #find forward difference (numerical derivative)
            forward_diff = (position_1 - position_0) / d_theta
            
            J[:, i] = forward_diff.flatten()
        
        return J
    
    
    # get joint angles for desired position
    # does not update state
    # shape: 1-D ndarray (no. joints,)
    def get_inverse(self, desired, initial_state=None):
        
        error = desired - self.get_position()
        
        #Need a scheduler for the gradient rate
        rate = 0.1
        
        i = 0
        
        #virtual copies of arm
        if not initial_state is np.ndarray:
            state_1 = copy.deepcopy(self.state).reshape(-1, 1)
        else:
            state_1 = initial_state
            
        joints_1 = copy.deepcopy(self.joints)
      
        while np.linalg.norm(error) > 1e-3:
            
            J = self.get_jacobian()
            gradient = np.linalg.pinv(J) @ error
            
            # TODO check why sign is positive here
            state_1 = state_1 + rate * gradient
           
            joints_1[:, 0] = state_1.flatten()
            
            #self.update_state(new_state)
            error = desired - self.get_position(joints_1)
        
            
            i += 1
            if i > 1000:
                break
       
        return state_1.flatten()
    
    
    #Need to debug
    def solve_trajectory(self, trajectory):
        
        num_points = trajectory.shape[0]
        num_joints = self.joints.shape[0]
        
        angle_solution = np.zeros((num_points, num_joints))
        print(angle_solution.shape)
        
        prev_state = self.state
        for i in range(num_points):
            
            desired = trajectory[i, :].reshape(-1,1)
            new_state = self.get_inverse(desired)
            angle_solution[i, :] = new_state
            self.update_state(new_state)
    
        
        return angle_solution
    
        
        
    
    
    
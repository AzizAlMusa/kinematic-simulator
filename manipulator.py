"""
Author: Zee Almusa
A simple revolute open chain manipulator
"""

import numpy as np
from math import sin, cos, atan2, asin
import copy
import pdb

import scipy.optimize as optimize

class Manipulator:
      
    def __init__(self, joints, joint_limits, state = None):
        
        
        # denavit-hartenberg structure of joints
        # numpy array [[theta, a, d, alpha], ...]
        self.joints = joints
        
        self.joint_limits = joint_limits
        
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
       
        theta, x, y, z , axis= joint[0], joint[1], joint[2], joint[3], joint[4]
        
       
        
        rotation_axis = np.nonzero(axis)[0][0]
        # transform from frame i-1 to i: R_z*Trans_z*Trans_x*R_x
        if    rotation_axis == 0:
            R = np.array([[1,          0,              0, 0],
                          [0, cos(theta),    -sin(theta), 0],
                          [0, sin(theta),     cos(theta), 0],
                          [0,          0,              0, 1]])
            
        elif  rotation_axis == 1:
            
            R = np.array([[cos(theta), 0,     -sin(theta), 0],
                          [0,          1,               0, 0],
                          [sin(theta), 0,      cos(theta), 0],
                          [0         , 0          ,     0, 1]])
        
        elif  rotation_axis == 2:
            R = np.array([[cos(theta), -sin(theta),      0, 0],
                          [sin(theta),  cos(theta) ,     0, 0],
                          [0         , 0           ,     1, 0],
                          [0         , 0           ,     0, 1]])
        
        T = np.eye(4,4)
        T[:3, 3] = np.array([x, y ,z])
        
        g = T @ R
        
        return g
    
    
    # cumulative homogoneous transform map for all the joints
    def get_forward_chain(self, joints=None):
        
        #if real joints used
        if not type(joints) is np.ndarray:
            joints = self.joints
            
        # shape: joints x [4 x 4] 
        # where i x [4 x 4] is the ith joint transform from origin up the chain
        transform_chain = np.zeros((joints.shape[0], 4, 4))
        composite_transform = np.eye(4,4)
        
        for i, joint in enumerate(joints):
            
            composite_transform = composite_transform @ self.joint_to_transform(joint)
            transform_chain[i, :, :] = composite_transform
        
        return transform_chain
    
    
    

    # get end-effector transform
    # shape: 4 x 4
    def get_forward(self,   joints=None, joint_idx=-1,):   
           
        return self.get_forward_chain(joints)[joint_idx, : , :]
    
    
    # get end-effector position
    # shape: 3 x 1
    def get_position(self, joints=None, joint_idx=-1):    
        
        
        # If real joints used
        if not type(joints) is np.ndarray:
            joints = self.joints
        
        return self.get_forward(joints, joint_idx)[:3, 3].reshape(-1,1)
        
    
    
    def matrix_to_euler(self, R):
        
        roll1, roll2 = None, None
        pitch1, pitch2 = None, None
        yaw1, yaw2 = None, None
        
        if np.abs(R[2,0]) != 1:
            # TODO Complete this
           
            roll1 = -asin(R[2,0])
            roll2 = np.pi - roll1
            
            pitch1 = atan2(R[2,1]/cos(roll1), R[2,2]/cos(roll1))
            pitch2 = atan2(R[2,1]/cos(roll2), R[2,2]/cos(roll2))
            
            yaw1 = atan2(R[1,0]/roll1, R[0,0]/roll1)
            yaw2 = atan2(R[1,0]/roll2, R[0,0]/roll2)
           
        else:
            yaw1 = 0
            
            if R[2,0] == -1:
                roll1 = np.pi/2
                pitch1 = yaw1 + atan2(R[0,1], R[0,2])
            
            else:
                roll1 = -np.pi/2
                pitch1 = -yaw1 + atan2(-R[0,1], -R[0,2])
            
        
        angles1 = np.array([roll1, pitch1, yaw1]).reshape(-1,1)
        angles2 = np.array([roll2, pitch2, yaw2]).reshape(-1,1)
        return angles1, angles2
        
            
    
    def get_orientation(self, joints=None, joint_idx=-1):
        
        if not type(joints) is np.ndarray:
            joints = self.joints
        
        
        transform = self.get_forward(joints, joint_idx)
        R = transform[0:3,0:3]
        #pdb.set_trace()
        angles1, angles2 = self.matrix_to_euler(R)
             
    
        return angles1, angles2
    
    
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
        
        position_0 = self.get_position(joints)
        
        
                             
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
    
    # cost function for optimizer to reach desired x, y, z
    def cost_function(self, state, desired, joints_1):
        
        joints_1[:, 0] = state
        error = desired - self.get_position(joints_1)
        
        return np.linalg.norm(error)
    
        
    def extension_allowance(self, state, joints):
        reference = self.get_position(joints, 1)[:2,0]
        end_effector =  self.get_position(joints)[:2,0]
        extension = np.linalg.norm(end_effector - reference)
        
        return -(extension - 0.45)
    
    def extension_allowance2(self, state, joints):
        reference = self.get_position(joints, 1)[:2,0]
        end_effector =  self.get_position(joints)[:2,0]
        extension = np.linalg.norm(end_effector - reference)
        
        return (extension - 0.43)
    
       
    
    # get joint angles for desired position
    # this uses scipy's optimizer
    # does not update state
    # shape: 1-D ndarray (no. joints,)
    def get_inverse2(self, desired, initial_state=None):
           
        #virtual copies of arm
        if not initial_state is np.ndarray:
            state_1 = copy.deepcopy(self.state).reshape(-1, 1)
        else:
            state_1 = initial_state
            
        joints_1 = copy.deepcopy(self.joints)
        
        # TODO add minimizer here
        
        ## constraint format
        #  cons=({'type': 'ineq','fun': lambda x: x[0]},
        #        {'type': 'ineq', 'fun': lambda x: x[0] },
        #{'type': 'ineq', 'fun': lambda x: x[2] + x[3] + x[5] - np.deg2rad(120)},
        #        {'type': 'eq', 'fun': lambda x: x[4]}
        #cons = ({'type': 'ineq', 'fun': self.extension_allowance, 'args': (joints_1,)})
        # constraints=cons
       
        limits = self.joint_limits
        optim = optimize.minimize(self.cost_function, state_1, args=(desired, joints_1) ,  bounds=limits)
        #print(optim.x)
        return optim.x
    
    
    #Need to debug
    def solve_trajectory(self, trajectory):
        
        num_points = trajectory.shape[0]
        num_joints = self.joints.shape[0]
        
        angle_solution = np.zeros((num_points, num_joints))
        print(angle_solution.shape)
        
        prev_state = self.state
        for i in range(num_points):
            
            desired = trajectory[i, :].reshape(-1,1)
            new_state = self.get_inverse2(desired)
            angle_solution[i, :] = new_state
            self.update_state(new_state)
            
            
         
        print('Solution found.')
        
        return angle_solution
    
        
        
    
    
    
#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import Axes3D
from gmlaas.msg import PreHMsg
import time
import tf
class EstimateHMatrix:
    def __init__(self) :
        rospy.init_node("h_computation_py",anonymous=False)
        self.m_points_sub = rospy.Subscriber("/h_computation/input_matrices",PreHMsg,self.callback)
        self.m_h_matrix_pub = rospy.Publisher("/h_computation/h_matrix",Float32MultiArray,queue_size=10)
        

    def buildInputMatrices(self,msg):
        self.current_points = np.array(msg.current_coordinates).reshape(int(len(msg.current_coordinates)/3),3)
        self.desired_points = np.array(msg.desired_coordinates).reshape(int(len(msg.desired_coordinates)/3),3)


    def estimateRTMatrices(self):
        """Estimates the pose (rotation and translation) that best aligns the current points to the desired points."""
        centroid_current = np.mean(self.current_points, axis=0)
        centroid_desired = np.mean(self.desired_points, axis=0)
        centered_current = self.current_points - centroid_current
        centered_desired = self.desired_points - centroid_desired
        H = np.dot(centered_current.T, centered_desired)
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)
        T = centroid_desired.T - np.dot(R, centroid_current.T)
        return R, T
    
    def buildHmatrix(self,R, T):
        # Initialize a 4x4 identity matrix
        H = np.eye(4)
        # Set the upper left 3x3 block to the rotation matrix
        H[:3, :3] = R
        # Set the upper right 3x1 block to the translation vector
        H[:3, 3] = T
        return H
    
    def publishMatrix(self,H):
        # Publish the current and desired coordinates as a PreHMsg message
        h_matrix_msg = Float32MultiArray()
        h_matrix_msg.data = H.flatten()
        self.m_h_matrix_pub.publish(h_matrix_msg)

    def callback(self,msg):
        self.buildInputMatrices(msg) 
        R_est, T_est = self.estimateRTMatrices()
        new_rotation= np.zeros((4,4))
        new_rotation[:3,:3] = R_est
        new_rotation[3,3] = 1
        # Convert the rotation matrix to euler angles
        degree= tf.transformations.euler_from_matrix(new_rotation)
        degree=np.degrees(degree) 
        H = self.buildHmatrix(R_est, T_est)        
    
if __name__ == "__main__":
    estimate_h_matrix=EstimateHMatrix()
    rospy.spin()

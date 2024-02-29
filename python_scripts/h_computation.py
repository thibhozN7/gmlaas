#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import Axes3D
from gmlaas.msg import PreHMsg

class EstimateHMatrix:
    def __init__(self) :
        rospy.init_node("estimate_h_matrix",anonymous=False)
        rospy.loginfo("Starting H computation...")
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


    #Visualize functions
    def applyTransformation(self,R, T):
        """Apply rotation and translation to the points."""
        transform = np.dot(self.current_points, R.T) + T
        return transform

    def visualizePoints(self):
        """Visualize the current, desired, and estimated points in a 3D plot."""
        current = np.array(self.current_points)
        desired = np.array(self.desired_points)
        estimated = np.array(self.estimated_points)
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(current[:, 0], current[:, 1], current[:, 2], c='r', label='current')
        ax.scatter(desired[:, 0], desired[:, 1], desired[:, 2], c='g', label='desired')
        ax.scatter(estimated[:, 0], estimated[:, 1], estimated[:, 2], c='b', label='Estimated')

        # Plotting the link between current and desired points
        for i in range(len(current)):
            ax.plot([current[i, 0], desired[i, 0]], [current[i, 1], desired[i, 1]], [current[i, 2], desired[i, 2]], c='k')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()
    
    def visualizeError(desired, estimated):
        """Visualize the error between the desired and estimated points."""
        error = np.linalg.norm(desired - estimated, axis=1)
        max_error = np.max(error)
        min_error = np.min(error)
        mae_error = np.mean(error)
        mape_error = np.mean(np.abs(np.divide((desired - estimated),desired)))*100
        print(f"Max Positional Error: {max_error}")
        print(f"Min Positional Error: {min_error}")
        print(f"MAE Positional Error: {mae_error}")
        print(f"MAPE Positional Error (%): {mape_error}")

    def callback(self,msg):
        self.buildInputMatrices(msg) 
        rospy.loginfo("Input matrices received.")
        rospy.loginfo("Estimating pose...")
        R_est, T_est = self.estimateRTMatrices()
        rospy.loginfo("Pose estimation completed.")
        rospy.loginfo("Building H matrix...")
        H = self.buildHmatrix(R_est, T_est)
        self.publishMatrix(H)
        rospy.loginfo("H matrix published.")
        
        #visualize the points and the error
        self.estimated_points = self.applyTransformation( R_est, T_est)
        #self.visualizePoints()
        #self.visualizeError(self.m_desired_points, self.m_estimated_points)
        
    
if __name__ == "__main__":
    estimate_h_matrix=EstimateHMatrix()
    rospy.spin()

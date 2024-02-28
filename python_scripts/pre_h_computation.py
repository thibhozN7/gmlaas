import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from message_filters import Subscriber, TimeSynchronizer
from gmlaas.msg import CustomMsg 
from gmlaas.msg import PreHMsg
from gmlaas.msg import GraphMatcherMsg
from apriltag_ros.msg import AprilTagDetectionArray

import os
import csv

# Get the current directory and package directory
current_dir = os.path.realpath(__file__)
package_dir = os.path.dirname(os.path.dirname(current_dir))

class PreHMatrix:
    def __init__(self):
        rospy.init_node("pre_h_matrix", anonymous=False)

        # Define subscribers
        self.m_graph_sub = rospy.Subscriber("/graph_matching/data", GraphMatcherMsg,self.callback)
        # self.m_tag_sub = Subscriber("/tag_detections", AprilTagDetectionArray)

        # # Synchronize subscribers
        # sync = TimeSynchronizer([self.m_isomorphism_list_sub, self.m_graph_sub, self.m_tag_sub], queue_size=10)
        # sync.registerCallback(self.callback)

        # Define publisher
        self.m_pre_h_pub = rospy.Publisher('/h_computation/input_matrices', PreHMsg, queue_size=10)

        # Initialize variables
        self.m_current_dict = {}
        self.m_desired_dict = {}
        self.m_h_current_coordinates = np.array([])
        self.m_h_desired_coordinates = np.array([])
    
    def buildReferenceIndexMatrix(self):
        # Read the reference index matrix from the reference tags dataset
        with open(f"{package_dir}/datasets/snapshots/reference_graph_dataset.csv", "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row
            for row in reader:
                self.m_reference_index_matrix = row[4]
        rospy.loginfo("Reference index matrix obtained.")

    def reshapeCurrentGraphData(self, msg):
        # Store the timestamp, adjacency matrix, and indexed matrix from the graph data
        self.current_indexed_matrix = msg.indexed_matrix
        self.current_tags_id = msg.tags_id 
        self.isomorphism_list =msg.isomorphism_list
        self.current_coordinates = np.reshape(msg.coordinate_matrix,(len(self.current_tags_id),3))
        rospy.loginfo("Current graph data reshaped.")
    
    def buildCurrentTagDict(self):
        # Store the current tag coordinates from the tag detections
        for i in self.current_tags_id:
            tag_id = self.current_tags_id[i]
            self.m_current_dict[tag_id] = self.current_coordinates[i]
        rospy.loginfo("Current tag dictionary built.")
    
    def buildDesiredTagDict(self):
        # Build dictionaries for desired tag coordinates
        with open(f"{package_dir}/datasets/snapshots/desired_frame_tags_dataset.csv", "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row
            for row in reader:
                tag_id = int(row[2])
                x = float(row[3])
                y = float(row[4])
                z = float(row[5])
                self.m_desired_dict[tag_id] = [x, y, z]
        rospy.loginfo("Desired tag dictionary built.")

    def calculateCoordinates(self):
        rospy.loginfo("Building the current and desired coordinates matrices...")
        # Calculate the current and desired coordinates for the isomorphism list
        for i in range(len(self.isomorphism_list) - 1):
            ref_id = self.m_reference_index_matrix[self.isomorphism_list[i]] # Get the reference tag ID
            if ref_id in self.m_desired_dict.keys(): # Check if the reference tag ID is in the desired dictionary
                self.m_h_desired_coordinates = np.append(self.m_h_desired_coordinates, self.m_desired_dict[ref_id]) # Append the desired coordinates
                cur_id = self.current_indexed_matrix[i] # Get the current tag ID
                self.m_h_current_coordinates = np.append(self.m_h_current_coordinates, self.m_current_dict[cur_id]) # Append the current coordinates
        check = len(self.m_h_current_coordinates) == len(self.m_h_desired_coordinates) # Check if the current and desired coordinates matrices match
        if check:
            rospy.loginfo("The current and desired coordinates matrices match.")
        else:
            rospy.loginfo("The current and desired coordinates matrices do not match.")
    
    def publishMatrix(self, current_coordinates, desired_coordinates):
        # Publish the current and desired coordinates as a PreHMsg message
        rospy.loginfo("Publishing the current and desired coordinates matrices...")
        msg = PreHMsg()
        msg.current_coordinates = list(current_coordinates.flatten())
        msg.desired_coordinates = list(desired_coordinates.flatten())
        self.m_pre_h_pub.publish(msg)
        rospy.loginfo("Coordinates matrices published.")

    def callback(self, msg):
        # Callback function for synchronized data
        self.buildReferenceIndexMatrix()
        self.reshapeCurrentGraphData(msg)
        self.buildCurrentTagDict()
        self.buildDesiredTagDict()
        self.calculateCoordinates()
        self.publishMatrix(self.m_h_current_coordinates, self.m_h_desired_coordinates)

if __name__ == "__main__":
    # Create an instance of PreHMatrix and start the ROS node
    pre_h_matrix = PreHMatrix()
    rospy.spin()

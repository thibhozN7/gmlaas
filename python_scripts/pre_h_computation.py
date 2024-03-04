#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from message_filters import Subscriber, TimeSynchronizer
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
        env_mode = "simu"
        if env_mode == "real":
            self.env = "real"
        elif env_mode == "simu":
            self.env = "simu"
    
    def buildReferenceIndexMatrix(self):
        # Read the reference index matrix from the reference tags dataset
        with open(f"{package_dir}/datasets/snapshots/{self.env}/reference_graph_dataset.csv", "r") as file:
            reader = csv.reader(file, delimiter=';')
            next(reader)  # Skip the header row
            next(reader)
            for row in reader:
                self.m_reference_index_matrix = [int(x) for x in row[4].replace("(","").replace(")","").split(",")]
                print(self.m_reference_index_matrix)
        rospy.loginfo("Reference index matrix obtained.")

    def reshapeCurrentGraphData(self, msg):
        # Store the timestamp, adjacency matrix, and indexed matrix from the graph data
        self.current_indexed_matrix = list(msg.indexed_matrix)
        self.current_tags_id = list(msg.tags_id)
        self.isomorphism_list = list(map(int,list(msg.isomorphism_list)))
        self.current_coordinates = np.reshape(msg.coordinate_matrix,(len(self.current_tags_id),3))
        rospy.loginfo("Current graph data reshaped.")
    
    def buildCurrentTagDict(self):
        # Store the current tag coordinates from the tag detections
        print(self.current_tags_id)
        for i in range(len(self.current_tags_id)-1):
            tag_id = self.current_tags_id[i]
            self.m_current_dict[tag_id] = self.current_coordinates[i]
        rospy.loginfo("Current tag dictionary built.")
    
    def buildDesiredTagDict(self):
        # Build dictionaries for desired tag coordinates
        with open(f"{package_dir}/datasets/snapshots/{self.env}/desired_frame_tags_dataset.csv", "r") as file:
            reader = csv.reader(file, delimiter=';')
            next(reader)  # Skip the header row
            next(reader)
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
        print(f"Isomorphism list: {self.isomorphism_list}")
        print(f"len of isomorphism list: {len(self.isomorphism_list)}")
        for i in range(len(self.isomorphism_list) - 1):

            ref_id = self.m_reference_index_matrix[self.isomorphism_list[i]] # Get the reference tag ID
            if ref_id in self.m_desired_dict.keys(): # Check if the reference tag ID is in the desired dictionary
                self.m_desired_matrix.append(self.m_desired_dict[ref_id])
                
                print(f"Desired ID: {ref_id} - Desired coordinates: {self.m_desired_dict[ref_id]}")
                cur_id = self.current_indexed_matrix[i] # Get the current tag ID
                self.m_current_matrix.append(self.m_current_dict[cur_id]) # Append the current coordinates
                print(f"Current ID: {cur_id} - Current coordinates: {self.m_current_dict[cur_id]}")
        
        check = (len(self.m_current_matrix) == len(self.m_desired_matrix)) # Check if the current and desired coordinates matrices match
        
        # print(f"Current coordinates: {self.m_h_current_coordinates}")
        # print("")
        # print(f"Desired coordinates: {self.m_h_desired_coordinates}")
        # print("")
        if check:
            rospy.loginfo("The current and desired coordinates matrices match.")
            print(f"Current coordinates: {self.m_current_matrix}")
            print(f"Size of current coordinates: {len(self.m_current_matrix)}")
        else:
            rospy.loginfo("The current and desired coordinates matrices do not match.")
    
    def publishMatrix(self, current_coordinates, desired_coordinates):
        # Publish the current and desired coordinates as a PreHMsg message
        rospy.loginfo("Publishing the current and desired coordinates matrices...")
        msg = PreHMsg()
        msg.current_coordinates = list(np.array(current_coordinates).flatten())
        msg.desired_coordinates = list(np.array(desired_coordinates).flatten())
        self.m_pre_h_pub.publish(msg)
        rospy.loginfo("Coordinates matrices published.")

    def callback(self, msg):
        self.m_current_dict = {}
        self.m_desired_dict = {}
        self.m_current_matrix = []
        self.m_desired_matrix = []
        # Callback function for synchronized data
        self.buildReferenceIndexMatrix()
        self.reshapeCurrentGraphData(msg)
        self.buildCurrentTagDict()
        self.buildDesiredTagDict()
        self.calculateCoordinates()
        self.publishMatrix(self.m_current_matrix, self.m_desired_matrix)
        
if __name__ == "__main__":
    # Create an instance of PreHMatrix and start the ROS node
    pre_h_matrix = PreHMatrix()
    
    rospy.spin()

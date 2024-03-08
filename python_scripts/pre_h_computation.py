#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from message_filters import Subscriber, TimeSynchronizer
from gmlaas.msg import PreHMsg
from gmlaas.msg import GraphMatcherMsg
from gmlaas.msg import PreHData

from apriltag_ros.msg import AprilTagDetectionArray

import os
import csv

# Get the current directory and package directory
current_dir = os.path.realpath(__file__)
package_dir = os.path.dirname(os.path.dirname(current_dir))

class PreHMatrix:
    def __init__(self):
        rospy.init_node("pre_h_computation_py", anonymous=False)
        # Define subscribers
        self.m_graph_sub = rospy.Subscriber("/graph_matching/data", GraphMatcherMsg,self.callback)

        # Define publisher
        self.m_pre_h_pub = rospy.Publisher('/h_computation/input_matrices', PreHMsg, queue_size=10)

        self.m_data_pub = rospy.Publisher('/data/pre_h_computation', PreHData, queue_size=10)

        # Initialize variables
        simu=rospy.get_param('/pre_h_computation_py/simu_value', True)
        if simu : 
            self.simu = "simu"
        else :
            self.simu = "real"
    
    def buildReferenceIndexMatrix(self):
        # Read the reference index matrix from the reference tags dataset
        with open(f"{package_dir}/datasets/snapshots/{self.simu}/reference_graph_dataset.csv", "r") as file:
            reader = csv.reader(file, delimiter=';')
            next(reader)  # Skip the header row
            next(reader)
            for row in reader:
                self.m_reference_index_matrix = [int(x) for x in row[4].replace("(","").replace(")","").split(",")]

    def reshapeCurrentGraphData(self, msg):
        # Store the timestamp, adjacency matrix, and indexed matrix from the graph data
        self.current_indexed_matrix = list(msg.indexed_matrix)
        self.current_tags_id = list(msg.tags_id)
        self.isomorphism_list = list(map(int,list(msg.isomorphism_list)))
        self.current_coordinates = np.reshape(msg.coordinate_matrix,(len(self.current_tags_id),3))
    
    def buildCurrentTagDict(self):
        # Store the current tag coordinates from the tag detections
        for i in range(len(self.current_tags_id)-1):
            tag_id = self.current_tags_id[i]
            self.m_current_dict[tag_id] = self.current_coordinates[i]
    
    def buildDesiredTagDict(self):
        # Build dictionaries for desired tag coordinates
        with open(f"{package_dir}/datasets/snapshots/{self.simu}/desired_frame_tags_dataset.csv", "r") as file:
            reader = csv.reader(file, delimiter=';')
            next(reader)  # Skip the header row
            next(reader)
            for row in reader:
                tag_id = int(row[2])
                x = float(row[3])
                y = float(row[4])
                z = float(row[5])
                self.m_desired_dict[tag_id] = [x, y, z]

    def calculateCoordinates(self):
        # Calculate the current and desired coordinates for the isomorphism list
        sc=0
        for i in range(len(self.isomorphism_list)-1):

            ref_id = self.m_reference_index_matrix[self.isomorphism_list[i]] # Get the reference tag ID
            if ref_id in self.m_desired_dict.keys(): # Check if the reference tag ID is in the desired dictionary
                self.m_desired_matrix.append(self.m_desired_dict[ref_id])
                cur_id = self.current_indexed_matrix[i] # Get the current tag ID
                self.m_current_matrix.append(self.m_current_dict[cur_id]) # Append the current coordinates
                if ref_id == cur_id:
                    sc+=1

        self.score = sc/len(self.isomorphism_list)*100
        
        check = (len(self.m_current_matrix) == len(self.m_desired_matrix)) # Check if the current and desired coordinates matrices match
        
        if check:
            pass
        else:
            rospy.loginfo("The current and desired coordinates matrices do not match.")
    
    def publishMatrix(self, current_coordinates, desired_coordinates):
        # Publish the current and desired coordinates as a PreHMsg message
        msg = PreHMsg()
        msg.current_coordinates = list(np.array(current_coordinates).flatten())
        msg.desired_coordinates = list(np.array(desired_coordinates).flatten())
        self.m_pre_h_pub.publish(msg)

    def publishData(self):
        msg_data=PreHData()
        msg_data.num_current_points= int(len(self.isomorphism_list))
        msg_data.num_matched_points=int(len(self.m_current_matrix))
        msg_data.score = int(self.score)
        self.m_data_pub.publish(msg_data)


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
        self.publishData()
        
if __name__ == "__main__":
    # Create an instance of PreHMatrix and start the ROS node
    pre_h_matrix = PreHMatrix()
    
    rospy.spin()

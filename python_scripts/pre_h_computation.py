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

    def storeCurrentGraphData(self):
        # Store the timestamp, adjacency matrix, and indexed matrix from the graph data
        self.adjacency_matrix = self.m_graph_sub.adjacency_matrix
        self.current_indexed_matrix = self.m_graph_sub.indexed_matrix
        self.isomorphism_list = self.m_graph_sub.indexed_matrix
        rospy.loginfo("Current graph data stored.")

    def buildReferenceIndexMatrix(self):
        # Read the reference index matrix from the reference tags dataset
        with open(f"{package_dir}/datasets/referenc/tags_dataset.csv", "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row
            for row in reader:
                self.m_reference_index_matrix = row[4]
        rospy.loginfo("Reference index matrix built.")
    
    def buildCurrentTagDict(self):
        # Store the current tag coordinates from the tag detections
        for detection in self.m_graph_sub.detections:
            tag_id = detection.id[0]
            x = detection.pose.pose.pose.position.x
            y = detection.pose.pose.pose.position.y
            z = detection.pose.pose.pose.position.z
            self.m_current_dict[tag_id] = [x, y, z]
        rospy.loginfo("Current tag dictionary built.")
    
    def buildDesiredTagDict(self):
        # Build dictionaries for desired tag coordinates
        with open(f"{package_dir}/datasets/desired/tags_dataset.csv", "r") as file:
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
        # Calculate the current and desired coordinates for the isomorphism list
        for i in range(len(self.isomorphism_list) - 1):
            ref_id = self.m_reference_index_matrix[self.isomorphism_list[i]]
            if ref_id in self.m_desired_dict.keys():
                self.m_h_desired_coordinates = np.append(self.m_h_desired_coordinates, self.m_desired_dict[ref_id])
                cur_id = self.current_indexed_matrix[i]
                self.m_h_current_coordinates = np.append(self.m_h_current_coordinates, self.m_current_dict[cur_id])
        check = len(self.m_h_current_coordinates) == len(self.m_h_desired_coordinates)
        if check:
            rospy.loginfo("The current and desired coordinates matrices match.")
        else:
            rospy.loginfo("The current and desired coordinates matrices do not match.")
    
    def publishMatrix(self, current_coordinates, desired_coordinates):
        # Publish the current and desired coordinates as a PreHMsg message
        msg = PreHMsg()
        msg.current_coordinates = list(current_coordinates.flatten())
        msg.desired_coordinates = list(desired_coordinates.flatten())
        self.m_pre_h_pub.publish(msg)
        rospy.loginfo("Coordinates matrices published.")

    def callback(self, m_isomorphism_list_sub, m_graph_sub, m_tag_sub):
        # Callback function for synchronized data
        self.storeCurrentGraphData(m_graph_sub)
        self.storeCurrentTagData(m_tag_sub)
        self.buildReferenceIndexMatrix()
        self.buildDesiredDict()
        self.calculateCoordinates()
        self.publishMatrix(self.m_h_current_coordinates, self.m_h_desired_coordinates)

if __name__ == "__main__":
    # Create an instance of PreHMatrix and start the ROS node
    pre_h_matrix = PreHMatrix()
    rospy.spin()

#!/usr/bin/env python3

import rospy
from apriltag_listener.msg import CustomMsg
from apriltag_ros.msg import AprilTagDetectionArray
from message_filters import Subscriber, TimeSynchronizer

def callback(graph_data, tag_detections):
    timestamp = graph_data.header.stamp
    adjacency_matrix = graph_data.adjacency_matrix
    indexed_matrix = graph_data.indexed_matrix

    # Process the received data as needed
    print(f"Received synchronized data at timestamp {timestamp}")
    print("Adjacency Matrix:", adjacency_matrix)
    print("Indexed Matrix:", indexed_matrix)

    # Process tag detections as needed
    for detection in tag_detections.detections:
        tag_id = detection.id[0]
        x = detection.pose.pose.pose.position.x
        y = detection.pose.pose.pose.position.y
        z = detection.pose.pose.pose.position.z
        print(f"Tag {tag_id} detected at coordinates ({x}, {y}, {z}) at timestamp {timestamp}")

        

if __name__ == "__main__":
    rospy.init_node("information_listener", anonymous=True)

    # Use message_filters to synchronize messages from both topics based on timestamps
    graph_sub = Subscriber("/apriltag_to_graph_py/graph_builder_data", CustomMsg)
    tag_sub = Subscriber("/tag_detections", AprilTagDetectionArray)
    sync = TimeSynchronizer([graph_sub, tag_sub], queue_size=10)
    sync.registerCallback(callback)
    rospy.spin()

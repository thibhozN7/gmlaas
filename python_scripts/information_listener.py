#!/usr/bin/env python3

import rospy
from gmlaas.msg import GraphBuilderMsg
from apriltag_ros.msg import AprilTagDetectionArray
from message_filters import Subscriber, TimeSynchronizer
import datetime as dt
import os
#import time

current_dir = os.path.realpath(__file__)
package_dir = os.path.dirname(os.path.dirname(current_dir))

file1 = open(f"{package_dir}/datasets/graph_dataset.csv", "w")
file2 = open(f"{package_dir}/datasets/tags_dataset.csv", "w")

datetime = dt.datetime.now()
formatted_datetime = datetime.strftime("%Y-%m-%d %H:%M")

#Headers
file1.write(f"<TITLE : Graph Dataset (YYYY-MM-DD HH:MM): {formatted_datetime} >\n")
file2.write(f"<TITLE : Tags Dataset (YYYY-MM-DD HH:MM): {formatted_datetime} >\n")
file1.write("timestamp sec, timpestamp nsecs, number_of_tags, adjacency_matrix, indexed_matrix\n") 
file2.write("timestamp, timpestamp nsecs, tag_id, x, y, z\n")

# Control frequency settings
#desired_rate = 1  # Desired acquisition frequency in Hz

    

def callback(graph_data, tag_detections):
    timestamp_secs = graph_data.header.stamp.secs #int
    timestamp_nsecs = graph_data.header.stamp.nsecs #int

    adjacency_matrix = graph_data.adjacency_matrix
    indexed_matrix = graph_data.indexed_matrix

    # Write data to file1
    file1.write(f"{timestamp_secs}, {timestamp_nsecs}, {len(tag_detections.detections)}, {adjacency_matrix}, {indexed_matrix}\n")

    # Process the received data as needed
    print(f"Received synchronized data at timestamp : {timestamp_secs}.{timestamp_nsecs}")
    print(timestamp_nsecs)
    print("Adjacency Matrix:", adjacency_matrix)
    print("Indexed Matrix:", indexed_matrix)   

    # Process tag detections as needed
    for detection in tag_detections.detections:
        tag_id = detection.id[0]
        x = detection.pose.pose.pose.position.x
        y = detection.pose.pose.pose.position.y
        z = detection.pose.pose.pose.position.z
        print(f"Tag {tag_id} detected at coordinates ({x}, {y}, {z}) at timestamp : {timestamp_secs}.{timestamp_nsecs}")

        # Write data to file2
        file2.write(f"{timestamp_secs}, {timestamp_nsecs}, {tag_id}, {x}, {y}, {z}\n")
    

def register_callback(sync, callback):
    try:
        sync.registerCallback(callback)

    except RuntimeError as e:
        print(f"Error occurred during callback registration: {e}")


if __name__ == "__main__":

    rospy.init_node("information_listener", anonymous=False)

    # Use message_filters to synchronize messages from both topics based on timestamps
    graph_sub = Subscriber("graph_building/data", GraphBuilderMsg)
    tag_sub = Subscriber("/tag_detections", AprilTagDetectionArray)
    sync = TimeSynchronizer([graph_sub, tag_sub], queue_size=1)

    register_callback(sync, callback)
    
    rospy.spin()

    


            

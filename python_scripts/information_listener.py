#!/usr/bin/env python3

import rospy
from gmlaas.msg import CustomMsg
from apriltag_ros.msg import AprilTagDetectionArray
from message_filters import Subscriber, TimeSynchronizer
import datetime as dt

file1 = open("datasets/graph/graph_dataset.csv", "w")
file2 = open("datasets/tags/tags_dataset.csv", "w")

datetime = dt.datetime.now()
formatted_datetime = datetime.strftime("%Y-%m-%d %H:%M")

#Headers
file1.write("<TITLE : Graph Dataset (YYYY-MM-DD HH:MM): {formatted_datetime} >\n")
file2.write("<TITLE : Tags Dataset (YYYY-MM-DD HH:MM): {formatted_datetime} >\n")
file1.write("timestamp, date_time, number_of_tags, adjacency_matrix, indexed_matrix\n") 
file2.write("timestamp, date_time, tag_id, x, y, z\n")



def callback(graph_data, tag_detections):
    timestamp = graph_data.header.stamp
    adjacency_matrix = graph_data.adjacency_matrix
    indexed_matrix = graph_data.indexed_matrix

    current_date_time = dt.datetime.fromtimestamp(timestamp)
    current_date_time = current_date_time.strftime('%H:%M:%S:%f')[:-3]

    # Write data to file1
    file1.write(f"{timestamp}, {current_date_time}, {len(tag_detections.detections)}, {adjacency_matrix}, {indexed_matrix}\n")

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

        # Write data to file2
        file2.write(f"{timestamp}, {current_date_time}, {tag_id}, {x}, {y}, {z}\n")
        

if __name__ == "__main__":

    rospy.init_node("information_listener", anonymous=False)
    
    # Use message_filters to synchronize messages from both topics based on timestamps
    graph_sub = Subscriber("/apriltag_to_graph_py/graph_builder_data", CustomMsg)
    tag_sub = Subscriber("/tag_detections", AprilTagDetectionArray)
    sync = TimeSynchronizer([graph_sub, tag_sub], queue_size=10)
    sync.registerCallback(callback)
    rospy.spin()
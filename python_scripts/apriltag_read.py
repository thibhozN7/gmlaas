#!/usr/bin/env python3

import rospy
import python_scripts.graph_builder as graph_builder
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float32MultiArray


connections = [[5, 8],
               [8, 2],
               [8, 9],
               [9, 7],
               [8, 6],
               [6, 1],
               [1, 3],
               [1, 4]]

def apriltag_callback(data):
    # Set to store existing node IDs
    existing_node_ids = []
    tree = graph_builder.Graph() 
    # Set to store existing edges
    existing_edges = []  

    for detection in data.detections:
        id = detection.id[0]
        x = detection.pose.pose.pose.position.x
        y = detection.pose.pose.pose.position.y
        z = detection.pose.pose.pose.position.z

        # Check if node already exists for the ID
        if id not in existing_node_ids:
            node = graph_builder.Node(id, x, y, z)
            tree.add_node(node)
            existing_node_ids.append(id)
        else:
            # Update node coordinates
            for node in tree.nodes:
                if node.id == id:
                    node.x = x
                    node.y = y
                    node.z = z
                    break
    


    # Add edges based on connections list
    for conn in connections:
        from_node_id, to_node_id = conn

        # Check if both nodes exist and edge does not already exist
        if (from_node_id, to_node_id) not in existing_edges and (to_node_id, from_node_id) not in existing_edges:
            if from_node_id in existing_node_ids and to_node_id in existing_node_ids:
                edge = graph_builder.Edge(from_node_id, to_node_id, "connection")
                tree.add_edge(edge)
                edge.calculate_edge_length(tree)
                existing_edges.append((from_node_id, to_node_id))
                

    #Print node mapping and adjacency matrix after processing data
    #tree.print_node_mapping(tree.create_node_mapping())
    #tree.print_adjacency_matrix(tree.calculate_adjacency_matrix())
    adjacency_matrix = tree.calculate_adjacency_matrix()
    adjacency_matrix_msg = Float32MultiArray(data=sum(adjacency_matrix, []))
    adjacency_matrix_publisher.publish(adjacency_matrix_msg)
  

def listener():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback)
    rospy.spin()

if __name__ == '__main__':
    adjacency_matrix_publisher = rospy.Publisher("/adjacency_matrix", Float32MultiArray, queue_size=10)
    listener()

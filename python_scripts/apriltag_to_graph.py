#!/usr/bin/env python3

import rospy
import graph_builder
from collections import OrderedDict
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Header
from gmlaas.msg import GraphBuilderMsg

class AprilTagToGraph:

    def __init__(self,nameNode,connections):
        # Store the connections
        self.m_connections=connections 
        # Initialize a ROS header for msg
        self.m_header = Header() 
        # Initialize the ROS node
        rospy.init_node(nameNode, anonymous=False) 
        # Subscribe to the AprilTag detections topic
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_listener_callback)
        # Initialize a publisher for the graph data
        self.m_publisher = rospy.Publisher("/graph_building/data", GraphBuilderMsg, queue_size=10)
 
    # callback to receive the AprilTag detections
    def apriltag_listener_callback(self,data):
        # Initialize sets and lists to store data
        self.m_existing_node_ids = [] 
        self.m_tree = graph_builder.Graph() # Initialize the graph
        self.m_existing_edges = []  
        self.m_coordinate_matrix=[] 
        self.m_tag_id_list=[] 

        # Loop over the detections
        for detection in data.detections:
            # Extract the ID and coordinates from the detection
            id = detection.id[0]
            x = detection.pose.pose.pose.position.x
            y = detection.pose.pose.pose.position.y
            z = detection.pose.pose.pose.position.z
            # Store the data 
            self.m_header = detection.pose.header
            self.m_tag_id_list.append(id) 
            self.m_coordinate_matrix.append([x,y,z])
            self.add_apriltag_node_to_graph(id,x,y,z)
        
        # Add the edges to the graph
        self.add_apriltag_edge_to_graph() 
    
    def add_apriltag_node_to_graph(self,id,x,y,z):
        # Check if the node already exists
        if id not in self.m_existing_node_ids:
            # Add the node to the graph
            node = graph_builder.Node(id, x, y, z)
            self.m_tree.add_node(node)
            self.m_existing_node_ids.append(id)
        else:
            # Update the node coordinates
            for node in self.m_tree.nodes:
                if node.id == id:
                    node.x = x
                    node.y = y
                    node.z = z
                    break
    
    def add_apriltag_edge_to_graph(self):
        for conn in self.m_connections:
            from_node_id, to_node_id = conn

            # Check if both nodes exist and edge does not already exist
            if (from_node_id, to_node_id) not in self.m_existing_edges and (to_node_id, from_node_id) not in self.m_existing_edges:
                if from_node_id in self.m_existing_node_ids and to_node_id in self.m_existing_node_ids:
                    edge = graph_builder.Edge(from_node_id, to_node_id, "connection")
                    self.m_tree.add_edge(edge)
                    edge.calculate_edge_length(self.m_tree)
                    self.m_existing_edges.append((from_node_id, to_node_id))
        # Publish the graph data
        self.publisher_fnc()

    def publisher_fnc(self):
        # Create a CustomMessage
        graph_builder_msg = GraphBuilderMsg()
        # Add header data
        graph_builder_msg.header = self.m_header
        # Add adjacency matrix data
        graph_builder_msg.adjacency_matrix = [ float(value) for row in self.m_tree.calculate_adjacency_matrix() for value in row ]
        # Add indexed matrix data
        graph_builder_msg.indexed_matrix = list(OrderedDict(self.m_tree.create_node_mapping()).keys())
        # Add tags id data
        graph_builder_msg.tags_id = self.m_tag_id_list
        # Add coordinate matrix data
        graph_builder_msg.coordinate_matrix = [ float(value) for row in self.m_coordinate_matrix for value in row ]
        # Publish the GraphBuilderMsg message
        self.m_publisher.publish(graph_builder_msg)

if __name__=="__main__":
    # Define the connections between the nodes, [parent, child]
    connections = [[0, 1],
                   [0, 2],
                   [1, 3],
                   [1, 4],
                   [2, 5],
                   [2, 6],
                   [3, 7],
                   [3, 8],
                   [4, 9],
                   [4, 10],
                   [5, 11],
                   [5, 12],
                   [6, 13],
                   [6, 14],
                   [7, 15],
                   [7, 16],
                   [8, 17],
                   [8, 18],
                   [9, 19],
                   [9, 20],
                   [10, 21],
                   [10, 22],
                   [11, 23],
                   [11, 24],
                   [12, 25],
                   [12, 26],
                   [13, 27],
                   [13, 28],
                   [14, 29],
                   [14, 30]]
                   
    apriltag_to_graph = AprilTagToGraph("apriltag_to_graph_py",connections)
    rospy.spin()    

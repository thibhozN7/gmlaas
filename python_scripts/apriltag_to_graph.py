#!/usr/bin/env python3

import rospy
import graph_builder
from collections import OrderedDict
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray, Header
from gmlaas.msg import GraphBuilderMsg

class AprilTagToGraph:

    def __init__(self,nameNode,connections):
        self.connections=connections
        # Set to store existing node IDs
        self.existing_node_ids = []
        self.tree = graph_builder.Graph() 
        # Set to store existing edges
        self.existing_edges = []  
        self.header = Header()
        rospy.init_node(nameNode, anonymous=False)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_listener_callback)
        self.publisher = rospy.Publisher("/graph_building/data", GraphBuilderMsg, queue_size=10)


    def apriltag_listener_callback(self,data):
        #Set to store existing node IDs
        self.existing_node_ids = []
        self.tree = graph_builder.Graph() 
        # Set to store existing edges
        self.existing_edges = []  
        self.coordinate_matrix=[]
        self.tag_id_list=[]
        for detection in data.detections:
            id = detection.id[0]
            x = detection.pose.pose.pose.position.x
            y = detection.pose.pose.pose.position.y
            z = detection.pose.pose.pose.position.z
            self.header = detection.pose.header
            self.tag_id_list.append(id)
            self.coordinate_matrix.append([x,y,z])
            self.add_apriltag_node_to_graph(id,x,y,z)
        self.add_apriltag_edge_to_graph()
    
    def add_apriltag_node_to_graph(self,id,x,y,z):
        if id not in self.existing_node_ids:
            node = graph_builder.Node(id, x, y, z)
            self.tree.add_node(node)
            self.existing_node_ids.append(id)
        else:
            # Update node coordinates
            for node in self.tree.nodes:
                if node.id == id:
                    node.x = x
                    node.y = y
                    node.z = z
                    break
    
    def add_apriltag_edge_to_graph(self):
        for conn in self.connections:
            from_node_id, to_node_id = conn

            # Check if both nodes exist and edge does not already exist
            if (from_node_id, to_node_id) not in self.existing_edges and (to_node_id, from_node_id) not in self.existing_edges:
                if from_node_id in self.existing_node_ids and to_node_id in self.existing_node_ids:
                    edge = graph_builder.Edge(from_node_id, to_node_id, "connection")
                    self.tree.add_edge(edge)
                    edge.calculate_edge_length(self.tree)
                    self.existing_edges.append((from_node_id, to_node_id))
        self.publisher_fnc()

    def publisher_fnc(self):
        # Create a CustomMessage
        graph_builder_msg = GraphBuilderMsg()
        graph_builder_msg.header = self.header
        # Add adjacency matrix data
        graph_builder_msg.adjacency_matrix = [ float(value) for row in self.tree.calculate_adjacency_matrix() for value in row ]
        # Add indexed matrix data
        graph_builder_msg.indexed_matrix = list(OrderedDict(self.tree.create_node_mapping()).keys())
        graph_builder_msg.tags_id = self.tag_id_list
        graph_builder_msg.coordinate_matrix = [ float(value) for row in self.coordinate_matrix for value in row ]
        # Publish the CustomMessage on the combined topic
        self.publisher.publish(graph_builder_msg)

if __name__=="__main__":
    connections = [[0,1],
                   [1,2],[1,28],
                   [2,3],[2,18],
                   [3,5],[3,13],
                   [4,7],[4,6],
                   [5,8],
                   [6,26],
                   [8,36],[8,9],[8,4],
                   [9,10],[9,11],
                   [11,12],[11,14],
                   [14,15],
                   [13,16],
                   [16,17],
                   [18,19],
                   [19,25],
                   [28,29],
                   [28,30],
                   [29,31],
                   [30,34],
                   [30,32]]
    apriltag_to_graph = AprilTagToGraph("apriltag_to_graph_py",connections)
    rospy.spin()    

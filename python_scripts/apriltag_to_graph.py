#!/usr/bin/env python3

import rospy
import Graph
from collections import OrderedDict
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Header
from apriltag_listener.msg import CustomMsg

class AprilTagToGraph:

    def __init__(self, nameNode, connections):
        self.connections = connections
        self.existing_node_ids = []
        self.tree = Graph.Graph() 
        self.existing_edges = []  
        self.header = Header()
        rospy.init_node(nameNode, anonymous=True)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_listener_callback)
        self.publisher = rospy.Publisher(nameNode + "/graph_builder_data", CustomMsg, queue_size=10)

    def apriltag_listener_callback(self, data):
        self.existing_node_ids = []
        self.tree = Graph.Graph() 
        self.existing_edges = []  
        for detection in data.detections:
            id = detection.id[0]
            x = detection.pose.pose.pose.position.x
            y = detection.pose.pose.pose.position.y
            z = detection.pose.pose.pose.position.z
            self.header = detection.pose.header
            self.add_apriltag_node_to_graph(id, x, y, z)
        self.add_apriltag_edge_to_graph()

    def add_apriltag_node_to_graph(self, id, x, y, z):
        if id not in self.existing_node_ids:
            node = Graph.Node(id, x, y, z)
            self.tree.add_node(node)
            self.existing_node_ids.append(id)
        else:
            for node in self.tree.nodes:
                if node.id == id:
                    node.x = x
                    node.y = y
                    node.z = z
                    break

    def add_apriltag_edge_to_graph(self):
        for conn in self.connections:
            from_node_id, to_node_id = conn
            if (from_node_id, to_node_id) not in self.existing_edges and (to_node_id, from_node_id) not in self.existing_edges:
                if from_node_id in self.existing_node_ids and to_node_id in self.existing_node_ids:
                    edge = Graph.Edge(from_node_id, to_node_id, "connection")
                    self.tree.add_edge(edge)
                    edge.calculate_edge_length(self.tree)
                    self.existing_edges.append((from_node_id, to_node_id))
        self.publisher_fnc()

    def publisher_fnc(self):
        # Create a CustomMessage
        custom_msg = CustomMsg()
        custom_msg.header = self.header
        # Add adjacency matrix data
        custom_msg.adjacency_matrix = [ float(value) for row in self.tree.calculate_adjacency_matrix() for value in row ]
        # Add indexed matrix data
        custom_msg.indexed_matrix = list(OrderedDict(self.tree.create_node_mapping()).keys())

        # Publish the CustomMessage on the combined topic
        self.publisher.publish(custom_msg)

if __name__ == "__main__":
    connections = [[5, 8],
               [8, 2],
               [8, 9],
               [9, 7],
               [8, 6],
               [6, 1],
               [1, 3],
               [1, 4]]
    apriltag_to_graph = AprilTagToGraph("apriltag_to_graph_py", connections)
    rospy.spin()


#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from gmlaas.msg import GraphBuilderMsg, GraphMatcherMsg
import numpy as np
import pygmtools as pygm
import functools
import csv
import os


class GraphMatcher:
    def __init__(self):
        rospy.init_node("graph_matcher_py", anonymous=False)        
        self.m_graph_builder_data_sub = rospy.Subscriber("/graph_building/data",GraphBuilderMsg, self.callback,queue_size=10)
        self.m_graph_matcher_data_pub = rospy.Publisher("/graph_matching/data",GraphMatcherMsg, queue_size=10)

        self.m_header = Header()
        self.m_reference_adjacency_matrix = self.initReferenceAdjMatrix() 
    
    def buildMatrix(self, data, rows, cols):
        """
        Builds a matrix from given data, rows, and columns.

        Args:
            data (list): List of matrix values.
            rows (int): Number of rows in the matrix.
            cols (int): Number of columns in the matrix.

        Returns:
            numpy.ndarray: The built matrix.
        """
        matrix = np.array(data).reshape(rows, cols)
        return matrix

    def initReferenceAdjMatrix(self):
        """
        Initializes the reference adjacency matrix.
f
        Returns:
            numpy.ndarray: The initialized reference adjacency matrix.
        """
        package_path = os.path.dirname(os.path.dirname(__file__))  # Get the directory of the ROS package (two levels up from the script file)
        csv_name = "reference_graph_dataset.csv"
        file_path = os.path.join(package_path, "datasets/snapshots", csv_name )
        
        with open(file_path, 'r',) as file:
            reader = csv.reader(file, delimiter=';')
            next(reader),next(reader)  # Skip title and header row
            for row in reader:
                reference_adjacency_matrix = [float(x) for x in row[3].replace("(","").replace(")","").split(", ")]
                size=int(row[2])
        shaped_matrix = self.buildMatrix(reference_adjacency_matrix, size, size)
        rospy.loginfo("Reference Adjacency Matrix initialized")  
        return shaped_matrix
    
    def listener(self, msg):
        """
        Callback function for receiving graph building data.

        Args:
            msg (GraphBuilderMsg): The received graph building data.
        """
        self.m_header = msg.header
        self.m_indexed_matrix = msg.indexed_matrix
        adjacency_matrix = msg.adjacency_matrix
        self.m_current_adjacency_matrix=self.buildMatrix(adjacency_matrix, len(self.m_indexed_matrix), len(self.m_indexed_matrix))
        self.m_tags_id = msg.tags_id
        self.m_coordinate_matrix = msg.coordinate_matrix

    def solvePygmMatching(self):
        """
        Compute the optimal matching between two graphs.
        
        Returns:
            tuple: A tuple containing the affinity matrix (K) and the matching matrix (X).
        """
        pygm.set_backend('numpy')
        np.random.seed(1)

        rospy.loginfo(f"Current Adjacency Matrix: {self.m_current_adjacency_matrix.shape[0]}x{self.m_current_adjacency_matrix.shape[1]}")
        rospy.loginfo(f"Reference Adjacency Matrix: {self.m_reference_adjacency_matrix.shape[0]}x{self.m_reference_adjacency_matrix.shape[1]}")

        n1 = np.array(self.m_current_adjacency_matrix.shape)
        n2 = np.array(self.m_reference_adjacency_matrix.shape)
        conn1, edge1 = pygm.utils.dense_to_sparse(self.m_current_adjacency_matrix)
        conn2, edge2 = pygm.utils.dense_to_sparse(self.m_reference_adjacency_matrix)

        gaussian_aff = functools.partial(pygm.utils.gaussian_aff_fn, sigma=.1)
        K = pygm.utils.build_aff_mat(None, edge1, conn1, None, edge2, conn2, None, None, None, None, edge_aff_fn=gaussian_aff)
        X = pygm.rrwm(K, n1, n2)
        return K, X
    
    def computeRelationships(self, X):
        """
        Compute the relationships between the nodes of the current and reference graphs.
        
        Args:
            X (numpy.ndarray): Matching matrix => X[i, j] is the probability that node i in the current graph is matched to node j in the reference graph.
        
        Returns:
            list: The relationships between the nodes.
        """
        relationships_data = []
        max_idx = np.argmax(X, axis=1)
        for i in range(X.shape[0]):
                    relationships_data.append(max_idx[i])
        return relationships_data


    def buildIsomorphismList(self, isomorphism_data):
        """
        Build a list of isomorphism data.

        Args:
            isomorphism_data (list): The isomorphism data.

        Returns:
            list: The built isomorphism list.
        """
        isomorphism_list = list(isomorphism_data)
        return isomorphism_list
    
    def publisher(self):
        """
        Publishes the graph matching data.
        """
        custom_msg = GraphMatcherMsg()
        custom_msg.header = self.m_header
        custom_msg.adjacency_matrix = [ float(value) for row in self.m_current_adjacency_matrix for value in row ]
        custom_msg.indexed_matrix = self.m_indexed_matrix
        custom_msg.isomorphism_list = self.isomorphism_list
        custom_msg.tags_id = self.m_tags_id
        custom_msg.coordinate_matrix = self.m_coordinate_matrix

        self.m_graph_matcher_data_pub.publish(custom_msg)
        rospy.loginfo("Graph Matching Data Published")
    
    
    def callback(self, msg):
        """
        Callback function for receiving graph building data.

        Args:
            msg (GraphBuilderMsg): The received graph building data.
        """
        self.listener(msg)
        K, X = self.solvePygmMatching()        
        isomorphism_data = self.computeRelationships(X)
        self.isomorphism_list = self.buildIsomorphismList(isomorphism_data)
        self.publisher()

if __name__ == "__main__":
    matcher = GraphMatcher()
    rospy.spin()

        

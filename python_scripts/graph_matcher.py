#!/usr/bin/env python3
import rospy
import numpy as np
import pygmtools as pygm
import functools
from std_msgs.msg import Int32MultiArray,Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Header
import csv
import os
from collections import OrderedDict
from gmlaas.msg import GraphBuilderMsg, GraphMatcherMsg
#gitfuck
#fake = rospy.get_param('~fake_arg')

class GraphMatcher:
    def __init__(self):

        rospy.init_node("graph_matcher_py", anonymous=False)        
        rospy.loginfo("Starting graph matcher...")
        #self.fake = fake

        # if self.fake:
        #     topic = "/graph_building/fake/adjacency_matrix"
        #     rospy.loginfo("...Using fake adjacency matrix topic.")
        # else:
        #     topic = "/graph_building/data"
        #     rospy.loginfo("...Using real data topic.")
        
        topic = "/graph_building/data"
        rospy.loginfo("...Using real data topic.")
        self.m_graph_builder_data_sub = rospy.Subscriber(topic,GraphBuilderMsg, self.callback,queue_size=10)
        self.header = Header()
        self.m_graph_matcher_data_pub = rospy.Publisher("/graph_matching/data",GraphMatcherMsg, queue_size=10)

        self.reference_adjacency_matrix = self.initReferenceAdjMatrix()
        rospy.loginfo("Graph matcher initialized.")   
    
    def buildMatrix(self, data, rows, cols):
        matrix = np.array(data).reshape(rows, cols)
        return matrix

    def initReferenceAdjMatrix(self):

        package_path = os.path.dirname(os.path.dirname(__file__))  # Get the directory of the ROS package (two levels up from the script file)
        csv_name = "reference_graph_dataset.csv"
        file_path = os.path.join(package_path, "datasets/snapshots", csv_name )
        
        # Open the file in read mode
        with open(file_path, 'r',) as file:
            reader = csv.reader(file, delimiter=';')
            next(reader),next(reader)  # Skip the title and header row
            for row in reader:
                reference_adjacency_matrix = [float(x) for x in row[3].replace("(","").replace(")","").split(", ")]
                print(reference_adjacency_matrix)
                print(type(reference_adjacency_matrix))
                size=int(row[2])
        rospy.loginfo("Reference index matrix obtained.")
        
        self.reference_adjacency_matrix = self.buildMatrix(reference_adjacency_matrix, size, size)
        rospy.loginfo("Reference adjacency matrix:")
        rospy.loginfo(self.reference_adjacency_matrix)
        return self.reference_adjacency_matrix
    
    def listener(self, msg):
        self.header = msg.header
        adjacency_matrix = msg.adjacency_matrix
        self.indexed_matrix = msg.indexed_matrix
        self.current_adjacency_matrix=self.buildMatrix(adjacency_matrix, len(self.indexed_matrix), len(self.indexed_matrix))
        self.tags_id = msg.tags_id
        self.coordinate_matrix = msg.coordinate_matrix

    def publisher(self):
        # Create a CustomMessage
        custom_msg = GraphMatcherMsg()
        custom_msg.header = self.header
        # Add adjacency matrix data
        custom_msg.adjacency_matrix = [ float(value) for row in self.current_adjacency_matrix for value in row ]
        # Add indexed matrix data
        custom_msg.indexed_matrix = self.indexed_matrix
        # Add isomorphism list data
        custom_msg.isomorphism_list = self.isomorphism_list
        # Add tags id data
        custom_msg.tags_id = self.tags_id
        # Add coordinate matrix data
        custom_msg.coordinate_matrix = self.coordinate_matrix

        # Publish the CustomMessage on the combined topic
        self.m_graph_matcher_data_pub.publish(custom_msg)
    

    def solvePygmMatching(self):
        """
        Compute the optimal matching between two graphs.
        
        Args:
            current_adj_matrix (numpy.ndarray): Adjacency matrix of the current graph.
            reference_adjacency_matrix (numpy.ndarray): Adjacency matrix of reference graph.
        Returns:
            K (numpy.ndarray): Affinity matrix.
            X (numpy.ndarray): Matching matrix => X[i, j] is the probability that node i in the current graph is matched to node j in the reference graph.
        """
        pygm.set_backend('numpy')
        np.random.seed(1)

        print(self.current_adjacency_matrix.shape)
        print(self.reference_adjacency_matrix.shape)

        n1 = np.array(self.current_adjacency_matrix.shape)
        n2 = np.array(self.reference_adjacency_matrix.shape)
        conn1, edge1 = pygm.utils.dense_to_sparse(self.current_adjacency_matrix)
        conn2, edge2 = pygm.utils.dense_to_sparse(self.reference_adjacency_matrix)

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
            relationships_data (list): List of relationships between the nodes of the current and reference graphs.
        """
        relationships_data = []
        max_idx = np.argmax(X, axis=1)
        for i in range(X.shape[0]):
                    relationships_data.append(max_idx[i])
        return relationships_data


    def buildIsomorphismList(self, isomorphism_data):
        isomorphism_list = list(isomorphism_data)
        return isomorphism_list
    
    def callback(self, msg):
        self.listener(msg)
        rospy.loginfo(f"Adjacency matrix {self.current_adjacency_matrix.shape[0]}x{self.current_adjacency_matrix.shape[1]} received...")

        rospy.loginfo("Solving graph matching...")
        K, X = self.solvePygmMatching()
        rospy.loginfo(f"...done \nMatching matrix: \n{X}")
        
        rospy.loginfo("Computing relationships...")
        isomorphism_data = self.computeRelationships(X)
        self.isomorphism_list = self.buildIsomorphismList(isomorphism_data)
        rospy.loginfo(f"...done")

        rospy.loginfo("Publishing relationships...")
        self.publisher()
        rospy.loginfo(f"...Published:\n {self.isomorphism_list}")

if __name__ == "__main__":
    matcher = GraphMatcher()
    rospy.spin()

        

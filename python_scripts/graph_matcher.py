#!/usr/bin/env python3
import rospy
import numpy as np
import pygmtools as pygm
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
import networkx as nx
import functools
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class GraphMatcher:
    def __init__(self):
        rospy.init_node('graph_matcher_py', anonymous=True)
        self.m_adj_matrix_sub = rospy.Subscriber('/graph_building/adjacency_matrix',Float32MultiArray, self.callback)
        self.m_isomorphism_pub = rospy.Publisher('/graph_matching/isomorphism_list',Float32MultiArray, queue_size=10)

    def buildMatrix(self, list, rows, cols):
        return (np.array([list[i * cols:(i + 1) * cols] for i in range(rows)]))

    
    def solvePygmMatching(self, input_adj_matrix, reference_adj_matrix):
        """
        Compute the optimal matching between two graphs.
        
        Args:
            input_adj_matrix (numpy.ndarray): Adjacency matrix of the input graph.
            reference_adj_matrix (numpy.ndarray): Adjacency matrix of reference graph.
        Returns:
            K (numpy.ndarray): Affinity matrix.
            X (numpy.ndarray): Matching matrix => X[i, j] is the probability that node i in the input graph is matched to node j in the reference graph.
        """
        pygm.set_backend('numpy')
        np.random.seed(1)
        n1 = np.array(input_adj_matrix.shape[0])
        n2 = np.array(reference_adj_matrix.shape[0])
        conn1, edge1 = pygm.utils.dense_to_sparse(input_adj_matrix)
        conn2, edge2 = pygm.utils.dense_to_sparse(reference_adj_matrix)

        gaussian_aff = functools.partial(pygm.utils.gaussian_aff_fn, sigma=.1)
        K = pygm.utils.build_aff_mat(None, edge1, conn1, None, edge2, conn2, None, None, None, None, edge_aff_fn=gaussian_aff)
        X = pygm.rrwm(K, n1, n2)
        return K, X

    def buildMessage(self, relationships_data):
        relationships_msg = Float32MultiArray()
        relationships_msg.data = list(relationships_data)
        return relationships_msg
    
    def callback(self, data):
        rows = data.layout.dim[0].size
        cols = data.layout.dim[1].size
        rospy.loginfo(f"Adjacency matrix {rows}x{cols} received.")

        adj_matrix_data = list(data.data)
        adj_matrix = self.buildMatrix(adj_matrix_data, rows, cols)

        rospy.loginfo("Solving graph matching...")
        K, X = self.solvePygmMatching(adj_matrix, self.reference_adj_matrix)
        rospy.loginfo(f"Matching matrix: {X}")
        rospy.loginfo("Computing relationships...")
        relationships_data = self.computeRelationships(X)
        relationships_msg = self.buildMessage(relationships_data)

        rospy.loginfo("Publishing relationships...")
        self.m_isomorphism_pub.publish(relationships_msg)
        rospy.loginfo(f"Published: {relationships_msg}")

if __name__ == "__main__":
    matcher = GraphMatcher()
    rospy.spin()

        

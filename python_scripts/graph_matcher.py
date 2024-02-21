#!/usr/bin/env python3
import rospy
import numpy as np
import pygmtools as pygm
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
import networkx as nx
import functools
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class GraphMatcher:
    def __init__(self):
        rospy.init_node('graph_matcher_py', anonymous=True)
        self.m_adj_matrix_sub = rospy.Subscriber('/graph_adj_matrix',Float32MultiArray, self.callback)
        self.m_isomorphism_pub = rospy.Publisher('/graph_isomorphism',Float32MultiArray, queue_size=10)

    def buildMatrix(list, rows, cols):
        return (np.array([list[i * cols:(i + 1) * cols] for i in range(rows)]))

    
    def solvePygmMatching(self, adj_matrix, reference_adj_matrix):
        """
        Compute the optimal matching between two graphs.
        
        Args:
            A1 (numpy.ndarray): Adjacency matrix of the first graph.
            A2 (numpy.ndarray): Adjacency matrix of the second graph.
            num_nodes (int): Number of nodes in the graphs.
        
        Returns:
            K (numpy.ndarray): Affinity matrix.
            X (numpy.ndarray): Matching matrix.
        """
        pygm.set_backend('numpy')
        np.random.seed(1)
        n1 = np.array(adj_matrix.shape[0])
        n2 = np.array(reference_adj_matrix.shape[0])
        conn1, edge1 = pygm.utils.dense_to_sparse(adj_matrix)
        conn2, edge2 = pygm.utils.dense_to_sparse(reference_adj_matrix)

        gaussian_aff = functools.partial(pygm.utils.gaussian_aff_fn, sigma=.1)
        K = pygm.utils.build_aff_mat(None, edge1, conn1, None, edge2, conn2, None, None, None, None, edge_aff_fn=gaussian_aff)
        X = pygm.rrwm(K, n1, n2)
        return K, X

    def computeRelationships(self, X):
        max_indices = np.argmax(X, axis=1)
        relationships_data = []
        seen = set()
        for idx, max_idx in enumerate(max_indices):
            if max_idx not in seen:  # Check if the relationship is not already reported
                relationships_data.append([idx, max_idx])
                seen.add(idx)  # Add both to avoid duplicates
                seen.add(max_idx)
        return relationships_data
    
    def buildMessage(self, relationships_data):
        relationships_msg = Float32MultiArray()
        relationships_msg.layout.dim = [MultiArrayDimension('matches', 2, 1)]
        relationships_msg.layout.data_offset = 0
        relationships_msg.data = relationships_data
        return relationships_msg
    
    def callback(self, data):
        rows = data.layout.dim[0].size
        cols = data.layout.dim[1].size
        rospy.loginfo(f"Adjacency matrix {rows}x{cols} received.")


        adj_matrix_data = list(data.data)
        adj_matrix = self.buildMatrix(adj_matrix_data, rows, cols)

        K, X = self.solvePygmMatching(adj_matrix, self.reference_adj_matrix)

        relationships_data = self.computeRelationships(X)
        relationships_msg = self.buildMessage(relationships_data)
        self.m_isomorphism_pub.publish(relationships_msg)

if __name__ == "__main__":
    matcher = GraphMatcher()
    rospy.spin()

        

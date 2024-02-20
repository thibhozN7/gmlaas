#!/usr/bin/env python3

import sys
import rospy
import threading
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

import pygmtools as pygm
import numpy as np
import matplotlib.pyplot as plt


import visualization
import computation

from reference import reference_tree

def resize_list_to_matrix(lst, rows, cols):
    return (np.array([lst[i * cols:(i + 1) * cols] for i in range(rows)]))

def callback(data):
    print("hello")
    # Afficher les dimensions de la matrice
    rows = data.layout.dim[0].size
    cols = data.layout.dim[1].size
    rospy.loginfo(f"Matrice Adj {rows}x{cols} reçue.")
    
    # Afficher les données de la matrice
    matrice_data = list(data.data)  # Convertir en liste pour une meilleure lisibilité
    adj_matrice=resize_list_to_matrix(matrice_data, rows, cols)
    rospy.loginfo(f"{adj_matrice}")
    handle_adjacency_matrix(adj_matrice)

def handle_adjacency_matrix(adj_matrice):

    pygm.set_backend('numpy')
    np.random.seed(1)
    K, X = computation.compute_optimal_matching(reference_tree, adj_matrice)
    rospy.loginfo(f"X = {X}")

    #threading.Thread(target=handle_visualization, args=(K, X, adj_matrice)).start()
    handle_relationships(X)

def handle_visualization(K, X, adj_matrice):
    G1, G2, pos1, pos2 = visualization.visualize_graphs(reference_tree, adj_matrice)
    visualization.visualize_affinity_matrix(K)
    visualization.visualize_matching_results(X, adj_matrice, reference_tree)
    visualization.visualize_graph_matching(G1, G2, pos1, pos2, X, len(reference_tree))
    plt.show(block=False)  # Use non-blocking mode
    rospy.loginfo(f"X = {X}")

def handle_relationships(X):
    max_indices = np.argmax(X, axis=1)
    print(max_indices)
    relationships_msg = Float32MultiArray()
    # Set up the layout of the multiarray to have two columns
    relationships_msg.layout.dim = [MultiArrayDimension('matches', 2, 1)]
    relationships_msg.layout.data_offset = 0
    relationships_data = []
    seen = set()

    for idx, max_idx in enumerate(max_indices):
        if max_idx not in seen:  # Check if the relationship is not already reported
            relationships_data.extend([idx, max_idx])
            seen.add(idx)  # Add both to avoid duplicates
            seen.add(max_idx)

    relationships_msg.data = relationships_data
    rospy.loginfo(f"Publishing: {relationships_msg}")
    pub.publish(relationships_msg)


if __name__ == '__main__':

    rospy.init_node('graph_matcher_node', anonymous=False)
    rospy.Subscriber("/chatter", Float32MultiArray, callback)
    pub = rospy.Publisher('/node_relationships', Float32MultiArray, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
        # Add any cleanup code here
        sys.exit(0)
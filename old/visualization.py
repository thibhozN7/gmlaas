
from matplotlib import pyplot as plt
from matplotlib.patches import ConnectionPatch
import networkx as nx
import numpy as np

def visualize_graphs(A1, A2):
    """
    Visualize the two graphs.
    
    Args:
        A1 (numpy.ndarray): Adjacency matrix of the first graph.
        A2 (numpy.ndarray): Adjacency matrix of the second graph.
    
    Returns:
        G1 (networkx.Graph): Graph object of the first graph.
        G2 (networkx.Graph): Graph object of the second graph.
        pos1 (dict): Node positions for the first graph.
        pos2 (dict): Node positions for the second graph.
    """
    plt.figure(figsize=(8, 4))
    G1 = nx.from_numpy_array(A1)
    G2 = nx.from_numpy_array(A2)
    pos1 = nx.spring_layout(G1)
    pos2 = nx.spring_layout(G2)
    plt.subplot(1, 2, 1)
    plt.title('Graph 1')
    nx.draw_networkx(G1, pos=pos1)
    plt.subplot(1, 2, 2)
    plt.title('Graph 2')
    nx.draw_networkx(G2, pos=pos2)
    return G1, G2, pos1, pos2

def visualize_affinity_matrix(K):
    """
    Visualize the affinity matrix.
    
    Args:
        K (numpy.ndarray): Affinity matrix.
    """
    plt.figure(figsize=(4, 4))
    plt.title(f'Affinity Matrix (size: {K.shape[0]}$\\times${K.shape[1]})')
    plt.imshow(K, cmap='Blues')

def visualize_matching_results(X, A2, A1):
    """
    Visualize the matching results.
    
    Args:
        X (numpy.ndarray): Matching matrix.
        X_gt (numpy.ndarray): Ground truth matching matrix.
        A2 (numpy.ndarray): Adjacency matrix of the second graph.
        A1 (numpy.ndarray): Adjacency matrix of the first graph.
    """
    plt.figure(figsize=(8, 4))
    plt.subplot(1, 2, 1)
    plt.title('RRWM Soft Matching Matrix')
    plt.imshow(X, cmap='Blues')

def visualize_graph_matching(G1, G2, pos1, pos2, X, num_nodes):
    """
    Visualize the graph matching results.
    
    Args:
        G1 (networkx.Graph): Graph object of the first graph.
        G2 (networkx.Graph): Graph object of the second graph.
        pos1 (dict): Node positions for the first graph.
        pos2 (dict): Node positions for the second graph.
        X (numpy.ndarray): Matching matrix.
        X_gt (numpy.ndarray): Ground truth matching matrix.
        num_nodes (int): Number of nodes in the graphs.
    """
    plt.figure(figsize=(8, 4))
    ax1 = plt.subplot(1, 2, 1)
    plt.title('Graph 1')
    nx.draw_networkx(G1, pos=pos1)
    ax2 = plt.subplot(1, 2, 2)
    plt.title('Graph 2')
    nx.draw_networkx(G2, pos=pos2)
    
    for i in range(num_nodes):
        j = np.argmax(X[i]).item()
        con = ConnectionPatch(xyA=pos1[i], xyB=pos2[j], coordsA="data", coordsB="data",
                              axesA=ax1, axesB=ax2, color="green")
        plt.gca().add_artist(con)
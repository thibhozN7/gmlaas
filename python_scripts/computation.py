import functools
import numpy as np
import pygmtools as pygm


def compute_optimal_matching(A1, A2):
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
    n1 = len(A2[0])
    n2 = len(A2[0])
    conn1, edge1 = pygm.utils.dense_to_sparse(A1)
    conn2, edge2 = pygm.utils.dense_to_sparse(A2)
    gaussian_aff = functools.partial(pygm.utils.gaussian_aff_fn, sigma=.1)
    #K = pygm.utils.build_aff_mat(None, edge1, conn1, None, edge2, conn2, n1, None, n2, None, edge_aff_fn=gaussian_aff)
    K = pygm.utils.build_aff_mat(None, edge1, conn1, None, edge2, conn2, n1, None, n2, None, edge_aff_fn=gaussian_aff)
    X = pygm.rrwm(K, n1, n2)
    return K, X
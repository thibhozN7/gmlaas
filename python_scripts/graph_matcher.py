#!/usr/bin/env python3
import rospy
import numpy as np
import pygmtools as pygm
import functools
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import yaml
import os



class GraphMatcher:
    def __init__(self, fake=False):

        rospy.init_node("graph_matcher_py", anonymous=False)        
        rospy.loginfo("Starting graph matcher...")

        if fake:
            topic = "/graph_building/fake/adjency_matrix"
            rospy.loginfo("...Using fake adjency matrix topic.")
        else:
            topic = "/graph_building/adjency_matrix"
            rospy.loginfo("...Using real adjency matrix topic.")
        
        self.m_adj_matrix_sub = rospy.Subscriber(topic,Float32MultiArray, self.callback,queue_size=10)

        self.m_isomorphism_pub = rospy.Publisher("/graph_matching/isomorphism_list",Float32MultiArray, queue_size=10)

        self.reference_adj_matrix = self.initReferenceAdjMatrix()
        rospy.loginfo("Graph matcher initialized.")   
    
    def buildMatrix(self, data, rows, cols):
        matrix = np.array(data).reshape(rows, cols)
        return matrix

    def initReferenceAdjMatrix(self):

        package_path = os.path.dirname(os.path.dirname(__file__))  # Get the directory of the ROS package (two levels up from the script file)
        file_path = os.path.join(package_path, "text_files", "reference_adjacency_matrix.txt")
        
        # Open the file in read mode
        with open(file_path, 'r') as file:
            # Read the contents of the file into a variable
            text = file.read()
        # Parse the YAML text
        parsed_data = yaml.safe_load(text)
        data = parsed_data['data']
        layout_dim = parsed_data['layout']['dim']

        # Create the layout dimensions
        layout = MultiArrayLayout()
        layout.dim = [MultiArrayDimension(label=dim['label'], size=dim['size'], stride=dim['stride']) for dim in layout_dim]
        layout.data_offset = parsed_data['layout']['data_offset']

        # Create a Float32MultiArray message
        reference_msg = Float32MultiArray(layout=layout, data=data)
        self.reference_adj_matrix = self.buildMatrix(reference_msg.data, layout_dim[0]['size'], layout_dim[1]['size'])
        
        rospy.loginfo("Reference adjacency matrix:")
        rospy.loginfo(self.reference_adj_matrix)
        return self.reference_adj_matrix
    
    
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

        print(input_adj_matrix.shape)
        print(reference_adj_matrix.shape)

        n1 = np.array(input_adj_matrix.shape)
        n2 = np.array(reference_adj_matrix.shape)
        conn1, edge1 = pygm.utils.dense_to_sparse(input_adj_matrix)
        conn2, edge2 = pygm.utils.dense_to_sparse(reference_adj_matrix)

        gaussian_aff = functools.partial(pygm.utils.gaussian_aff_fn, sigma=.1)
        K = pygm.utils.build_aff_mat(None, edge1, conn1, None, edge2, conn2, None, None, None, None, edge_aff_fn=gaussian_aff)
        X = pygm.rrwm(K, n1, n2)
        return K, X
    
    def computeRelationships(self, X):
        """
        Compute the relationships between the nodes of the input and reference graphs.
        
        Args:
            X (numpy.ndarray): Matching matrix => X[i, j] is the probability that node i in the input graph is matched to node j in the reference graph.
        Returns:
            relationships_data (list): List of relationships between the nodes of the input and reference graphs.
        """
        relationships_data = []
        max_idx = np.argmax(X, axis=1)
        for i in range(X.shape[0]):
                    relationships_data.append(max_idx[i])
        return relationships_data


    def buildMessage(self, relationships_data):
        relationships_msg = Float32MultiArray()
        relationships_msg.data = list(relationships_data)
        return relationships_msg
    
    def callback(self, msg):
        data = msg.data
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        rospy.loginfo(f"Adjacency matrix {rows}x{cols} received...")

        adj_matrix = self.buildMatrix(data, rows, cols)
        rospy.loginfo(f".... : \n{adj_matrix}")

        rospy.loginfo("Solving graph matching...")
        K, X = self.solvePygmMatching(adj_matrix, self.reference_adj_matrix)
        rospy.loginfo(f"...done \nMatching matrix: \n{X}")
        
        rospy.loginfo("Computing relationships...")
        relationships_data = self.computeRelationships(X)
        relationships_msg = self.buildMessage(relationships_data)
        rospy.loginfo(f"...done")

        rospy.loginfo("Publishing relationships...")
        self.m_isomorphism_pub.publish(relationships_msg)
        rospy.loginfo(f"...Published:\n {relationships_msg}")

if __name__ == "__main__":
    matcher = GraphMatcher(True)
    rospy.spin()

        

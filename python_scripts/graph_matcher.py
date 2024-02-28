#!/usr/bin/env python3
import rospy
import numpy as np
import pygmtools as pygm
import functools
from std_msgs.msg import Int32MultiArray,Float32MultiArray, MultiArrayDimension, MultiArrayLayout, Header
import yaml
import os
from collections import OrderedDict
from gmlaas.msg import GraphBuilderMsg, GraphMatcherMsg

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
    
    def listener(self, msg):
        self.header = msg.header
        adjacency_matrix = msg.adjacency_matrix
        self.indexed_matrix = msg.indexed_matrix
        self.adjacency_matrix=self.buildMatrix(adjacency_matrix, len(self.indexed_matrix), len(self.indexed_matrix))
        self.tags_id = msg.tags_id
        self.coordinate_matrix = msg.coordinate_matrix

    def publisher(self):
        # Create a CustomMessage
        custom_msg = GraphMatcherMsg()
        custom_msg.header = self.header
        # Add adjacency matrix data
        custom_msg.adjacency_matrix = [ float(value) for row in self.adjacency_matrix for value in row ]
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
    

    def solvePygmMatching(self, input_adj_matrix, reference_adj_matrix):
        """
        Compute the optimal matching between two graphs.
        
        Args:
            current_adj_matrix (numpy.ndarray): Adjacency matrix of the current graph.
            reference_adj_matrix (numpy.ndarray): Adjacency matrix of reference graph.
        Returns:
            K (numpy.ndarray): Affinity matrix.
            X (numpy.ndarray): Matching matrix => X[i, j] is the probability that node i in the current graph is matched to node j in the reference graph.
        """
        pygm.set_backend('numpy')
        np.random.seed(1)

        print(current_adj_matrix.shape)
        print(reference_adj_matrix.shape)

        n1 = np.array(current_adj_matrix.shape)
        n2 = np.array(reference_adj_matrix.shape)
        conn1, edge1 = pygm.utils.dense_to_sparse(current_adj_matrix)
        conn2, edge2 = pygm.utils.dense_to_sparse(reference_adj_matrix)

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
        rospy.loginfo(f"Adjacency matrix {self.adjacency_matrix.shape[0]}x{self.adjacency_matrix.shape[1]} received...")

        rospy.loginfo("Solving graph matching...")
        K, X = self.solvePygmMatching(self.adjacency_matrix, self.reference_adj_matrix)
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

        

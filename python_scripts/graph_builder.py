#!/usr/bin/env python3
import math

class Edge:
    def __init__(self, from_node_id, to_node_id, type):
        self.from_node_id = from_node_id
        self.to_node_id = to_node_id
        self.type = type
        self.length = 0  # Calculate later if needed

    def calculate_edge_length(self, graph):
        #print("from_node_id",self.from_node_id)
        for node in graph.nodes:
            if node.id==self.from_node_id:
                from_node=node
            elif node.id==self.to_node_id:
                to_node=node
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        self.length = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        theta = math.atan2(dy, dx)  # Azimuthal angle
        phi = math.acos(dz / self.length) if self.length != 0 else 0  # Polar angle
        return(self.length, theta, phi)
    
class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def __del__(self):
        for node in self.nodes:
            del node
        for edge in self.edges:
            del edge

    def add_node(self, node):
        self.nodes.append(node)

    def add_edge(self, edge):
        self.edges.append(edge)

    def calculate_adjacency_matrix(self):
        node_mapping = self.create_node_mapping()
        adjacency_matrix = [[0] * len(self.nodes) for _ in range(len(self.nodes))]

        for edge in self.edges:
            #print("from node")
            #print(edge.from_node_id.id)
            from_id = node_mapping[edge.from_node_id]
            to_id = node_mapping[edge.to_node_id]
            adjacency_matrix[from_id][to_id] = edge.length
            #adjacency_matrix[to_id][from_id] = edge.length  # Comment out for directed graphs

        return adjacency_matrix

    def print_adjacency_matrix(self, adjacency_matrix):
        print("Adjacency Matrix:")
        for row in adjacency_matrix:
            print(" ".join(map(str, row)))

    def create_node_mapping(self):
        node_mapping = {}
        for index, node in enumerate(self.nodes):
            node_mapping[node.id] = index
            node.mapped_id = index
        return node_mapping

    def print_node_mapping(self, node_mapping):
        print("Node Mapping:")
        for original_id, sequential_index in node_mapping.items():
            print(f"Original ID: {original_id} -> Sequential Index: {sequential_index}")

class Node:
    def __init__(self, id, x, y, z):
        self.id = id
        self.mapped_id = -1
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.children = []

    def add_child(self, child):
        self.children.append(child)

import xml.etree.ElementTree as ET
import random
from math import sqrt, atan2, asin, degrees
import numpy as np
from Cube import Cube
from Stick import Stick

class URDFGenerator:
    def __init__(self, num_cubes):
        self.num_cubes = num_cubes
        self.cubes = {}
        self.cube_joints = {}
        self.cube_colors = {}
        self.stick_pos = {}
        self.stick_rpy = {}
        self.stick_length = {}
        self.sticks = {}
        self.stick_joints = {}
        self.stick_colors = {}

    def generate_urdf(self):
        root = ET.Element("robot")

        # Adding trunk cube
        cubetmp = Cube(0)
        self.cubes[0] = cubetmp.link
        self.cube_colors[0] = cubetmp.color
        self.stick_pos[0] = [0, 0, 0]

        # Generating tree structure
        scaling_x = 0.4
        scaling_y = 0.3
        scaling_z = 0.3

        for i in range(1, self.num_cubes):
            parent_id = (i - 1) // 2
            x = scaling_x
            y = scaling_y
            z = scaling_z
            relative_position = [random.uniform(-x, x), random.uniform(-y * 0.75, y), random.uniform(0.9 * z, z)]
            self.stick_pos[i], self.stick_rpy[i], self.stick_length[i] = self.find_frame_between_points(relative_position)
            cubetmp = Cube(i)
            self.cubes[i] = cubetmp.link
            self.cube_joints[i] = self.create_joint_element(i, f"cube_link_{parent_id}", f"cube_link_{i}", relative_position)
            self.cube_colors[i] = cubetmp.color
            scaling_x += 0.07
            scaling_y += 0.03
            scaling_z += 0.04

        # Generating sticks
        for i in range(1, self.num_cubes):
            parent_id = (i - 1) // 2
            child_id = i
            sticktmp = Stick(i,self.stick_length[i])
            self.sticks[i] = sticktmp.link
            self.stick_joints[i] = self.create_stick_joint_element(i, f"cube_link_{parent_id}", f"stick_link_{i}", self.stick_pos[i], self.stick_rpy[i])
            self.stick_colors[i] = sticktmp.color

        # Create URDF tree
        for cube in self.cubes.values():
            root.append(cube)
        for cube_joint in self.cube_joints.values():
            root.append(cube_joint)
        for cube_color in self.cube_colors.values():
            root.append(cube_color)
        for stick in self.sticks.values():
            root.append(stick)
        for stick_joint in self.stick_joints.values():
            root.append(stick_joint)
        for stick_color in self.stick_colors.values():
            root.append(stick_color)

        tree = ET.ElementTree(root)
        return tree

    def create_joint_element(self, cube_joint_id, parent_link, child_link, position):
        joint = ET.Element("joint", name=f"cube_joint_{cube_joint_id}", type="fixed")
        parent = ET.SubElement(joint, "parent", link=parent_link)
        child = ET.SubElement(joint, "child", link=child_link)
        origin = ET.SubElement(joint, "origin", xyz=f"{position[0]} {position[1]} {position[2]}", rpy="0 0 0")
        axis = ET.SubElement(joint, "axis", xyz="0 0 1")
        return joint

    def create_stick_joint_element(self, stick_joint_id, parent_link, child_link, position, orientation):
        stick_joint = ET.Element("joint", name=f"stick_joint_{stick_joint_id}", type="fixed")
        parent = ET.SubElement(stick_joint, "parent", link=parent_link)
        child = ET.SubElement(stick_joint, "child", link=child_link)
        origin = ET.SubElement(stick_joint, "origin", xyz=f"{position[0]} {position[1]} {position[2]}", rpy=f"{orientation[0]} {orientation[1]} {orientation[2]}")
        axis = ET.SubElement(stick_joint, "axis", xyz="0 0 1")
        return stick_joint

    def find_frame_between_points(self, point):
        vector = np.array(point)
        midpoint = np.array(point)/ 2
        axis_z = vector / np.linalg.norm(vector)
        roll = atan2(-axis_z[1], axis_z[2])
        pitch = asin(axis_z[0])
        yaw = 0
        return midpoint, [roll, pitch, yaw], sqrt(pow(point[0],2)+pow(point[1],2)+pow(point[2],2))*0.8


if __name__ == "__main__":
    num_cubes = 15
    urdf_generator = URDFGenerator(num_cubes)
    urdf_tree = urdf_generator.generate_urdf()
    urdf_tree.write("treetest.urdf", encoding="utf-8", xml_declaration=True, method="xml", short_empty_elements=False)
    print("URDF file generated successfully.")

import xml.etree.ElementTree as ET
import random
import math
import os

pathfile = os.path.abspath(__file__)
parentdir = os.path.dirname(pathfile)

def create_cube_color(cube_id, material_name):
    cube = ET.Element("gazebo", reference=f"link_{cube_id}")
    material = ET.SubElement(cube, "material")
    material.text = material_name
    return cube

def create_link_element(link_id, shape='box', dimensions=(0.1, 0.1, 0.1)):
    link = ET.Element("link", name=f"link_{link_id}")
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    visual_geometry = ET.SubElement(visual, "geometry")
    if shape == 'box':
        ET.SubElement(visual_geometry, "box", size=" ".join(map(str, dimensions)))
    elif shape == 'cylinder':
        ET.SubElement(visual_geometry, "cylinder", radius=str(dimensions[0]), length=str(dimensions[1]))
    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    collision_geometry = ET.SubElement(collision, "geometry")
    if shape == 'box':
        ET.SubElement(collision_geometry, "box", size=" ".join(map(str, dimensions)))
    elif shape == 'cylinder':
        ET.SubElement(collision_geometry, "cylinder", radius=str(dimensions[0]), length=str(dimensions[1]))
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value="1.0")
    ET.SubElement(inertial, "inertia", ixx="0.01", ixy="0", ixz="0", iyy="0.01", iyz="0", izz="0.01")
    return link

def create_joint_element(joint_id, parent_link, child_link, position, orientation=(0,0,0)):
    joint = ET.Element("joint", name=f"joint_{joint_id}", type="fixed")
    ET.SubElement(joint, "parent", link=parent_link)
    ET.SubElement(joint, "child", link=child_link)
    ET.SubElement(joint, "origin", xyz=f"{position[0]} {position[1]} {position[2]}", rpy=f"{orientation[0]} {orientation[1]} {orientation[2]}")
    ET.SubElement(joint, "axis", xyz="0 0 1")
    return joint

def calculate_cylinder_properties(parent_position, child_position):
    dx, dy, dz = (child_position[i] - parent_position[i] for i in range(3))
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    yaw, pitch = math.atan2(dy, dx), math.atan2(math.sqrt(dx**2 + dy**2), dz) if dz != 0 else 0.0
    return distance, (0.0, pitch, yaw)

def generate_urdf(num_cubes):
    root = ET.Element("robot", name="tree")
    links, colors, cylinder_links, cylinder_joints = {}, {}, [], []

    # Create and add the root link first
    links[0] = create_link_element(0)
    root.append(links[0])
    colors[0] = create_cube_color(0, "testing/AprilTag_tag36h11_ID_0000")
    root.append(colors[0])

    positions = {0: (0, 0, 0)}

    scaling_x, scaling_y, scaling_z, cylinder_radius, margin = 0.4, 0.3, 0.3, 0.02, 0.05

    for i in range(1, num_cubes):
        parent_id = (i - 1) // 2
        cube_position = (random.uniform(-scaling_x, scaling_x), random.uniform(-scaling_y, scaling_y), random.uniform(scaling_z, 2*scaling_z))
        positions[i] = cube_position

        links[i] = create_link_element(i)
        root.append(links[i])
        colors[i] = create_cube_color(i, f"testing/AprilTag_tag36h11_ID_{str(i).zfill(4)}")
        root.append(colors[i])

        distance, orientation = calculate_cylinder_properties(positions[parent_id], cube_position)
        cylinder_length = distance - margin

        cylinder_id = f"cylinder_{i}"
        cylinder_links.append(create_link_element(cylinder_id, 'cylinder', (cylinder_radius, cylinder_length)))
        midpoint = tuple((p + c) / 2 for p, c in zip(positions[parent_id], cube_position))
        cylinder_joints.append(create_joint_element(f"joint_cylinder_{i}", f"link_{parent_id}", f"link_{cylinder_id}", midpoint, orientation))

    for cylinder_link in cylinder_links:
        root.append(cylinder_link)
    for cylinder_joint in cylinder_joints:
        root.append(cylinder_joint)

    tree = ET.ElementTree(root)
    return tree

def save_urdf(tree, filename):
    with open(filename, "wb") as f:
        tree.write(f, encoding="utf-8", xml_declaration=True, method="xml", short_empty_elements=True)

if __name__ == "__main__":
    num_cubes = 15
    urdf_tree = generate_urdf(num_cubes)
    save_urdf(urdf_tree, os.path.join(parentdir, "tree_with_branches.urdf"))
    print("URDF file generated successfully.")

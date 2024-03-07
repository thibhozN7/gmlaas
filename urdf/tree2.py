import xml.etree.ElementTree as ET
import random
from math import sqrt, atan2, asin, pi
import numpy as np

def create_cube_color(cube_id, material_name):
    cube = ET.Element("gazebo", reference=f"link_{cube_id}")
    material = ET.SubElement(cube, "material")
    material.text = material_name
    return cube

def create_stick_color(stick_id):
    stick_color = ET.Element("gazebo", reference=f"stick_{stick_id}")
    material = ET.SubElement(stick_color, "material")
    material.text = "Blue"
    return stick_color

def create_link_element(link_id):
    link = ET.Element("link", name=f"link_{link_id}")

    # Visual
    visual = ET.SubElement(link, "visual")
    visual_origin = ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_box = ET.SubElement(visual_geometry, "box", size="0.1 0 0.1")

    # Collision
    collision = ET.SubElement(link, "collision")
    collision_origin = ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    collision_geometry = ET.SubElement(collision, "geometry")
    collision_box = ET.SubElement(collision_geometry, "box", size="0.1 0.1 0.1")

    # Inertial
    inertial = ET.SubElement(link, "inertial")
    inertial_origin = ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    inertial_mass = ET.SubElement(inertial, "mass", value="1.0")
    inertial_inertia = ET.SubElement(inertial, "inertia", ixx="0.01", ixy="0", ixz="0", iyy="0.01", iyz="0", izz="0.01")

    return link

def create_stick_element(stick_id, length):
    stick = ET.Element("link", name=f"stick_{stick_id}")

    # Visual
    visual = ET.SubElement(stick, "visual")
    visual_origin = ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_cylinder = ET.SubElement(visual_geometry, "cylinder", radius="0.02", length=f"{length}")

    # Collision
    collision = ET.SubElement(stick, "collision")
    collision_origin = ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    collision_geometry = ET.SubElement(collision, "geometry")
    collision_cylinder = ET.SubElement(collision_geometry, "cylinder", radius="0.02", length=f"{length}")

    # Inertial
    inertial = ET.SubElement(stick, "inertial")
    inertial_origin = ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    inertial_mass = ET.SubElement(inertial, "mass", value="1.0")
    inertial_inertia = ET.SubElement(inertial, "inertia", ixx="0.01", ixy="0", ixz="0", iyy="0.01", iyz="0", izz="0.01")

    return stick

def create_joint_element(joint_id, parent_link, child_link, position):
    joint = ET.Element("joint", name=f"joint_{joint_id}", type="fixed")

    parent = ET.SubElement(joint, "parent", link=parent_link)
    child = ET.SubElement(joint, "child", link=child_link)
    origin = ET.SubElement(joint, "origin", xyz=f"{position[0]} {position[1]} {position[2]}", rpy="0 0 0")

    axis = ET.SubElement(joint, "axis", xyz="0 0 1")

    return joint

def create_stick_joint_element(joint_id, parent_link, child_link, position, orientation):
    stick_joint = ET.Element("joint", name=f"stick_joint_{joint_id}", type="fixed")

    parent = ET.SubElement(stick_joint, "parent", link=parent_link)
    child = ET.SubElement(stick_joint, "child", link=child_link)
    origin = ET.SubElement(stick_joint, "origin", xyz=f"{position[0]} {position[1]} {position[2]}", rpy=f"{orientation[0]} {orientation[1]} {orientation[2]}")

    axis = ET.SubElement(stick_joint, "axis", xyz="0 0 1")

    return stick_joint

def find_frame_between_points(point1, point2):
    # Calcola il vettore che va dal primo punto al secondo punto
    vector = np.array(point2) - np.array(point1)
    
    # Trova il punto medio tra i due punti
    midpoint = (np.array(point1) + np.array(point2)) / 2
    
    # Normalizza il vettore per ottenere la direzione dell'asse z
    axis_z = vector / np.linalg.norm(vector)
    
    # Calcola gli angoli di Eulero (roll, pitch, yaw) dalla matrice di rotazione
    roll = atan2(-axis_z[1], axis_z[2])
    pitch = asin(axis_z[0])
    yaw = 0  # Non c'è una singola soluzione per yaw, possiamo scegliere arbitrariamente un valore
    
    return midpoint, [roll, pitch, yaw]

def generate_urdf(num_cubes):
    root = ET.Element("robot")

    links = {}
    joints = {}
    color = {}
    cube_position = {}

    # Adding trunk cube
    links[0] = create_link_element(0)
    color[0] = create_cube_color(0, "testing/AprilTag_tag36h11_ID_0000")
    cube_position[0] = [0, 0, 0]

    # Generating tree structure
    scaling_x = 0.4
    scaling_y = 0.3
    scaling_z = 0.3

    for i in range(1, num_cubes):
        parent_id = (i - 1) // 2
        x = scaling_x
        y = scaling_y
        z = scaling_z
        cube_position[i] = [random.uniform(-x, x), random.uniform(-y * 0.75, y), random.uniform(z, z)]
        links[i] = create_link_element(i)
        joints[i] = create_joint_element(i, f"link_{parent_id}", f"link_{i}", cube_position[i])
        id_str = str(i).zfill(4)
        color[i] = create_cube_color(i, f"testing/AprilTag_tag36h11_ID_{id_str}")

    sticks = {}
    stick_joints = {}
    stick_colors = {}

    for i in range(1, num_cubes):
        parent_id = (i - 1) // 2
        child_id = i
        pos, orient = find_frame_between_points(cube_position[parent_id], cube_position[child_id])
        length = sqrt((cube_position[child_id][0] - cube_position[parent_id][0]) ** 2 +
                     (cube_position[child_id][1] - cube_position[parent_id][1]) ** 2 +
                     (cube_position[child_id][2] - cube_position[parent_id][2]) ** 2)
        sticks[i] = create_stick_element(i, length)
        stick_joints[i] = create_stick_joint_element(i, f"link_{parent_id}", f"stick_{i}", pos, orient)
        stick_colors[i] = create_stick_color(i)

    for link in links.values():
        root.append(link)
    for joint in joints.values():
        root.append(joint)
    for color in color.values():
        root.append(color)
    for stick in sticks.values():
        root.append(stick)
    for stick_joint in stick_joints.values():
        root.append(stick_joint)
    for stick_color in stick_colors.values():
        root.append(stick_color)

    # Create URDF tree
    tree = ET.ElementTree(root)
    return tree

def save_urdf(tree, filename):
    with open(filename, "wb") as f:
        tree.write(f, encoding="utf-8", xml_declaration=True, method="xml", short_empty_elements=False)

if __name__ == "__main__":
    num_cubes = 15
    urdf_tree = generate_urdf(num_cubes)
    save_urdf(urdf_tree, "treewithvisiblejoint.urdf")
    print("URDF file generated successfully.")

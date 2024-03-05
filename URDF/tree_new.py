import xml.etree.ElementTree as ET
import random

def create_cube_color(cube_id, material_name):
    cube = ET.Element("gazebo", reference=f"link_{cube_id}")
    material = ET.SubElement(cube, "material")
    material.text = material_name
    return cube

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

def create_joint_element(joint_id, parent_link, child_link,position):
    joint = ET.Element("joint", name=f"joint_{joint_id}", type="fixed")

    parent = ET.SubElement(joint, "parent", link=parent_link)
    child = ET.SubElement(joint, "child", link=child_link)
    orientation=random.uniform(-3.14, 3.14)
    origin = ET.SubElement(joint, "origin", xyz=f"{position[0]} {position[1]} {position[2]}", rpy=f"0 0 0")

    axis = ET.SubElement(joint, "axis", xyz="0 0 1")

    return joint

def generate_urdf(num_cubes):
    root = ET.Element("robot")

    links = {}
    joints = {}
    color = {}
    # Adding trunk cube
    links[0] = create_link_element(0)
    color[0] = create_cube_color(0, "testing/AprilTag_tag36h11_ID_0000")

    # Generating tree structure
    for i in range(1, num_cubes):
        parent_id = (i - 1) // 2
        cube_position = (random.uniform(-2, 2), random.uniform(-0.5,0.5), random.uniform(0.3, 1))
        links[i] = create_link_element(i)
        
        joints[i] = create_joint_element(i, f"link_{parent_id}", f"link_{i}", cube_position)
        id=str(i).zfill(4)
        color[i] = create_cube_color(i, f"testing/AprilTag_tag36h11_ID_{id}")

    for link in links.values():
        root.append(link)
    for joint in joints.values():
        root.append(joint)
    for color in color.values():
        root.append(color)

    # Create URDF tree
    tree = ET.ElementTree(root)
    return tree

def save_urdf(tree, filename):
    with open(filename, "wb") as f:
        tree.write(f, encoding="utf-8", xml_declaration=True, method="xml", short_empty_elements=False)

if __name__ == "__main__":
    num_cubes = 15
    urdf_tree = generate_urdf(num_cubes)
    save_urdf(urdf_tree, "tree.urdf")
    print("URDF file generated successfully.")


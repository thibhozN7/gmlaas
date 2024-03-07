import xml.etree.ElementTree as ET
import random
import math
import os

# Function to create a visual representation of a link in the URDF with a box shape
def create_link_cube(link_id, position=(0, 0, 0), dimensions=(0.1, 0.1, 0.1)):
    link = ET.Element("link", name=f"link_{link_id}")
    visual = ET.SubElement(link, "visual")
    origin = ET.SubElement(visual, "origin", xyz=f"{position[0]} {position[1]} {position[2]}", rpy="0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", size=f"{dimensions[0]} {dimensions[1]} {dimensions[2]}")
    return link

# Function to create a visual representation of a link in the URDF with a cylinder shape
def create_link_cylinder(link_id, length, orientation, radius=0.02):
    link = ET.Element("link", name=f"link_cylinder_{link_id}")
    visual = ET.SubElement(link, "visual")
    origin = ET.SubElement(visual, "origin", xyz="0 0 0", rpy=f"{orientation[0]} {orientation[1]} {orientation[2]}")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "cylinder", radius=str(radius), length=str(length))
    return link

# Function to calculate the properties of a cylinder that connects a parent and a child cube
def cylinder_properties_computing(parent_pos, child_pos):
    dx, dy, dz = [c - p for c, p in zip(child_pos, parent_pos)]
    length = math.sqrt(dx**2 + dy**2 + dz**2)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(dz, math.sqrt(dx**2 + dy**2))
    roll = 0.0  # Assuming no roll for simplicity
    return length, [roll, pitch - math.pi/2, yaw]  # Adjusting pitch for the URDF coordinate system

# Function to create the joint element in URDF
def create_joint(joint_id, parent_link, child_link, joint_type="fixed"):
    joint = ET.Element("joint", name=f"joint_{joint_id}", type=joint_type)
    ET.SubElement(joint, "parent", link=parent_link)
    ET.SubElement(joint, "child", link=child_link)
    ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
    return joint

def generate_urdf(num_cubes):
    root = ET.Element("robot", name="tree_structure", xmlns="http://www.ros.org/wiki/urdf")

    # Create the root link (base link of the robot)
    root_link = create_link_cube(0)
    root.append(root_link)

    # Create links and joints
    for i in range(1, num_cubes):
        # Generate random positions and create cubes
        pos = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0, 2))
        cube_link = create_link_cube(i, pos)
        root.append(cube_link)

        # Compute cylinder properties and create cylinders
        length, orientation = cylinder_properties_computing((0, 0, 0), pos) # Simplified for illustration
        cylinder_link = create_link_cylinder(i, length, position=pos, orientation=orientation)
        root.append(cylinder_link)

        # Create joints to connect cubes and cylinders
        parent_joint = create_joint(f"{i-1}_to_cylinder_{i}", f"link_{i-1}", f"link_cylinder_{i}")
        root.append(parent_joint)
        child_joint = create_joint(f"cylinder_{i}_to_{i}", f"link_cylinder_{i}", f"link_{i}")
        root.append(child_joint)

    # Write the URDF to a file
    tree = ET.ElementTree(root)
    tree.write("tree_structure.urdf", encoding="utf-8", xml_declaration=True)

if __name__ == "__main__":
    num_cubes = 15  # Define the number of cubes you want in your structure
    generate_urdf(num_cubes)
    print("URDF file generated successfully.")
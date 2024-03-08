import xml.etree.ElementTree as ET
import random
import math
import os

def create_tag(tag_id, material_name):
    tag = ET.Element("tag", reference=f"tag_link_{tag_id}")
    material = ET.SubElement(tag, "material")
    material.text = material_name
    return tag

def create_link_cube(link_id, position=(0,0,0), orientation=(0,0,0), dimensions=(0.1, 0.1, 0.1)):
    link = ET.Element("link", name=f"link_cube_{link_id}")

    # Visual
    visual = ET.SubElement(link, "visual")
    visual_origin = ET.SubElement(visual, "origin", xyz=" ".join(str(p) for p in position), rpy=" ".join(str(o) for o in orientation))
    visual_geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(visual_geometry, "box", size=" ".join(map(str, dimensions)))
    
    # # Collision
    # collision = ET.SubElement(link, "collision")
    # collision_origin = ET.SubElement(collision, "origin", xyz=" ".join(str(p) for p in position), rpy=" ".join(str(o) for o in orientation))
    # collision_geometry = ET.SubElement(collision, "geometry")
    # ET.SubElement(collision_geometry, "box", size=" ".join(map(str, dimensions)))

    # # Inertial
    # inertial = ET.SubElement(link, "inertial")
    # inertial_origin = ET.SubElement(inertial, "origin", xyz=" ".join(str(p) for p in position), rpy=" ".join(str(o) for o in orientation))
    # mass = ET.SubElement(inertial, "mass", value="1.0")
    # inertia = ET.SubElement(inertial, "inertia", ixx="0.01", ixy="0", ixz="0", iyy="0.01", iyz="0", izz="0.01")

    return link

def create_link_cylinder(link_id, length, orientation, radius=0.02, position=(0,0,0)):
    link = ET.Element("link", name=f"link_cylinder_{link_id}")

    # Visual
    visual = ET.SubElement(link, "visual")
    visual_origin = ET.SubElement(visual, "origin", xyz=" ".join(str(p) for p in position), rpy=" ".join(str(o) for o in orientation))
    visual_geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(visual_geometry, "cylinder", radius=str(radius), length=str(length))
    
    # # Collision
    # collision = ET.SubElement(link, "collision")
    # collision_origin = ET.SubElement(collision, "origin", xyz=" ".join(str(p) for p in position), rpy=" ".join(str(o) for o in orientation))
    # collision_geometry = ET.SubElement(collision, "geometry")
    # ET.SubElement(collision_geometry, "cylinder", radius=str(radius), length=str(length))

    # # Inertial
    # inertial = ET.SubElement(link, "inertial")
    # inertial_origin = ET.SubElement(inertial, "origin", xyz=" ".join(str(p) for p in position), rpy=" ".join(str(o) for o in orientation))
    # mass = ET.SubElement(inertial, "mass", value="1.0")
    # inertia = ET.SubElement(inertial, "inertia", ixx="0.01", ixy="0", ixz="0", iyy="0.01", iyz="0", izz="0.01")

    return link
  
def create_joint(joint_id, parent_link_name, child_link_name, position=(0, 0, 0), orientation=(0, 0, 0), type="fixed"):
    joint = ET.Element("joint", name=f"joint_{joint_id}", type=f"{type}")
    ET.SubElement(joint, "parent", link=f"{parent_link_name}")
    ET.SubElement(joint, "child", link=f"{child_link_name}")
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, position)), rpy=" ".join(map(str, orientation)))
    return joint


def cylinder_proprieties_computing(parent_cube_pos, child_cube_pos):
    dx = child_cube_pos[0] - parent_cube_pos[0]
    dy = child_cube_pos[1] - parent_cube_pos[1]
    dz = child_cube_pos[2] - parent_cube_pos[2]

    length = math.sqrt(dx**2 + dy**2 + dz**2)
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.sqrt(dx**2 + dy**2))

    # Roll is zero for a cylinder
    roll = 0.0
    
    return (length, [roll, pitch - math.pi/2, yaw])

def cubes_layout(num_cubes):
    # 3D layout initial distribution of child tags
    scaling_x = 0.4
    scaling_y = 0.3
    scaling_z = 0.3

    cube_position = [[0, 0, 0]]  # Start with root position
    cylinder_proprieties = [(0, [0, 0, 0])]  # Start with dummy properties for root
    
    for i in range(1, num_cubes):
        # Simple parent rule for tree structure generation (1 parent and a maximum of 2 children)
        parent_cube_id = (i - 1) // 2
        parent_cube_position = cube_position[parent_cube_id]

        # 3D layout uniform distribution of child tags
        x = random.uniform(-scaling_x, scaling_x)
        y = random.uniform(-scaling_y, scaling_y)
        z = random.uniform(scaling_z, 2 * scaling_z)
        
        child_cube_position = [x, y, z]
        cube_position.append(child_cube_position)

        # Cylinder i connecting child cube i to the parent (i-1)//2
        cylinder_properties = cylinder_proprieties_computing(parent_cube_position, child_cube_position)
        cylinder_proprieties.append(cylinder_properties)

        # Update scaling factors
        scaling_x += 0.07
        scaling_y += 0.03
        scaling_z += 0.04

    return cube_position, cylinder_proprieties


def generate_urdf(num_cubes, cube_position, cylinder_proprieties):

    root = ET.Element("robot", name="tree", xmlns="http://www.ros.org/wiki/urdf")

    # root tag
    links = [create_link_cube(0)]
    joints = [0]*num_cubes
    #colors = [create_tag(i, "Gazebo/Red")]

    for i in range(1, num_cubes):

        parent_cube_id = (i - 1) // 2
        link_parent_cube_name = f"link_cube_{parent_cube_id}"
        child_cube_id = i
        link_child_cube_name = f"link_cube_{child_cube_id}"
        cylinder_id = i
        link_cylinder_name = f"link_cylinder_{cylinder_id}"

        parent_joint = create_joint(parent_cube_id, link_parent_cube_name, link_cylinder_name, cube_position[parent_cube_id])
        child_joint = create_joint(child_cube_id, link_cylinder_name, link_child_cube_name, cube_position[child_cube_id])

        [length,orientation] = cylinder_proprieties[cylinder_id]
        cylinder = create_link_cylinder(cylinder_id, length, orientation)

        # Updating cubes
        links.append(create_link_cube(i))
        # Updating joints
        joints[parent_cube_id] = parent_joint
        joints[child_cube_id] = child_joint
        # Updating cylinders
        links.append(cylinder)
        # Updating colors
    
    for link in links:
        root.append(link)
    for joint in joints:
        root.append(joint)
    #for color in colors:
        #root.append(color)

    # Creating URDF here
    tree = ET.ElementTree(root)

    return tree

def save_urdf(tree, filename):
    tree.write(filename, encoding='utf-8', xml_declaration=True)

if __name__ == "__main__":
    num_cubes = 15
    cube_position, cylinder_proprieties = cubes_layout(num_cubes)
    urdf_tree = generate_urdf(num_cubes, cube_position, cylinder_proprieties)

    filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tree_with_branches.urdf")
    save_urdf(urdf_tree, filename)

    print("URDF file generated successfully.")

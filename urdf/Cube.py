import xml.etree.ElementTree as ET

class Cube:
    def __init__(self, cube_id):
        self.id = cube_id
        self.color = self.create_cube_color()
        self.link = self.create_link_element()

    def create_cube_color(self):
        cube_color = ET.Element("gazebo", reference=f"cube_link_{self.id}")
        material = ET.SubElement(cube_color, "material")
        material.text = f"testing/AprilTag_tag36h11_ID_{str(self.id).zfill(4)}"
        return cube_color
    
    def create_link_element(self):
        link = ET.Element("link", name=f"cube_link_{self.id}")

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
import xml.etree.ElementTree as ET

class Stick:
    def __init__(self, stick_id,length):
        self.id = stick_id
        self.length = length
        self.link = self.create_link_element()
        self.color = self.create_stick_color()

    def create_stick_color(self):
        stick_color = ET.Element("gazebo", reference=f"stick_link_{self.id}")
        material = ET.SubElement(stick_color, "material")
        material.text = "Gazebo/Red"
        return stick_color

    def create_link_element(self):
        link = ET.Element("link", name=f"stick_link_{self.id}")

        # Visual
        visual = ET.SubElement(link, "visual")
        visual_origin = ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        visual_geometry = ET.SubElement(visual, "geometry")
        visual_box = ET.SubElement(visual_geometry, "cylinder", radius="0.005", length=f"{self.length}")

        # Collision
        collision = ET.SubElement(link, "collision")
        collision_origin = ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
        collision_geometry = ET.SubElement(collision, "geometry")
        collision_box = ET.SubElement(collision_geometry, "cylinder", radius="0.005", length=f"{self.length}")

        # Inertial
        inertial = ET.SubElement(link, "inertial")
        inertial_origin = ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        inertial_mass = ET.SubElement(inertial, "mass", value="1.0")
        inertial_inertia = ET.SubElement(inertial, "inertia", ixx="0.01", ixy="0", ixz="0", iyy="0.01", iyz="0", izz="0.01")

        return link

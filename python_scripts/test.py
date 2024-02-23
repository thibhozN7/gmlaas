import rospy
import yaml
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

# Provided text
text = """
---
layout: 
  dim: 
    - 
      label: "rows"
      size: 9
      stride: 81
    - 
      label: "cols"
      size: 9
      stride: 9
  data_offset: 0
data: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 2.2360680103302, 0.0, -1.0, 0.0, 1.4142135381698608, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4142135381698608, 0.0, 1.0, 1.4142135381698608, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4142135381698608, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.8027756214141846, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8027756214141846, 0.0, 0.0, 0.0, -2.2360680103302, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"""

# Parse the YAML text
parsed_data = yaml.safe_load(text)
data = parsed_data['data']
layout_dim = parsed_data['layout']['dim']

# Create the layout dimensions
layout = MultiArrayLayout()
layout.dim = [MultiArrayDimension(label=dim['label'], size=dim['size'], stride=dim['stride']) for dim in layout_dim]
layout.data_offset = parsed_data['layout']['data_offset']

# Create a Float32MultiArray message
float32_array = Float32MultiArray(layout=layout, data=data)

# Display the Float32MultiArray message
print(float32_array)
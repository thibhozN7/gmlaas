import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

current_dir = os.path.realpath(__file__)
package_dir = os.path.dirname(os.path.dirname(current_dir))

# Read CSV file into a pandas DataFrame
df = pd.read_csv(f'{package_dir}/datasets/data/test_trajectory.csv', delimiter=';')

def find_range(df):
    min_x = df['x'].min()
    max_x = df['x'].max()
    diff_x = abs(max_x - min_x)
    mid_x = (max_x + min_x)/2

    min_y = df['y'].min()
    max_y = df['y'].max()
    diff_y = abs(max_y - min_y)
    mid_y = (max_y + min_y)/2

    min_z = df['z'].min()
    max_z = df['z'].max()
    diff_z = abs(max_z - min_z)
    mid_z = (max_z + min_z)/2

    # finding maximum difference between the three
    max_diff = max(diff_x,diff_y,diff_z)
    max_diff = max_diff + 0.1*max_diff
    # setting the range for all three
    range_x = [mid_x-(max_diff/2),mid_x+(max_diff/2)]
    range_y = [mid_y-(max_diff/2),mid_y+(max_diff/2)]
    range_z = [mid_z-(max_diff/2),mid_z+(max_diff/2)]

    return range_x,range_y,range_z

# Plotting 3D trajectory
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.plot(df['x'].values, df['y'].values, df['z'].values, label='Trajectory')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Object Trajectory in 3D Space')

x_range, y_range, z_range = find_range(df)

# Manually set equal aspect ratio for all axes
ax.set_xlim(x_range)
ax.set_ylim(y_range)
ax.set_zlim(z_range)

plt.legend()
plt.show()


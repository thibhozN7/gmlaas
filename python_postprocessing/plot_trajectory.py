import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import argparse
current_dir = os.path.realpath(__file__)
package_dir = os.path.dirname(os.path.dirname(current_dir))

# Parse command line arguments 'param'
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('param', type=str, help='parameter for file names')
args = parser.parse_args()

# Read CSV file into a pandas DataFrame
df1 = pd.read_csv(f'{package_dir}/datasets/data/{args.param}_trajectory_data.csv', delimiter=';')

# Read CSV file into pandas DataFrame
df2 = pd.read_csv(f'{package_dir}/datasets/data/{args.param}_desired_frame_pose.csv', delimiter=';')

def find_range(df1):
    min_x = df1['x'].min()
    max_x = df1['x'].max()
    diff_x = abs(max_x - min_x)
    mid_x = (max_x + min_x)/2

    min_y = df1['y'].min()
    max_y = df1['y'].max()
    diff_y = abs(max_y - min_y)
    mid_y = (max_y + min_y)/2

    min_z = df1['z'].min()
    max_z = df1['z'].max()
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

# Plot the trajectory
ax.plot(df1['x'].values, df1['y'].values, df1['z'].values, label='Trajectory')

# Plot the desired position as a red point
ax.scatter(df2['x'], df2['y'], df2['z'], c='red', marker='o', label='Desired Position')

# Plot the initial position  as a black cross
ax.scatter(df1['x'].iloc[0], df1['y'].iloc[0], df1['z'].iloc[0], c='black', marker='x', label='Initial Position')

# Plot the final position as a blue cross
ax.scatter(df1['x'].iloc[-1], df1['y'].iloc[-1], df1['z'].iloc[-1], c='black', marker='o', label='Final Position')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Object Trajectory in 3D Space')

x_range, y_range, z_range = find_range(df1)

# Manually set equal aspect ratio for all axes
ax.set_xlim(x_range)
ax.set_ylim(y_range)
ax.set_zlim(z_range)
ax.grid(True)

plt.legend()
plt.show()


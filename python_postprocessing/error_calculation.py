import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np

current_dir = os.path.realpath(__file__)
package_dir = os.path.dirname(os.path.dirname(current_dir))

# Read CSV file into a pandas DataFrame
df1 = pd.read_csv(f'{package_dir}/datasets/data/test_trajectory_data.csv', delimiter=';')

# Read CSV file into pandas DataFrame
df2 = pd.read_csv(f'{package_dir}/datasets/snapshots/simu/desired_frame_pose.csv', delimiter=';')

# Extract desired x, y, and z values from df2
desired_x = df2['x'].iloc[0]
desired_y = df2['y'].iloc[0]
desired_z = df2['z'].iloc[0]
desired_roll = df2['roll'].iloc[0]*180/np.pi
desired_pitch = df2['pitch'].iloc[0]*180/np.pi
desired_yaw = df2['yaw'].iloc[0]*180/np.pi

# calculate error
# df1['error_x'] = df1['x'] - desired_x
# df1['error_y'] = df1['y'] - desired_y
# df1['error_z'] = df1['z'] - desired_z
df1['error_x'] = desired_x - df1['x'] 
df1['error_y'] = desired_y - df1['y']
df1['error_z'] = desired_z - df1['z']
# converting roll, pitch, yaw to degrees
df1['roll'] = df1['roll']*180/np.pi
df1['pitch'] = df1['pitch']*180/np.pi
df1['yaw'] = df1['yaw']*180/np.pi

df1['error_roll'] = desired_roll - df1['roll']
df1['error_pitch'] = desired_pitch - df1['pitch']
df1['error_yaw'] = desired_yaw - df1['yaw']

def find_error_range(df1):
    min_x = df1['error_x'].min()
    max_x = df1['error_x'].max()

    min_y = df1['error_y'].min()
    max_y = df1['error_y'].max()
 
    min_z = df1['error_z'].min()
    max_z = df1['error_z'].max()

    min_roll = df1['error_roll'].min()
    max_roll = df1['error_roll'].max()

    min_pitch = df1['error_pitch'].min()
    max_pitch = df1['error_pitch'].max()

    min_yaw = df1['error_yaw'].min()
    max_yaw = df1['error_yaw'].max()

    max_max_values = max(max_x,max_y,max_z)
    min_min_values = min(min_x,min_y,min_z)
    range_x = [min_min_values, max_max_values]
    range_y = [min_min_values, max_max_values]
    range_z = [min_min_values, max_max_values]
    min_angular = min(min_roll,min_pitch,min_yaw)
    max_angular = max(max_roll,max_pitch,max_yaw)
    range_roll = [min_angular, max_angular]
    range_pitch = [min_angular, max_angular]
    range_yaw = [min_angular, max_angular]

    return range_x,range_y,range_z,range_roll,range_pitch,range_yaw

# Create a 1x3 subplot grid
fig, axs = plt.subplots(1, 3, figsize=(15, 5))

x_range, y_range, z_range,range_roll,range_pitch,range_yaw = find_error_range(df1)

time_min = df1['time'].min()
# subtract time_min from all time values to start from 0
df1['time'] = df1['time'] - time_min

# Plot error in x
axs[0].plot(df1['time'].values, df1['error_x'].values, label='Error in X', color='r')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Error in X (-e = s*-s)')
axs[0].set_title('Error in X over Time')
axs[0].set_ylim(x_range)

# Plot error in y
axs[1].plot(df1['time'].values, df1['error_y'].values, label='Error in Y', color='g')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Error in Y (-e = s*-s)')
axs[1].set_title('Error in Y over Time')
axs[1].set_ylim(y_range)

# Plot error in z
axs[2].plot(df1['time'].values, df1['error_z'].values, label='Error in Z', color='b')
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Error in Z (-e = s*-s)')
axs[2].set_title('Error in Z over Time')
axs[2].set_ylim(z_range)

# Adjust layout to prevent clipping of labels
plt.tight_layout()

# Create a 1x3 subplot grid
fig, axs = plt.subplots(1, 3, figsize=(15, 5))

# Plot error in roll
axs[0].plot(df1['time'].values, df1['error_roll'].values, label='Error in Roll', color='r')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Error in Roll (deg)')
axs[0].set_title('Error in Roll over Time')
axs[0].set_ylim(range_roll)

# Plot error in pitch
axs[1].plot(df1['time'].values, df1['error_pitch'].values, label='Error in Pitch', color='g')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Error in Pitch (deg)')
axs[1].set_title('Error in Pitch over Time')
axs[1].set_ylim(range_pitch)

# Plot error in yaw
axs[2].plot(df1['time'].values, df1['error_yaw'].values, label='Error in Yaw', color='b')
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Error in Yaw (deg)')
axs[2].set_title('Error in Yaw over Time')
axs[2].set_ylim(range_yaw)

# Adjust layout to prevent clipping of labels
plt.tight_layout()

# Show the plot
plt.show()


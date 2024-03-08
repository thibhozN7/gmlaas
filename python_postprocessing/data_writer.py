import os
import rosbag
import csv
import tf.transformations as tf
import argparse


# Parse command line arguments 'param'
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('param', type=str, help='parameter for file names')
args = parser.parse_args()

current_dir = os.path.realpath(__file__)
package_dir = os.path.dirname(os.path.dirname(current_dir))


# Open the ROS bag file
bag = rosbag.Bag(f"{package_dir}/datasets/rosbags/{args.param}_bag.bag")

# Open CSV
csvfile1 = open(f"{package_dir}/datasets/data/{args.param}_matching_data.csv", "w")
filewriter1 = csv.writer(csvfile1, delimiter=';')
filewriter1.writerow(['Time', 'Number of visible nodes', 'number of matched nodes', 'Score'])

# Open CSV
csvfile2 = open(f"{package_dir}/datasets/data/{args.param}_H_data.csv", "w")
filewriter2 = csv.writer(csvfile2, delimiter=';')
filewriter2.writerow(['Time', 'T', 'R', 'R_degree'])

# Open CSV
csvfile3 = open(f"{package_dir}/datasets/data/{args.param}_trajectory_data.csv", "w")
filewriter3 = csv.writer(csvfile3, delimiter=';')
filewriter3.writerow(['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])

# Iterate through messages in the bag file
last_time_sec = 0
for topic, msg, t in bag.read_messages(topics=['/data/pre_h_computation','/data/h_computation','/gazebo/model_states']):
    time_sec=t.to_sec()
    if time_sec != last_time_sec:
        if topic == '/data/pre_h_computation':
            # Process data for pre_h_computation topic
            data_topic1 = [time_sec, msg.num_current_points, msg.num_matched_points, msg.score]
            filewriter1.writerow(data_topic1)

        elif topic == "/data/h_computation":
            # Process data for h_computation topic
            data_topic2 = [time_sec, msg.T, msg.R, msg.R_degree]
            filewriter2.writerow(data_topic2)

        elif topic == "/gazebo/model_states":
            # Process data for gazebo/model_states topic
            pose_id = msg.name.index("robot")
            x = msg.pose[pose_id].position.x
            y = msg.pose[pose_id].position.y
            z = msg.pose[pose_id].position.z
            orientation = msg.pose[pose_id].orientation
            roll, pitch, yaw = tf.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            filewriter3.writerow([time_sec, x, y, z, roll, pitch, yaw])
        
    last_time_sec = time_sec
# Close the CSV files and the ROS bag file
print("Data written to CSV files.")
csvfile1.close()
csvfile3.close()
bag.close()

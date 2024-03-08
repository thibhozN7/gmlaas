import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse

current_dir = os.path.realpath(__file__)
package_dir = os.path.dirname(os.path.dirname(current_dir))

# Parse command line arguments 'param'
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('param', type=str, help='parameter for file names')
args = parser.parse_args()

# Read CSV file into a pandas DataFrame
df = pd.read_csv(f'{package_dir}/datasets/data/{args.param}_matching_data.csv', delimiter=';')

# Convert the 'Time' column to datetime format
df['Time'] = pd.to_datetime(df['Time'], unit='s')

# Calculate time differences in nanoseconds and convert to seconds
df['Time_diff'] = (df['Time'] - df['Time'].iloc[0]).dt.total_seconds()

print(df.head())
# Plotting the number of visible nodes and number of matched nodes
plt.figure(figsize=(10, 6))
plt.plot(df['Time_diff'].values, df['Number_of_visible_nodes'].values, label='Number of Visible Nodes')
plt.plot(df['Time_diff'].values, df['number_of_matched_nodes'].values, label='Number of Matched Nodes')
plt.xlabel('Time (seconds)')
plt.ylabel('Count')
plt.title('Number of Visible and Matched Nodes Over Time')
plt.legend()
plt.xlim(0, df['Time_diff'].max())
plt.ylim(0, max(df[['Number_of_visible_nodes', 'number_of_matched_nodes']].values.max()+1, 0))

# Plotting the score of graph matching through time
plt.figure(figsize=(10, 6))
plt.plot(df['Time_diff'].values, df['Score'].values, label='Score')
plt.xlabel('Time (seconds)')
plt.ylabel('Score')
plt.title('Score of Graph Matching Over Time')
plt.legend()
plt.xlim(0, df['Time_diff'].max())
plt.ylim(0, max(df['Score'].max()+1, 0))
plt.show()


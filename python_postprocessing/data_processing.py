import pandas as pd
import matplotlib.pyplot as plt

# Read CSV file into a pandas DataFrame
df = pd.read_csv('matching_data.csv', delimiter=';')

# Convert the 'Time' column to datetime format
df['Time'] = pd.to_datetime(df['Time'], unit='ns')

# Calculate time differences in nanoseconds and convert to seconds
df['Time_diff'] = (df['Time'] - df['Time'].iloc[0]).dt.total_seconds()

# Plotting the number of visible nodes and number of matched nodes
plt.figure(figsize=(10, 6))
plt.plot(df['Time_diff'], df['Number of visible nodes'], label='Number of Visible Nodes')
plt.plot(df['Time_diff'], df['number of matched nodes'], label='Number of Matched Nodes')
plt.xlabel('Time (seconds)')
plt.ylabel('Count')
plt.title('Number of Visible and Matched Nodes Over Time')
plt.legend()
plt.show()

# Plotting the score of graph matching through time
plt.figure(figsize=(10, 6))
plt.plot(df['Time_diff'], df['Score'], label='Score')
plt.xlabel('Time (seconds)')
plt.ylabel('Score')
plt.title('Score of Graph Matching Over Time')
plt.legend()
plt.show()

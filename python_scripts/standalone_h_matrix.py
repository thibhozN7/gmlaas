#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

### FUNCTIONS + EXAMPLE

#####FUNCTIONS
def generate_points(n,bound):
    """Generate a dataset of 3D points."""
    points = np.random.uniform(-bound, bound, (n, 3))
    return points

def apply_transformation(points, R, T):
    """Apply rotation and translation to the points."""
    transform = np.dot(points, R.T) + T
    return transform

def estimate_pose(source_points, target_points): #ESTIMATE HMATRIX
    """Estimates the pose (rotation and translation) that best aligns the source points to the target points."""
    centroid_source = np.mean(source_points, axis=0)
    centroid_target = np.mean(target_points, axis=0)
    centered_source = source_points - centroid_source
    centered_target = target_points - centroid_target
    H = np.dot(centered_source.T, centered_target)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)
    T = centroid_target.T - np.dot(R, centroid_source.T)
    return R, T
def visualize_points(source, target, estimated):
    """Visualize the source, target, and estimated points in a 3D plot."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(source[:, 0], source[:, 1], source[:, 2], c='b', label='Source')
    ax.scatter(target[:, 0], target[:, 1], target[:, 2], c='g', label='Target')
    ax.scatter(estimated[:, 0], estimated[:, 1], estimated[:, 2], c='r', label='Estimated')

    # Plotting the link between source and target points
    for i in range(len(source)):
        ax.plot([source[i, 0], target[i, 0]], [source[i, 1], target[i, 1]], [source[i, 2], target[i, 2]], c='g')
        ax.plot([source[i, 0], estimated[i, 0]], [source[i, 1], estimated[i, 1]], [source[i, 2], estimated[i, 2]], c='r')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    

def visualize_error(target, estimated, R_true, R_est):
    """Visualize the error between the target and estimated points."""
    error = np.linalg.norm(target - estimated, axis=1)
    max_error = np.max(error)
    min_error = np.min(error)
    mae_error = np.mean(error)
    mape_error = np.mean(np.abs(np.divide((target - estimated),target)))*100
    print(f"Max Positional Error: {max_error}")
    print(f"Min Positional Error: {min_error}")
    print(f"MAE Positional Error: {mae_error}")
    print(f"MAPE Positional Error (%): {mape_error}")

    
##### EXAMPLE
#parameters
n_points = 10 #number of points
bound = 100 #bound of the points
theta = np.radians(45) #rotation angle
c, s = np.cos(theta), np.sin(theta)
R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
T = np.array([10, -10, 10]) #translation vector

#generate points
source_points = generate_points(n_points,bound)
print (source_points)
target_points = apply_transformation(source_points, R, T)
noise = bound/100
noise_target_points = apply_transformation(source_points, R, T) + np.random.uniform(-noise,noise,(n_points,3))

# estimate R T and apply transformation 
R_est, T_est = estimate_pose(source_points, target_points)
estimated_points = apply_transformation(source_points, R_est, T_est)
noise_R_est, noise_T_est = estimate_pose(source_points, noise_target_points)
noise_estimated_points = apply_transformation(source_points, noise_R_est, noise_T_est)

#visualize
visualize_points(source_points, target_points, estimated_points)
visualize_error(target_points, estimated_points, R, R_est)
print("------------------------------------- ")
visualize_points(source_points, target_points, noise_estimated_points)
visualize_error(target_points, noise_estimated_points, R, R_est)
plt.show()
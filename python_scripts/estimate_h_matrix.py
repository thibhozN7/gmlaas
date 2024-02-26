import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import Axes3D


class EstimateHMatrix:
    def __init__(self) :
        rospy.init_node("estimate_h_matrix",anonymous=False)
        self.m_points_sub = rospy.Subscriber("/??",Float32MultiArray,self.callback_)
        self.m_targets_points_sub = rospy.Subscriber("/??",Float32MultiArray,self.callback_target)
        self.m_h_matrix_pub = rospy.Publisher("/graph/h_matrix",Float32MultiArray,queue_size=10)


    def applyTransformation(points, R, T):
        """Apply rotation and translation to the points."""
        transform = np.dot(points, R.T) + T
        return transform

    def estimateHMatrix(source_points, target_points):
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

    def visualizePoints(source, target, estimated):
        """Visualize the source, target, and estimated points in a 3D plot."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.scatter(source[:, 0], source[:, 1], source[:, 2], c='r', label='Source')
        ax.scatter(target[:, 0], target[:, 1], target[:, 2], c='g', label='Target')
        ax.scatter(estimated[:, 0], estimated[:, 1], estimated[:, 2], c='b', label='Estimated')

        # Plotting the link between source and target points
        for i in range(len(source)):
            ax.plot([source[i, 0], target[i, 0]], [source[i, 1], target[i, 1]], [source[i, 2], target[i, 2]], c='k')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()


    def visualizeError(target, estimated):
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

    def callback(self,points,target_points):
        R_est, T_est = EstimateHMatrix(points, target_points)
        estimated_points = self.applyTransformation(points, R_est, T_est)
        self.visualizePoints(points, target_points, estimated_points)
        self.visualizeError(target_points, estimated_points)
    
if __name__ == "__main__":
    estimate_h_matrix=EstimateHMatrix()
    rospy.spin()

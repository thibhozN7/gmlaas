# Gazebo Camera Setup

## Required Packages

1. Download and install the RealSense D405 camera package from [rjwb1/realsense-ros](https://github.com/rjwb1/realsense-ros).
2. Replace the `realsense2_description` folder with the one available on [Issaiass' RealSense2 Description repository](https://github.com/Issaiass/realsense-ros).
3. Download the RealSense Gazebo Plugin by Pal Robotics.

## Setup Catkin Workspace

1. Ensure all downloaded packages are in the `src` folder of your Catkin workspace.
2. Run `catkin_make` to compile the workspace.

## Launch the Simulation

Execute the following command to launch the simulation with the RealSense D435 camera model in both RViz and Gazebo:

```bash
roslaunch realsense2_description view_d435_model_rviz_gazebo.launch 
```

## Technical Specification File

Inside the folder `catkin_ws/src/realsense-ros/realsense2_description/urdf/`, you will find the file `_d435.gazebo.xacro`, which contains all the technical specifications of the camera.
Follow these steps to set up and run the simulation with the RealSense D405 camera using the Gazebo and RViz plugins.

### Camera Calibration Instructions

These instructions will guide you through the process of calibrating the camera used in the project using Gazebo and ROS.

#### Prerequisites

- ROS (Robot Operating System) installed on your system.
- Gazebo installed on your system.
- A camera model configured in Gazebo.
- A board with known characteristics (you can use the SDF file provided in the `calibration_gazebo` repository).

#### Calibration Procedure

1. Launch the camera model in Gazebo with gravity disabled using the following command:

    ```bash
    roslaunch realsense2_description view_d435_model_rviz_gazebo.launch
    ```

2. Add the board to the Gazebo world using the following command:

    ```bash
    rosrun gazebo_ros spawn_model -file landmark.sdf -sdf -model calibrator
    ```

    (Optional) Add a lighting source if necessary to ensure good scene visibility.

3. Run the calibration process using the following command:

    ```bash
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/camera/color/image_raw camera:=/camera
    ```

    A calibration screen will open, follow the instructions to move the board in different directions until all parameters on the right turn green.

4. Click on "Calibrate" to save the calibration parameters, and then click on "Save" and "Commit".

5. The collected data will be saved in a YAML file in the `~/.ros/camera_info` directory.


#### Additional Notes

- The size specified as "8x6" in the calibration command (`--size 8x6`) refers to the number of intersection points between the black squares of the chessboard. This parameter does not indicate the number of black squares on the chessboard but the number of intersection points (8 along the width and 6 along the height).

- When specifying the image and camera parameters in the calibration command, it is important to correctly identify the services (not the topics) associated with the camera. To identify the correct services, you can use the `rosservice list` command.

- Ensure to redo the calibration whenever there are changes to the technical characteristics of the camera or if the `camera_info` topic changes. The YAML calibration file generated is essential for the proper functioning of many computer vision or robotics applications. Include it in your project and refer to it as needed.

Découpé en 2 parties :
/Graph building
/Graph matching

//installer requirements
pip install -r requirements.txt


""Example de compilation :
 g++ -o graph_building/output/test -Igraph_building/include/GMLAAS graph_building/src/Node.cpp graph_building/src/Edge.cpp graph_building/src/Graph.cpp graph_building/example/main.cpp
 
 puis :
 ./graph_building//output/test

avec debbuger gdb
 g++ -g -o graph_building/output/test -Igraph_building/include/GMLAAS graph_building/src/Node.cpp graph_building/src/Edge.cpp graph_building/src/Graph.cpp graph_building/example/main.cpp
 puis :
 gdb ./graph_building//output/test
 ""
//camera in gazebo
Download Required Packages:

Download the package rjwb1/realsense-ros to use the RealSense D405 camera.
Replace the realsense2_description folder with the one available on Issaiass' RealSense2 Description repository.
Download the RealSense Gazebo Plugin by Pal Robotics.
Setup Catkin Workspace:

Make sure you have all downloaded packages in the src folder of your Catkin workspace.
Run catkin_make to compile the workspace.
Launch the Simulation:

To start the simulation, run the command roslaunch realsense2_description view_d435_model_rviz_gazebo.launch.
This command will launch the simulation with the RealSense D435 camera model in both RViz and Gazebo.
Technical Specification File:

Inside the folder catkin_ws/src/realsense-ros/realsense2_description/urdf/, you will find the file _d435.gazebo.xacro, which contains all the technical specifications of the camera.
By following these steps, you will be able to set up and run the simulation with the RealSense D405 camera using the Gazebo and RViz plugins. If you encounter any issues or have questions, feel free to ask for assistance from the community or the creators of the packages. Happy simulating!

//camera calibration
Camera Calibration Instructions
These instructions will guide you through the process of calibrating the camera used in the project using Gazebo and ROS.

Prerequisites
ROS (Robot Operating System) installed on your system.
Gazebo installed on your system.
A camera model configured in Gazebo.
A board with known characteristics (you can use the SDF file provided in the calibration_gazebo repository).
Calibration Procedure
Launch the camera model in Gazebo with gravity disabled using the following command:

Copy code
roslaunch realsense2_description view_d435_model_rviz_gazebo.launch
Add the board to the Gazebo world using the following command:

Copy code
rosrun gazebo_ros spawn_model -file landmark.sdf -sdf -model calibrator
(Optional) Add a lighting source if necessary to ensure good scene visibility.

Run the calibration process using the following command:

arduino
Copy code
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.03 image:=/camera/color/image_raw camera:=/camera
A calibration screen will open, follow the instructions to move the board in different directions until all parameters on the right turn green.

Click on "Calibrate" to save the calibration parameters, and then click on "Save" and "Commit".

The collected data will be saved in a YAML file in the ~/.ros/camera_info directory.

Additional Notes
The size specified as "8x6" in the calibration command (--size 8x6) refers to the number of intersection points between the black squares of the chessboard. This parameter does not indicate the number of black squares on the chessboard, but the number of intersection points. In this case, there are 8 points along the width and 6 along the height.

When specifying the image and camera parameters in the calibration command, it is important to correctly identify the services (not the topics) associated with the camera. To identify the correct services, you can use the rosservice list command. The services associated with the image and camera must be correctly specified in the calibration command to ensure that the data is correctly acquired and used during the calibration process.

Ensure to redo the calibration whenever there are changes to the technical characteristics of the camera or if the camera_info topic changes. The YAML calibration file generated is essential for the proper functioning of many computer vision or robotics applications. Ensure to include it in your project and refer to it as needed.

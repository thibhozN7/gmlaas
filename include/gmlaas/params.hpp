#pragma once
#include <vector> 
#include <string>

namespace global{

const std::vector<double> K = {605.132568359375, 0.0, 322.9114074707031,
			              0.0, 604.875244140625, 239.2445526123047,
			              0.0, 0.0, 1.0};

const std::vector<double> K_d405={432.2490539550781, 0.0,
								  431.7025451660156, 0.0,
								  432.2490539550781, 242.901611328125,
								  0.0, 0.0, 1.0};

//the intrinsic parameters vector is given as follow: [fx, cx, fy, cy]
const std::vector<double> rs_intrinsic_params{K[0], K[2], K[4], K[5]};
const std::vector<double> rs_intrinsic_params_d405 = {K_d405[0], K_d405[2], K_d405[4], K_d405[5]};

const double rs_depth_scale = 0.001; 
const double rs_depth_scale_d405 = 9.999999747378752e-05;
const double h_fov = 87;//degree
const double v_fov = 58;//degree
const double baseline = 0.018;// meters

const double max_distance_threshold = 0.65;//meters 

const std::string data_init_pos_file_path = 
	"/home/fgebrayel/r2t2-deficle/catkin_ws/src/r2t2-deficle/visual_servoing_realsense_visp/python_scripts/data_plots/init_position_sample_file.txt";

const std::string data_pos_file_path = 
	"/home/fgebrayel/r2t2-deficle/catkin_ws/src/r2t2-deficle/visual_servoing_realsense_visp/python_scripts/data_plots/position_sample_file.txt";

const std::string data_vel_file_path =
	"/home/fgebrayel/r2t2-deficle/catkin_ws/src/r2t2-deficle/visual_servoing_realsense_visp/python_scripts/data_plots/vel_sample_file.txt";

const std::string data_vel_command_file_path =
	"/home/fgebrayel/r2t2-deficle/catkin_ws/src/r2t2-deficle/visual_servoing_realsense_visp/python_scripts/data_plots/vel_command_file.txt";

}
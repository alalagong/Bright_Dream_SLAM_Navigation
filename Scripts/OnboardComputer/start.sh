source ~/net_setup.sh

roslaunch trr_global_planner map_generator_bag.launch & sleep 5;

roslaunch trr_global_planner global_planner_bag.launch & sleep 5;

rosrun vins vins_node catkin_ws/src/VINS-Fusion-master/config/realsense_d435i/realsense_stereo_imu_config.yaml & sleep 5;

rosrun loop_fusion loop_fusion_node catkin_ws/src/VINS-Fusion-master/config/realsense_d435i/realsense_stereo_imu_config.yaml & sleep 3;


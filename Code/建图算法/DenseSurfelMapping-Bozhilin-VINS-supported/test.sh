roslaunch surfel_fusion map_rviz.launch & sleep 5;
rosrun vins vins_node '/home/dji/catkin_ws/src/VINS-Fusion/config/realsense_d435i/dji_stereo_imu_config.yaml'  & sleep 5;
rosrun loop_fusion loop_fusion_node '/home/dji/catkin_ws/src/VINS-Fusion/config/realsense_d435i/dji_stereo_imu_config.yaml' & sleep 5;
roslaunch surfel_fusion fuse_realsense.launch & sleep 5;
wait

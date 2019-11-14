source ~/net_setup_gs.sh
roslaunch trr_global_planner odom_rviz.launch & sleep 5;
roslaunch surfel_fusion fuse_realsense.launch & sleep 5;

wait

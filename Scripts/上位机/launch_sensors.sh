source ~/net_setup.sh
roslaunch dji_sdk sdk.launch & sleep 13;
rosservice call /dji_sdk/set_hardsyc "frequency: 100  
tag: 0" & sleep 3;
roslaunch realsense2_camera rs_camera.launch & sleep 5;
rosrun dynamic_reconfigure dynparam set /camera/Stereo_Module 'Emitter_Enabled' false & sleep 3;
roslaunch wheel_driver driver.launch;

wait

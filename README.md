1、I use velodyne (16), imu9250,
2、velodyne(frame_id is velodyne,topic,velodyne_points ),imu(frame_id is imu_link,topic,imu)
3、Note that the direction of the IMU and the laser can be adjusted to the URDF file
4、In addition, IMU and velodyne should be compiled separately in other workspaces (not using catkin_make_isolated)
************************************
I run roslaunch 
1、roslaunch velodyne_pointcloud vlp16.launch
2、roslaunch imu  imu.launch
3、roslaunch cartographer_ros demo_vlp16_imu3d.launch 



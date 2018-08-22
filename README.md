## 
**The code in this repository amis at mapping successful with cartographer.**
The sensors used in this case are IMU and LIDAR.
## Sensors
- ### IMU
  The IMU device used in this project is HI216. You can get some information and technique documents in [HERE](http://www.hipnuc.com/).

  You can find the driver of IMU in the imu_interface_broadcaster folder.All you need is compilling it separately in a **new** workspaces.

- ### LIDAR
  The LIDAR device used in this project is velodyne 16 lines.
  
  The driver of velodyne is [velodyne_driver](http://wiki.ros.org/velodyne_driver). It has a wiki page where you can know it better.

## Others

- ### URDF
  One thing must be noticed. The relationship(tf) of the IMU and the LIDAR can be adjusted to the URDF file, and it must fit the real world.
  
- ### Launch 
  After all node compiled, you can run cartographer with commands below
  
  `roslaunch imu_interface_broadcaster imu.launch`
  
  `roslaunch velodyne_pointcloud VLP16_points.launch`
  
  `roslaunch cartographer_ros demo_velodyne2d.launch`
  

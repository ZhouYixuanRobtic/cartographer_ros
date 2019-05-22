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

- ### Demos

  Only 2D demo succeed, 3D demo needs to be tuned, pure_localization

  - #### 2D demo 

    After all sensors connected, use command below

    ```
    1. roslaunch imu_intercace_broadcaster imu.launch
    2. roslaunch velodyne_pointcloud VLP16_points.launch
    3. roslaunch cartographer_ros demo_velodyne2d.launch
    ```

    **Must be  noticed **

    The realtive rotation of LIDAR and IMU must be consistent with the URDF file. Also, the URDF file can be modifed to fit the real world. In this project, coordinate system of world frame and base_link frame are both front Y right X, while that of IMU is front X left Y and that of LIDAR is front X  left Y. In other words,yaw is $ \frac{\pi}{2}=1.5708 \ \ rad$.

    In personal experience, the RPY got after the imu.launch launched can be used as the values of the URDF files because of the consistency of the coordinate system of base_link and word frame.

  - #### Pure_localization demo

    Use command below

    ```c++
    roslaunch cartographer_ros demo_velodyne2d_localization.launch \ load_state_filename:=${HOME}/catkin_ws/2018-07-11-15-37-39.bag.pbstream \ bag_filename:=${HOME}/catkin_ws/2018-07-11-21-23-42.bag
    ```

    The pbstream file was generated by the command below

    ```
    roslaunch cartographer_ros offline_velodyne_2d.launch bag_filenams:=${HOME}/catkin_ws/2018-07-11-15-37-39.bag
    ```

    The initial pose can be appointed manual by command below

    You can refer to [This page](https://github.com/googlecartographer/cartographer_ros/issues/579#issuecomment-351482589)

    ```
    1.rosservice call /finish_trajectory "trajectory_id: 1"
    2.rosrun cartographer_ros cartographer_start_trajectory -configuration_directory=/home/xcy/catkin_ws/install_isolated/share/cartographer_ros/configuration_files/ -initial_pose="{to_trajectory_id=0,relative_pose={translation={1.0,-0.2,0.},rotation={0.,0.,0.}}}"
    ```

  - 3D demo

    Use command below after all sensors connected

    ```
    1. roslaunch imu_intercace_broadcaster imu.launch
    2. roslaunch velodyne_pointcloud VLP16_points.launch
    3. roslaunch cartographer_ros demo_velodyne.launch
    ```

    **Warning**

    3D demo isn't tuned well. I guess may the complex envrionment causes the bad map, please try 3D demo in corridor.

  - **Save map**

    Use command below when you wanna save the map

    ```
    1. rosservice call /finish_trajectory "trajectory_id: 1"
    2. rosrun map_server map_saver -f map001
    ```

    **Notice**

    The map looks well in 2d mode while not in 3d mode.

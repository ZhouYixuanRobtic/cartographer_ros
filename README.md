## 
**The code in this repository amis at mapping successful with cartographer.**
The sensors used in this case are IMU and LIDAR.
## Sensors
- ### IMU
  The IMU device used in this project is HI216. You can get some information and technique documents in [HERE](http://www.hipnuc.com/).

  You can find the driver of IMU in the `src/imu_interface_broadcaster` folder.

- ### LIDAR
  The LIDAR device used in this project is velodyne 16 lines.

  The driver of velodyne is [velodyne_driver](http://wiki.ros.org/velodyne_driver). It has a wiki page where you can know it better.
  
  **Warning:  the newest Velodyne driver may cause A cartographer  warning, so try the driver in this repository**

## Others

- ### URDF
  One thing must be noticed. The relationship(tf) of the IMU and the LIDAR can be adjusted to the URDF file, and it must fit the real world.

- ### Launch 
  After all node compiled, you can run cartographer with commands below to generate your own map

  `roslaunch cartographer_ros demo_velodyne2d.launch`

- ### Demos

   Those demo are tested in a large wild environment.

  - #### 2D demo 

    After all sensors connected, use command below(**Do not forget your own filename**)

    ```c++
     roslaunch cartographer_ros demo_velodyne2d.launch
     //after finish your mapping
     rosservice call /finish_trajectory 0
     rosservice all /write_state "filename: '' include_unfinished_submaps: 'false' "
```
    
- #### Pure_localization demo
  
  Use command below, remember to use your own bag 
  
  ```c++
    roslaunch cartographer_ros demo_velodyne2d_localization.launch \ load_state_filename:= \ bag_filename:=
  ```
  
  The initial pose can be appointed manual by command below
  
    You can refer to [This page](https://github.com/googlecartographer/cartographer_ros/issues/579#issuecomment-351482589)
    
  ```
  1.rosservice call /finish_trajectory "trajectory_id: 1"
  2.rosrun cartographer_ros cartographer_start_trajectory -configuration_directory=/home/xcy/catkin_ws/install_isolated/share/cartographer_ros/configuration_files/ -initial_pose="{to_trajectory_id=0,relative_pose={translation={1.0,-0.2,0.},rotation={0.,0.,0.}}}"
  ```
    
  - 3D demo

    Use command below after all sensors connected

    ```c++
  roslaunch cartographer_ros demo_vlp16_imu3d.launch
    //after finish your mapping
    rosservice call /finish_trajectory 0
    rosservice all /write_state "filename: '' include_unfinished_submaps: 'false' "
  ```
  
  - **Save map**

    Use command below when you wanna save the map(pgm map)

    ```
    1. rosservice call /finish_trajectory "trajectory_id: 1"
    2. rosrun map_server map_saver -f map001
    ```
  

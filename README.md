# pseudo_lidar_sim

This is virtual lidar package for me





## Install&Use

### Dependency
pip3 install py-getch

### Run
roslaunch pl_visualizer map_generator.launch


## IO

### Subscribe

```
Nav_msgs/Odometry 
    * "/odom"
    * odom of robot

Geometry_msgs/PoseStamped
    * "/move_base_simple/goal"
    * pose of obstacle (you can publish this topic from rviz)
    
Keyboard input
    * Moving obstacle
    * w, a, s, d = Moving verticle, horizon    
    * q, e, z, c = Moving diagonal
    * x = terminate
    
```



### Publish

```
Nav_msgs/Odometry
    * "/robot_odom"
    * odom of robot

Nav_msgs/Odometry
    * "/obstacle_odom"
    * odom of obstacle

Visualization_msgs/Marker
    * "/visualization_marker"
    * rviz marker for obstacle

Sensor_msgs/PointCloud2
    * "/pseudo_scan"
    * virtual lidar scan data

Nav_msgs/OccupancyGrid
    * "/pseudo_scan"
    * 2d Occupancy map
```

**All of above topics are published as "map" frame**



---


### Selectable LiDAR Spec
- Range                 (default 100.0 m)
- Horizon resolution    (default 0.5 deg)
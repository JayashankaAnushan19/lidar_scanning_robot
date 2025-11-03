# Lidar Scanning Robot _ Project Log

## Project Overview
This project focuses on simulating a LiDAR-equipped robot within the Gazebo environment. The goal is to replicate real-world scenarios in simulation, allowing for testing and development in a controlled virtual space.

The robot will support teleoperation via keyboard control, enabling manual navigation and experimentation with sensor data.

## 1. Robot View
Robot is basically two wheeled with caster wheel. Used diff_drive_controller for simulate within the Gazebo. Added a lidar top of the robot and able to scan the area.

Able to see the robot using :
```
roslaunch lidar_scanning_robot view_robot_model.launch 
```

## 2. Mapping
For the localization and mapping i used Hector SLAM and GMapping. Then saved the map withing the root folder ```lidar_scanning_robot/map``` as ```my_map```

This will use to path planning and driver to target.

## 3. Path planning
This stage is to make a path planning and give the target base approach to the robot. For this, wanted to build a 

## 4. Drive to exact target




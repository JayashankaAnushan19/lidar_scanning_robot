rosrun tf view_frames


rosnode info /gazebo



rosrun tf tf_monitor



Save map
-----------------------
rosrun map_server map_saver -f ~/ros/catkin_ws/src/lidar_scanning_robot/maps/my_map


View map
-----------------------
eog ~/ros/catkin_ws/src/lidar_scanning_robot/maps/my_map.pgm


Inspect map
-----------------------
cat ~/ros/catkin_ws/src/lidar_scanning_robot/maps/my_map.yaml


Add these to reuirements.txt
----------------------------
sudo apt install ros-noetic-move-base ros-noetic-amcl ros-noetic-global-planner ros-noetic-dwa-local-planner ros-noetic-navfn
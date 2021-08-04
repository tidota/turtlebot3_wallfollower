# Turtlebot3 Wallfollower

This repository provides a simple wallfollowing program.
A turtlebot3 follows the wall in the right by using the scan data.

Assuming the turtlebot3 has been setup, connect to the robot by SSH and launch
the nodes.
```
ros2 launch turtlebot3_bringup robot.launch.py
```

Then, at the local PC,
```
export ROS_DOMAIN_ID=30
ros2 run turtlebot3_wallfollower wallfollower
```

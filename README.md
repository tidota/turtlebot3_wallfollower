# Turtlebot3 Wallfollower

This repository is my personal project to practice to run turtlebot3 with **ROS1 Noetic**.
The program makes a turtlebot3 to follow the wall on the right by using the ranging data.

It should work on both the actual hardware and the simulated one.

NOTE: It was originally developed for ROS2, but I switched to ROS1 since I realized that
Turtlebot3 is well developed for ROS1 and more compatible. (at least as of the time of writing)
To view the original ROS2 version, checkout the commit tagged `ros2-exp`
```
git chckout ros2-exp
```


## Actual hardware

Assuming the turtlebot3 has been setup, connect to the robot by SSH and launch
the nodes.
```
roslaunch turtlebot3_bringup robot.launch
```

Then, at the local PC,
```
export ROS_DOMAIN_ID=30
rosrun turtlebot3_wallfollower wallfollower
```

To start,
```
rosservice call /set_running std_srvs/srv/SetBool 'data: true'
```

To stop,
```
rosservice call /set_running std_srvs/srv/SetBool 'data: false'
```

## Simulation

![](./img/sim1.jpg)

![](./img/sim2.jpg)

Start the simulation.
```
roslaunch turtlebot3_wallfollower start_sim.launch
```

To run the robot, call the `set_running` service.
```
rosservice call /set_running std_srvs/srv/SetBool 'data: true'
```

To stop it,
```
rosservice call /set_running std_srvs/srv/SetBool 'data: false'
```


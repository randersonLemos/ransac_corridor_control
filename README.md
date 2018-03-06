# Ransac Corridor Control #
ROS package `ransac_corridor_control` repository. This package aims to solve the problem of intra-row autonomous navigation with an unisensory approach. The unique sensor device considered is a laser LIDAR. The package can also be used in any navigation problem well-characterized by a corridor with some sort of stochastic walls. The problem of leaving one corridor and entering in the parallel one is not covered here.

* `ransac_corridor_control` package in action!

![Alt Text](https://i.imgur.com/vXl15sO.gif)

## Installation
Just clone the repository into a ROS workspace and compile it with `catkin_make`.
* There are no dependencies others than the ones commonly required by ROS

### System Requirements ###
Some requirements are:
* GCC version 5.4.1 or later
* CMake version 2.8.3 or later
* Git
* python 2.7

## Algorithms
The main algorithms used by the `ransac_corridor_control` are:
* The method Random Sample Consensus (RANSAC);
* An Extended Kalman Filter implementation;
* A PID control.

## Usage
The Ros package `ransac_corridor_control` can easily be initialized by the command:

```
roslaunch ransac_corridor_control full.launch
```

The command above will launch three nodes, which are called:
* bisector;
* filter;
* control;
* to_twist.

### bisector node
This node subscribes to the topics:
* `\scan` of message type **sensor_msgs/LaserScan**.

It is from the topic `\scan` that the data from the corridor environment provided by a laser scan sensor are made available for the method RANSAC, which basically estimates support lines of the corridor's wall and a bisector line.

This node publishes to the topics:

* `\bisector_coeffs` of messsage type **ransac_corridor_control/LineCoeffs3Stamped** (custom message);
* `\bisector_pcl` of message type **sensor_msgs/PointCloud2**;
* `\points_ransac` of message type **sensor_msgs/PointCloud2**.

It is from the topic `\bisector_coeffs` that the coefficients of the bisector line are made available to the ROS environment. The others two topic are just to data visualization. The topic `bisector_plc` contains generated points of the support and bisector lines and the topic `points_ransac` contains the points used by the RANSAC in the estimation process.

### filter node
This node subscribes to the topics:

* `\bisector_coeffs` of messsage type **ransac_corridor_control/LineCoeffs3Stamped** (custom message)
* `\car_command` of message type **ransac_corridor_control/CarCommandStamped** (custom message).

With the coefficients of the last computed bisector line, of the last command sent to the car and with a simplified kinematic vehicle model, this node uses the EKF to estimate a bisector line less noise than the one computed by the node bisector.

This node publishes to the topics:

* `\filtered_bisector_coeffs` of messsage type **ransac_corridor_control/LineCoeffs3Stamped** (custom message)
* `\filtered_bisector_pcl` of message type **sensor_msgs/PointCloud2**

It is from the topic `/filtered_bisector_coeffs` that the filtered coefficients of the bisector line are made available to the ros enviroment. The topic `\filtered_bisector_coeffs` is just for data visualization.

### control node
This node subscribes to the topics:
* `\filtered_bisector_coeffs` of messsage type **ransac_corridor_control/LineCoeffs3Stamped** (custom message)

With the bisector line coefficients from the topic `\filtered_bisector_coeffs` the control node computes an steering angle that orients the vehicle in the direction of reducing its distance from the filtered bisector line (lateral error).

This node publishes to the topics:
* `\car_command` of message type **ransac_corridor_control/CarCommandStamped** (custom message).

The control command to the low-level platform control implementations are sent by the topic `\car_command`. The message publised by the topic `\car_command` contains three fields which are:
* speedLeft (left wheel speed)
* speedRight (right wheel speed)
* steerAngle (steering angle)

### to_twist node
This node publishes to the topics:
* `\car_command` of message type **ransac_corridor_control/CarCommandStamped** (custom message).

This node publishes to the topics:
* `\cmd_vel` of message type **geometry_msgs/Twist**

This node just convert from the message **ransac_corridor_control/CarCommandStamped** to the message **geometry_msgs/Twist**.

### Parameters
All setting parameters are available to customization in the file `yaml/config.yaml`. Some parameters are:

* `l`: float parameter of the distance between car axle;
* `laser/verbose`: bool parameter for debugging purposes;
* `laser/window_width`: float parameter of the width of the window of interest;
* `laser/window_length`: float parameter of the length of the window of interest;
* `laser/threshold`: float parameter of the formation of the consensus group threshold;
* `laser/iterations`: int parameter of the number of iterations realized by the method RANSAC
* `control/kpt`: float parameter of the control proportional gain;
* `control/kit`: float parameter of the control integral gain;
* `control/kvt`: float parameter of the control velocity gain (like derivative);
* `control/krt`: float parameter of the control rescaling gain;
* `control/lin_vel`: float parameter of the target linear velocity;
* `control/wait_time`: float parameter of the waiting time to start to send the target velocity;
* `kalman/verbose`: bool parameter for debugging purposes;
* `kalman/q00`: float parameter of the element associated with the angle between the bisector line and the vehicle of the process covariance matrix Q;
* `kalman/q11`: float parameter of the element associated with the linear coefficient of the bisector line of the process covariance matrix Q;
* `kalman/q22`: float parameter of the element associated with the vehicle linear acceleration of the process covariance matrix Q;
* `kalman/q33`: float parameter of the element associated with the vehicle angular acceleration of the process covariance matrix Q;
* `kalman/r00`: float parameter of the elements associated with the angle between the bisector line and the vehicle of the measurement covariance matrix R;
* `kalman/r11`: float parameter of the elements associated with the linear coefficient of the bisector line and the vehicle of the measurement covariance matrix R;
* `kalman/r22`: float parameter of the elements associated with the vehicle linear velocity of the measurement covariance matrix R;
* `kalman/r33`: float parameter of the elements associated with the vehicle angular velocity of the measurement covariance matrix R.

## Licensing
This software is being made available for research purposes only.  See
the [LICENSE](LICENSE.txt) file in this directory for conditions of use.


## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/randersonLemos/ransac_corridor_control/issues).

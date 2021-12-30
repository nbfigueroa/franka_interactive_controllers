# Learning DS Motion Policies from Demonstrations

Fill in from ds-ltl and ds-opt packages.


## Using a Learned DS as a Motion Policy for Robot Control
Once you have verified that the DSs were learned correctly (and exhibit the desired behavior) we can use them as a motion policy to the control the end-effector of a real robot. There are several ways to accomplish this. These are listed below:
- [lpvDS-lib](https://github.com/nbfigueroa/lpvDS-lib): This repository contains a standalone C++ library that reads the parameters of the learned DS, in either yaml/txt format and can compile the library either with ROS-ified catkin_make or pure cmake compilation commands. This is the preferred library if you are NOT using ROS to control your robot and use/like C++. 
- [ds_motion_generator](https://github.com/nbfigueroa/ds_motion_generator) **[Preferred]**: This repository is a ROS package that includes nodes for DS motion generation of different types (linear DS, lpv-ds, lags-ds, etc,). It depends on the standalone [lpvDS-lib](https://github.com/nbfigueroa/lpvDS-lib) library and uses yaml files to load the parameters to ros node. This node generates the desired velocity from the learned DS (given the state of the robot end-effector) and also filters and truncates the desired velocity (if needed). The node also generates a trajectory roll-out (forward integration of the DS) to visualize the path the robot will follow -- this is updayed in realtime at each time-step. 
- [ds-opt-py](https://github.com/nbfigueroa/ds-opt-py): This is an experimental python package that includes a python library to read parameters in yaml format for the execution of the lpv-DS, ROS integration is yet to be done, but should be straightforward. However, filtering and smoothing should be done separately. 

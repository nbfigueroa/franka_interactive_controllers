# franka_interactive_controllers

Control interface built on top of [franka_ros](https://frankaemika.github.io/docs/franka_ros.html) that allows to control the franka robot arm in several joint and Cartesian space impedance control schemes for interactive, safe and reactive (mostly DS-based) motion planning and learning. This low-level contol interface is used and developed by/for [Prof. Nadia Figueroa](https://github.com/nbfigueroa) and her collaborators and students. Initially developed at MIT in the [Interactive Robotics Group](https://interactive.mit.edu/).

---
## Installation
This package depends on several other code repositories that are robot-agnostic. To install this repo and all its dependencies do the following steps:
* In your catkin src directory clone the repository
```
$ git clone https://github.com/nbfigueroa/franka_interactive_controllers.git
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$  wstool init
$  wstool merge franka_interactive_controllers/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro <your-ros-distro> 
```

* Finally complie
```bash
  $ cd ~/catkin_ws
  $ catkin_make
  $ source devel/setup.bash
  $ catkin_make
  $ rospack profile
```
 You might need to source the `./bashrc` file and compile again if the first compliation could not find some of the in-house dependencies. If `roscd` doesn't find the compiled packages run `rospack profile`.


## Usage
To bring up the standalone robot with no specific controllers but all the franka_ros functionalities + gripper GUI contoller + geometry messages of EE pose:
```bash
roslaunch franka_interactive_controllers franka_bringup.launch 
```

Repo includes a ros-nodified version of  [franka_gripper_run](https://github.com/nbfigueroa/franka_gripper_run) that uses the actionlib server from franka_ros/franka_gripper. A simple action client node that open/closes the gripper can be used by running the following:
```bash
rosrun franka_interactive_controllers franka_gripper_run_node <command_type>
```
Where ``<command_type>``= 1 (close) and 0 (open).

To run the GUI like [franka_gripper_run](https://github.com/nbfigueroa/franka_gripper_run) run the following script:
```bash
rosrun franka_interactive_controllers franka_gui_gripper_run.py
```

---
## Contact
Maintainer: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig @ seas dot upenn edu)  
Code Contributions by: [Bilkit Githinji](https://interactive.mit.edu/about/people/bilkit), [Shen Li](https://shenlirobot.github.io/).

## Licenses
Please note that the code for some of the controllers in this repository is derived from [franka_ros](https://github.com/frankaemika/franka_ros/), specifically the [franka_example_controllers](https://github.com/frankaemika/franka_ros/tree/develop/franka_example_controllers) package which is licenced under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0.html). The remaining code in the repository is licensed under an MIT license (see LICENSE for details).

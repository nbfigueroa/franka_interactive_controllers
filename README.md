# franka_interactive_controllers

Control interface built on top of [franka_ros](https://frankaemika.github.io/docs/franka_ros.html) that allows to control the franka robot arm in several joint and Cartesian space impedance control schemes for interactive, safe and reactive (mostly DS-based) motion planning and learning. This low-level control interface is used and developed by/for [Prof. Nadia Figueroa](https://github.com/nbfigueroa) and her collaborators and students. Initially developed at MIT in the [Interactive Robotics Group](https://interactive.mit.edu/).

---
## Installation
Besides [franka_ros](https://frankaemika.github.io/docs/franka_ros.html) and by consequence [libfranka](https://github.com/frankaemika/libfranka) this package depends on several other code repositories that are robot-agnostic and useful for control and learning of motion policies from demonstrations. 

To install this repo and all its dependencies do the following steps:
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
 
---
## Instructions

We include a list of instructions for how to start using the franka panda arm with this controller package:
- [Startup the robot](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/instructions/robot_startup.md)
- [External tool compensation](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/instructions/external_tool_compensation.md)
- [Basic robot operating functionalities](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/instructions/basic_robot_control.md) (open/close gripper and move robot to a desired joint position bypassing franka_ros -- useful to startup an experiment).
- [Kinesthetic Teaching/Recording](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/instructions/kinesthetic_teaching_recording.md)
- Learning DS motion policies
- Executing DS motion policies

## Usage
Following we detail instructions on how to bringup the controllers and their functionalities:   

### Main Robot Launch
To bring up the standalone robot with [franka_ros](https://frankaemika.github.io/docs/franka_ros.html) without any specific controller (useful for testing -- can be included in your custom launch file):
```bash
roslaunch franka_interactive_controllers franka_interactive_bringup.launch 
```
This will load all franka_ros (franka_control, franka_gripper, etc.) functionalities + gripper GUI controller + configured rviz settings.
<p align="center">
  <img src="doc/img/franka_interactive_bringup.png" width="700x"> 
</p>

### Robot Controllers
#### Joint Gravity Compensation
To load the [joint gravity compensation controller](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/src/franka_joint_controllers/joint_gravity_compensation_controller.cpp) launch the following:
```bash
roslaunch franka_interactive_controllers joint_gravity_compensation_controller.launch
```
  - This will load a joint gravity compensation torque controller. It compensates (from code) the weight of any additional tool (i.e., a tool grasped by the hand or cameras mounted on the gripper). As well as the capability to lock certain joints for ease of demonstration. 
  - The external forces imposed on the end-effector with the additional weight should be defined in ``./config/external_tool_compensation.yaml``. Instructions on how to finding values for your custom tool [here](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/instructions/external_tool_compensation.md). You can toggle to **activate/deactivate** this compensation online using dynamic reconfigure. Default is set to ``true``.
  - The desired joints to lock and the locked positions can be modified online by dynamic reconfigure. Default is set to ``false`` for all locks.
  - To launch ``franka_interactive_bringup.launch`` within this same launch file ``set load_franka_control:=true``. Default is set to ``false``.
  <p align="center">
      <img src="doc/img/franka_joint_gravity_compensation.png" width="700x"> 
  </p>

**NOTE: If you run this script and the robot moves by itself, that means that your external_tool_compensation forces are incorrect. See external_tool_compensation instructions to correct it.** 

#### Kinesthetic Teaching/Recording Pipeline
You can launch the [joint gravity compensation controller](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/src/franka_joint_controllers/joint_gravity_compensation_controller.cpp) together with data recording nodes from my [easy-kinesthetic-teaching](https://github.com/nbfigueroa/easy-kinesthetic-recording) repository ``latest-franka`` branch. 

```bash
roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```

<p align="center">
    <img src="doc/img/franka_kinesthetic_teaching.png" width="800x"> 
</p> 


This will run scripts and nodes that will allow you to:
- Select topics to record and define path to save rosbags.
- Run all functionalities from [joint gravity compensation controller](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/src/franka_joint_controllers/joint_gravity_compensation_controller.cpp).
- Bringup rosservice to record/stop a recording.
- Visualize end-effector trajectories being recorded in real-time and gripper state (open/closed).
- Replay recorded end-effector trajectoriers and gripper state in RViz, franka controllers and launch files must be turned off for this functionality.

See [Kinesthetic Teaching/Recording](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/instructions/kinesthetic_teaching_recording.md) instructions for definitions and usage.


#### Cartesian Impedance Controller with Pose Command
To load a cartesian impedance controller with pose command (a PD control law with position error tracking; i.e., **stiffness control** and velocity damping) launch the following:
```bash
roslaunch franka_interactive_controllers cartesian_pose_impedance_controller.launch
```
This launch file will load a [cartesian impedance controller](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/src/franka_cartesian_controllers/cartesian_pose_impedance_controller.cpp) that:
- Takes as input a desired end-effector pose (position and orientation) as a ``geometry_msg::PoseStamped`` with topic name ``/cartesian_impedance_controller_pose_command/desired_pose``.
- Will compensate for external forces imposed by additional tools/accesories mounted on the gripper (as described in joint gravity compensation controller above).
- Control for a desired nullspace configuration, defined in  [config/impedance_control_additional_params.yaml](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/config/impedance_control_additional_params.yaml)

#### Cartesian Impedance Controller with Twist Command
```bash
roslaunch franka_interactive_controllers cartesian_twist_impedance_controller.launch
```

#### Joint Impedance Control with Position Command
PD control law with position error tracking (stiffness control) and velocity damping.
*To fill...*
  
#### Joint Impedance Control with Velocity Command  
*To fill...*


---
## Contact
Maintainer: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig @ seas dot upenn edu)  
Code Contributions by: [Bilkit Githinji](https://interactive.mit.edu/about/people/bilkit), [Shen Li](https://shenlirobot.github.io/).

## Licenses
Please note that the code for some of the controllers in this repository is derived from [franka_ros](https://github.com/frankaemika/franka_ros/), specifically the [franka_example_controllers](https://github.com/frankaemika/franka_ros/tree/develop/franka_example_controllers) package which is licenced under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0.html). The remaining code in the repository is licensed under an MIT license (see LICENSE for details).

# Kinesthetic Teaching/Recording

These instructions assume you have installed the [easy-kinesthetic-recording](https://github.com/nbfigueroa/easy-kinesthetic-recording) package and all of its dependencies. 
- See [README](https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/README.md) file for installation instructions.  
- Make sure to have the ``latest-franka`` branch checked out. 
- These instructions are a summary/copy of what can be found in the [README](https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/README.md) of the [easy-kinesthetic-recording](https://github.com/nbfigueroa/easy-kinesthetic-recording) package.

## Definitions
Prior to trying this for yourself, you should define the topics that you wish to record and the path/name for the rosbags.

In the launch file ```launch/franka_kinesthetic_teaching.launch``` you can define the topics that you wish to record in the following argument.
```xml  
<arg name="topics_rosbag"       
    default="/tf 
    /franka_state_controller/joint_states 
    /franka_state_controller/F_ext 
    /franka_state_controller/O_T_EE 
    /franka_state_controller/O_T_FL 
    /franka_gripper/joint_states"/>
```
To define the path to the directory where all bags will be recorded and the bag prefix-:
```xml
<arg name="path_save_rosbag"           default="/home/panda2/rosbag_recordings/cooking/"/>  
```

## Visualization Settings
Within the [easy-kinesthetic-recording](https://github.com/nbfigueroa/easy-kinesthetic-recording) package you will find the launch file ```launch/franka_record_demonstrations.launch``` where you could also define the above definitions and set the following trajectory visualization commands:
```xml  
<arg name="viz_traj"  default="true"/> 
```
When set to true, you can visualize the trajectories in real-time and being replayed with the franka.

The following setting: 
```xml  
<arg name="viz_obj" default="true"/>
```
Enables the visualization of the state of the gripper, indicated by the block in between the fingers:
- green: an object is grasped
- gray: no object is grasped

These settings are the same for replaying a rosbag, as will be shown next. 

---
## Step 1: Recording Kinesthetic Demonstrations as ROSBags
### Bringup Kinesthetic Teaching Pipeline
If you have the main robot launch file already running in a terminal; i.e.,
```bash
roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```

Then all you need to do is run the kinesthetic teaching pipeline as follows:
```bash
roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```

Alternatively, you can run everything in **one terminal** by setting the following parameters:
```bash
roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch load_franka_control:=true
```
If everything was installed and compiled correctly, you should see the following windows (without trajectories if they haven't been recorded yet):
<p align="center">
<img src="https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/img/franka_kinesthetic_teaching.png" width="800x">
</p>

**NOTE: If you run this script and the robot moves by itself, that means that your external_tool_compensation forces are incorrect. See external_tool_compensation instructions to correct it.**

### Record Demonstrations as ROSbags
To record/stop a rosbag recording you can either do it by: 
- Pressing the buttons on the GUI as shown in the image above
- Type the following in a terminal
```bash
 rosservice call /record/cmd "cmd: 'record/stop'"
 ```

### Replaying a recorded demonstration
```bash
roslaunch easy_kinesthetic_recording franka_replay_bag_demonstrations.launch
```
The same visualization definition can be set for trajectory and gripper states. 

##### Play bag
```bash
$ rosbag play *.bag
```

**Note:** When repalying a bag, the gripper will not be shown and you might see some erros in rviz, but that's fine.

See examples below.

### Examples

This code together with [franka_interactive_controllers](https://github.com/nbfigueroa/franka_interactive_controllers) has been used for two household tasks:
- **cooking preparation task**: scooping and mixing ingredients from bowls

<p align="center">
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_task_reduced.gif" width="425x">
<!-- 	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_recording.gif" width="400x">  -->
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_rosbag_replay.gif" width="450x">
</p>
<p align="center">
	Left: Video of kinesthetic demonstration, Right: Visualization of recorded trajectories by replaying recorded rosbag
</p>

- **table setting task**: grasping plates/cutlery from dish rack and placing it on a table.
*To Fill..*


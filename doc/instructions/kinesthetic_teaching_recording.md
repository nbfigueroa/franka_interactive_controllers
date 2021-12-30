# Kinesthetic Teaching + Data Collection and Pre-Processing

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

This code has been used for two household tasks:
- **cooking preparation task**: scooping and mixing ingredients from bowls

<p align="center">
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_task_reduced.gif" width="425x">
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/scooping_rosbag_replay.gif" width="450x">
</p>
<p align="center">
	Left: Video of kinesthetic demonstration, Right: Visualization of recorded trajectories by replaying recorded rosbag
</p>

- **table setting task**: grasping plates/cutlery from dish rack and placing it on a table.

<p align="center">
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/tablesetting_task_reduced.gif" width="375x">
	<img src="https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/img/tablesetting_rosbag_replay.gif" width="400x">
</p>
<p align="center">
	Left: Video of kinesthetic demonstration, Right: Visualization of recorded trajectories by replaying recorded rosbag
</p>


---
## Step 2: Extracting Trajectories from ROSBag Data for Task Learning

### Extracting ROSBag Data to MATLAB (Working)
To export the data recorded in the rosbags to MATLAB you can use the [rosbag_to_mat](https://github.com/nbfigueroa/rosbag_to_mat) package. Follow the instructions in the [README](https://github.com/nbfigueroa/rosbag_to_mat/blob/main/README.md) file to extract data for the following tasks:

- **cooking preparation task**: raw trajectories from demonstrations (colors indicate continuous demonstration):
<p align="center">
  <img src="https://github.com/nbfigueroa/rosbag_to_mat/blob/main/figs/franka-cooking-multistep.png" width="700x"> 
</p>

- **table setting task**: raw trajectories from demonstrations (colors indicate continuous demonstration):
<p align="center">
  <img src="https://github.com/nbfigueroa/rosbag_to_mat/blob/main/figs/franka-tablesetting-multistep.png" width="700x"> 
</p>

### Extracting ROSBag Data to Python (Experimental)
This functionality hasn't been tested yet but I suggest to try out the [bagpy](https://jmscslgroup.github.io/bagpy/): a python package provides specialized class bagreader to read and decode ROS messages from bagfiles in just a few lines of code. 


---
## Step 3: Trajectory Segmentation of Multi-Step Tasks for Motion Policy Learning
If the trajectories are continuous demonstrations of **a multi-step task** which will be represented as a **sequence of goal-oriented motion policies**, then the trajectories must be segmented.

- **cooking preparation task**: segmented and processed trajectories from demonstrations (colors indicate trajectory clusters), see [README](https://github.com/nbfigueroa/rosbag_to_mat/blob/main/README.md) for segmentation algorithm details:
<p align="center">
  <img src="https://github.com/nbfigueroa/rosbag_to_mat/blob/main/figs/franka_cooking_DS_s1_clustered_trajectories.png" width="306x"><img src="https://github.com/nbfigueroa/rosbag_to_mat/blob/main/figs/franka_cooking_DS_s2_clustered_trajectories.png" width="302x"><img src="https://github.com/nbfigueroa/rosbag_to_mat/blob/main/figs/franka_cooking_DS_s3_clustered_trajectories.png" width="300x"> 
</p>

- **table setting task**: segmented and processed trajectories from demonstrations (colors indicate trajectory clusters), see [README](https://github.com/nbfigueroa/rosbag_to_mat/blob/main/README.md) for segmentation details:

<p align="center">
  <img src="https://github.com/nbfigueroa/rosbag_to_mat/blob/main/figs/franka-tablesetting-multistep-segmented.png" width="500x"> 
</p>

---
## Next Step 
Now that the trajectories have been extracted and segmented you can learn motion policies from them! 

If you want to learn Dynamical System (DS) based motion policies following the the LPV-DS approach proposed in [Figueroa and Billard, 2018](http://proceedings.mlr.press/v87/figueroa18a.html) see 

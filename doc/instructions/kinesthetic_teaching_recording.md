# Kinesthetic Teaching/Recording

These instruction assume you have installed the [easy-kinesthetic-recording](https://github.com/nbfigueroa/easy-kinesthetic-recording) package and all of its dependencies. 
- See [README](https://github.com/nbfigueroa/easy-kinesthetic-recording/blob/latest-franka/README.md) file for installation instructions.  
- Make sure to have the ``latest-franka`` branch checked out. 

If you have the main robot launch file already running in a terminal; i.e.,
```bash
roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```

Then all you need is to run the kinesthetic teaching pipeline as follows:
```bash
roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```

Alternatively, you can run all in one terminal by setting the following parameters:
```bash
roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch load_franka_control:=true
```

### Definitions
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


### Usage
- Visualize end-effector trajectories being recorded in real-time and gripper state (open/closed).
- Bringup rosservice to record/stop a recording, see README of [easy-kinesthetic-recording](https://github.com/nbfigueroa/easy-kinesthetic-recording) for instructions (can be triggered by command line or GUI).
- Replay recorded end-effector trajectoriers and gripper state in RViz, franka controllers and launch files must be turned off for this functionality, see README of [easy-kinesthetic-recording](https://github.com/nbfigueroa/easy-kinesthetic-recording) for instructions on which launch file to define this.

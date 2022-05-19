# ICRA 2022 Inspection Task Demo Instructions

## On Old Lenovo Laptop (running the RT-Kernel Ubuntu 18)

### 1. Before starting execution of the task send robot to home joint position (franka_control_interactive.launch should NOT BE running -- make sure robot is not near a collision before running this! You can move the robot manually first):
```
$ rosrun franka_interactive_controllers libfranka_joint_goal_motion_generator 1
```

### 2. Bringup the franka interactive control interface (in one terminal):
```
$ roslaunch franka_interactive_controllers franka_control_interactive.launch
```

### 3. To execute a learned motion policy load cartesian impedance (twist) controller (in another terminal):
```
$ roslaunch franka_interactive_controllers cartesian_twist_impedance_controller.launch
```

### [extra demo]: To record demonstrations you can run the gravity compensation controller (in another terminal):
```
$ roslaunch franka_interactive_controllers joint_gravity_compensation_controller.launch
```

NOTE: Only one controller can run at the same time!

NOTE2: Every now and then you need to cleanup the log files!
```bash
$ cd /home/nbfigueroa/.ros/log
$ rm -rf *
```

## On New Lenovo Laptop (running the RT-Kernel Ubuntu 20)

### 1. Bringup visualization (in one terminal):
```
$ roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```

### 2. To execute a learned motion policy load franka-lpvds tasks (in one terminal):

#### To run the PICKING DS, run the following:
```
$ roslaunch ds_motion_generator franka_inspection_lpvDS_motionGenerator.launch ds_num:=1_1
```
This will run DS1 (pick block) with target 1. Targets denoted on box. Options are:
- 1_1: top-left
- 1_2: top-right
- 1_3: bottom-left
- 1_4: bottom-right

#### To run the INSPECTION-RELEASE DS, run the following:
```
$ roslaunch ds_motion_generator franka_inspection_lpvDS_motionGenerator.launch ds_num:=2
```

### 3. On this PC you can show the learned trajectories from MATLAB:
- Open MATLAB in terminal write the following:
```
$ run_matlab
```
Navigate to: `` /home/nadiafig/code/auto-ds-learning/figs``
Open the files: ``franka-inspection-ds1.fig`` and ``franka-inspection-ds2.fig``

You should see this:
 <p align="center">
      <img src="img/DS-learned.png" width="700x"> 
  </p>

### 4. On this PC you can also show videos of the demonstrations, trajecories and execution:
All are in this folder: ``/home/nadiafig/Desktop/videos-icra-demo ``

### Alternative [extra demo]: If you want to record demos then you can run the following and click "record" in the pop-up GUI:
```
$ roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```


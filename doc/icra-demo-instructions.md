# ICRA 2022 Inspection Task Demo Instructions

## On Old Lenovo Laptop (running the RT-Kernel Ubuntu 18)

Befor starting execution of the task send robot to home joint position (franka_control_interactive.launch should NOT BE running -- make sure robot is not near a collision before running this! You can move the robot manually first):
```
$ rosrun franka_interactive_controllers libfranka_joint_goal_motion_generator 1
```


### 1. Bringup the franka interactive control interface (in one terminal):
```
$ roslaunch franka_interactive_controllers franka_control_interactive.launch
```

### 2a. To record demonstrations you can run the gravity compensation controller (in another terminal):
```
$ roslaunch franka_interactive_controllers cartesian_twist_impedance_controller.launch
```

### 2b. To execute a learned motion policy load cartesian impedance (twist) controller (in another terminal):
```
$ roslaunch franka_interactive_controllers joint_gravity_compensation_controller.launch
```

NOTE: Only one controller can run at the same time!


## On New Lenovo Laptop (running the RT-Kernel Ubuntu 20)

### 1. Bringup visualization (in one terminal):
```
$ roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```

### 2a. To record demonstrations you can run the gravity compensation controller (in one terminal):
```
$ roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```

### 2b. To execute a learned motion policy load franka-lpvds tasks (in one terminal):
```
$
```

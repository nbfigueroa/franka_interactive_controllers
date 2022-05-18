# ICRA 2022 Inspection Task Demo Instructions

## On Old Lenovo Laptop (running the RT-Kernel Ubuntu 18)

Befor starting execution of the task send robot to home joint position (franka_control_interactive.launch should be disabled -- make sure robot is not near a collision before running this! You can move the robot manually first):
```
$ ...
```


### 1. Bringup the franka interactive control interface (in one terminal):
```
$ ...
```

### 2a. If you want to record demonstrations you can run the gravity compensation controller (in another terminal):
```
$ ...
```

### 2b. To execute a motion policy load cartesian impedance (twist) controller (in another terminal):
```
$ ...
```

NOTE: Only one controller can run at the same time!


## On New Lenovo Laptop (running the RT-Kernel Ubuntu 20)

### 1. Bringup visualization (in one terminal):
```
$ roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```

### 2a. If you want to record demonstrations you can run the gravity compensation controller (in one terminal):
```
$ roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```

### 2b. To execute a motion policy load franka-lpvds tasks (in one terminal):
```
$
```

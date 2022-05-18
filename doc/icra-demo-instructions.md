++++ On Old Lenovo Laptop (running the RT-Kernel Ubuntu 18)++++

Send robot to home joint position (franka_control_interactive.launch should be disabled -- make sure robot is not near a collision before running this! You can move the robot manually first):
```
$ ...
```


1. Bringup the franka interactive control interface:
```
$ ...
```

2a. If you want to record demonstrations you can run the gravity compensation controller:
```
$ ...
```

2b. To execute a motion policy load cartesian impedance (twist) controller
```
$ ...
```

NOTE: Only one controller can run at the same time!


++++ On New Lenovo Laptop (running the RT-Kernel Ubuntu 20)  ++++


1. Bringup visualization!:
```
$ roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```

2a. If you want to record demonstrations you can run the gravity compensation controller:
```
$ roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```

2b. To execute a motion policy load franka-lpvds tasks:
```
$
```

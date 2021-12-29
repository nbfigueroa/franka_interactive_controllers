# Basic robot operating functionalities

## Gripper Control
This repo includes a ros-nodified version of  [franka_gripper_run](https://github.com/nbfigueroa/franka_gripper_run) that uses the actionlib server from [franka_ros/franka_gripper](https://frankaemika.github.io/docs/franka_ros.html#franka-gripper). A simple action client node that open/closes the gripper can be used by running the following:
```bash
rosrun franka_interactive_controllers franka_gripper_run_node <command_type>
```
Where ``<command_type>``= 1 (close) and 0 (open).

To programatically open/close the gripper from your own code check out the franka_gripper_run_node [C++ code](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/src/franka_gripper_run_node.cpp).

You can also control the gripper with a GUI like in [franka_gripper_run](https://github.com/nbfigueroa/franka_gripper_run). To do so simply run the following script:
```bash
rosrun franka_interactive_controllers franka_gui_gripper_run.py
```

## Libfranka Joint Controllers
We also include some controllers for **joint motion generation to a goal and the open/close gripper bypassing franka_ros**; i.e. using solely the [libfranka](https://frankaemika.github.io/docs/libfranka.html) driver. 

NOTE: These cannot be used when either of the launch files above are running, but can be useful to quickly setup a robot; i.e. open/close gripper and send to a desired joint configuration.

- Move robot to desired ``q_goal``:
  ```bash
  rosrun franka_interactive_controllers libfranka_joint_goal_motion_generator <goal_id>
  ```
  See [cpp file](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/src/libfranka_joint_goal_motion_generator.cpp) for ``q_goal`` definitions, you can replace or add more as you like, you should only recompile.

- Open/Close gripper (same as above but withou the actionlib interface, using ONLY [libfranka](https://frankaemika.github.io/docs/libfranka.html)):
  ```bash
  rosrun franka_interactive_controllers libfranka_gui_gripper_run.py
  ```

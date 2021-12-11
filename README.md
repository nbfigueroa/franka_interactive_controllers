# franka_interactive_controllers

Control interface built on top of franka_ros for interactive controllers.

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

## Executing a DS Motion Generator with Franka Panda
In the repo we will use the [ds_motion_generator](https://github.com/nbfigueroa/ds_motion_generator) package to generated the desired velocity of a DS that can be learned or parametrized by different approaches.

### Cooking Task
**Step 1:** Send robot to task initial position. For now, this can be done by:
```bash
rosrun franka_interactive_controllers libfranka_joint_goal_motion_generator 2
```
- 1: home
- 2: scooping init
- 3: right bowl
- 4: center bowl
- 5: left bowl

**Step 2:** Launch cartesian impedance controller with twist command
```bash
roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```
```bash
roslaunch franka_interactive_controllers cartesian_pose_impedance_controller.launch
```

**Step 3:** Launch the desired DS Motion Generator Node (for now)

Run the following to execute DS-1: Scooping from left bowl
```bash
roslaunch ds_motion_generators franka_cooking_lpvDS_motion_generator.launch ds_num:=1
```
Run the following to execute DS-2: Pouring to center bowl
```bash
roslaunch ds_motion_generators franka_cooking_lpvDS_motion_generator.launch ds_num:=2
```
Run the following to execute DS-3: Scooping from right bowl
```bash
roslaunch ds_motion_generators franka_cooking_lpvDS_motion_generator.launch ds_num:=3
```
Run the following to execute DS-2: Pouring to center bowl
```bash
roslaunch ds_motion_generators franka_cooking_lpvDS_motion_generator.launch ds_num:=2
```

### Table Setting Task
*To fill:...*

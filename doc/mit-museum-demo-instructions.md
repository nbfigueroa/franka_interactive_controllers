# Instructions for MIT Museum Inspection Demo

First follow [robot_startup](https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/instructions/robot_startup.md) as per usual to startup the robot FCI control. Then you have **2 DEMO OPTIONS**:

 a. **Learning+Execution:** Learn new models from trajectories kinesthetically demonstrated by museum goer and execute those learned models with interactive perturbation capabilities (for this demo you need to run instructions for Phase 1 and 2 below)  
 b. **Execution:** Run pre-learned optimal models and execute them with interactive perturbation capabilities (for this demo you only need to follow intructions Phase 2 below)
 
---

### Phase 1. Learning New Motion Policy Models from Kinesthetic Demonstrations (~4min to complete)

1. To record demonstrations you can run the gravity compensation controller (in another terminal):
```
$ roslaunch franka_interactive_controllers joint_gravity_compensation_controller.launch
```
3. On this PC you can show the learned trajectories from MATLAB:
- Open MATLAB in terminal write the following:
```
$ run_matlab
```
Alternative [extra demo]: If you want to record demos then you can run the following and click "record" in the pop-up GUI:
```
$ roslaunch franka_interactive_controllers franka_kinesthetic_teaching.launch
```

---

### Phase 2. Running the Interactive Execution of Pre-Learned Models

#### On Primary Control PC w/Real-TIME Kernel (interactive2)
1. Before starting execution of the task send robot to home joint position (franka_control_interactive.launch should NOT BE running -- make sure robot is not near a collision before running this! You can move the robot manually first):
   ```bash
   rosrun franka_interactive_controllers libfranka_joint_goal_motion_generator_mit 1
   ```
2. Bringup the franka interactive control interface with museum configure rviz (in one terminal):
   ```bash
   roslaunch franka_interactive_controllers franka_museum_interactive_bringup.launch
   ```

3. Load the passive_ds_controller that will receive velocities from the learned DS motion policies and convert them to control torques:
   ```bash
   roslaunch franka_interactive_controllers passive_ds_impedance_controller.launch
   ```
   **NOTE**: 2 and 3 can be merge to same launch file once pipeline is robust.

#### The following can run on Primary (interactive2) or Secondary (interactive) Control PC 
4. Load and visualize all learned DS motion policies with real-time update trajectories:
   ```bash
   roslaunch ds_motion_generator franka_museum_inspection_lpvDS_motionGenerator.launch
   ```
   There are two options for this, you can either load the 'latest' learned models if Phase 1 was performed OR you can load the 'optimal' learned models which are ones that were learned with optimal demonstrations, this can be set as follows:
   ```xml
     <arg name="which_DS"      default="optimal"/>
     <arg name="which_DS"      default="latest"/>
   ``` 
   This is what should be visualized after executing these instructions:
    <p align="center">
    <img src="https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/doc/img/After-executing-control-instructions.png" width="600x"> 
    </p>
    Each of these trajectories represent the learned motion policies that the robot will follow once control is activated. You can physically move the robot and see how each of them will change in real-time. 
    
    - Red trajectory: DS1 towards right block picking location.
    - Green trajectory: DS1 towards left block picking location.
    - Blue trajectory: DS2 towards release station.

4 (alternative). To execute (with the robot) and visualize each learned motion policy individually run the following:
   ```bash
   roslaunch ds_motion_generator franka_museum_single_lpvDS_motionGenerator.launch ds_num:=1_left
   ```
   This will run DS1 (pick block) with target 1. Targets denoted on box. Options are:
   - 1_left:  left-most picking location (person view)
   - 1_right: right-most pickig location
   - 2: inspection ds


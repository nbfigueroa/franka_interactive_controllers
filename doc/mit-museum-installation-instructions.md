# Installation & Setup for MIT Museum Inspection Demo

1. Install and compile [franka_interactive_controllers](https://github.com/nbfigueroa/franka_interactive_controllers). 
- Follow the instructions in the [README](https://github.com/nbfigueroa/franka_interactive_controllers#readme) file verbatim.  
- Specificially the ``wstool`` lines that will download all the required dependencies.

2. Switch to the ``mit-museum-demo`` branch in the [ds_motion_generator](https://github.com/nbfigueroa/ds_motion_generator) repo:
    ```
    roscd ds_motion_generator/
    git checkout -b mit-museum-demo
    git pull . orgin/mit-museum-demo
    ```
    
3. Create a directory and subdirectories for trajectory recordings in your home folder:
    ```
    cd && mkdir museum_recordings
    cd museum_recordings && mkdir bags && mkdir mat
    ```

4. If there are any changes in the setup wrt. to the configuration in MIT IRG 32 lab regarding relative positions of the stations wrt. the base of the robot the ``computeFrankaInspectionTransforms.m`` function within the [rosbag_to_mat](https://github.com/nbfigueroa/rosbag_to_mat) repo needs to be modified to define the relative reference frames:

    ```matlab
    %% CODE TO MODIFY HERE
    ```


5. Download the [auto-ds-learning](https://github.com/nbfigueroa/auto-ds-learning) code in your ``~./catkin_ws/src/``folder, this code runs in MATLAB and is used to automagically segment the trajectories for DS1: Reach-to-grasp Cubes, DS2: Inspection Task and Release, and learn the 2 DS models as lpv-DS. 
- Once downloaded in the MATLAB Command Window run `` setup_code.m``

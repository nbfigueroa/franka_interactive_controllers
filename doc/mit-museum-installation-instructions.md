# Installation & Setup for MIT Museum Inspection Demo

1. Install and compile [franka_interactive_controllers](https://github.com/nbfigueroa/franka_interactive_controllers). 
    - Follow the instructions in the [README](https://github.com/nbfigueroa/franka_interactive_controllers#readme) file verbatim.  
    - The ``wstool`` lines should be modified as below:
        ```bash
        $  wstool init
        $  wstool merge franka_interactive_controllers/dependencies_museum.rosinstall 
        $  wstool up 
        ```
    - If you already have a franka_ros installed and it complains about being a different version, select skip.
    - Continue to build or make as usual.

<!-- 2. Switch to the ``mit-museum-demo`` branch in the [ds_motion_generator](https://github.com/nbfigueroa/ds_motion_generator) repo:
    ```bash
    roscd ds_motion_generator/
    git checkout -b mit-museum-demo
    git pull . orgin/mit-museum-demo
    ``` -->
    
2. If not done so already, create a directory and subdirectories for trajectory recordings in your home folder or at the same level as your ``~/catkin_ws``, following commands assuming workspace is in home dir:
    ```bash
    cd && mkdir museum_recordings
    cd museum_recordings && mkdir bags && mkdir mat
    ```

3. If not done so already, add an alias in your ``~/.bashrc`` to open MATLAB from within the auto-ds-learning folder:
    ```bash
    # To run matlab within catkin_was for museum demo
    alias run_matlab_museum='roscd && cd ../src/auto-ds-learning && matlab' 
    ```
4. Setup MATLAB directories for demo.
 - If ds-libraries haven't been compiled yet run the following in the MATLAB Command Window
     ```matlab
     >> setup_museum_code    
     ```
 - If they have been compiled (i.e., mex files created for lightspeed library) then just run:
     ```matlab
     >> setup_museum_code(0)    
     ```
4. If there are any changes in the setup wrt. to the physical configuration in MIT IRG 32 lab regarding relative positions of the stations wrt. the base of the robot you should navigate to this folder: ``~/catkin_ws/src/rosbag_to_mat/tasks/industrial`` within the [rosbag_to_mat](https://github.com/nbfigueroa/rosbag_to_mat) repo and modify this function``computeFrankaInspectionTransforms.m`` function to the corrct relative reference frames wrt. robot base:

    ```matlab
        function [H_pickup_station, H_inspection_tunnel, H_release_station] = computeFrankaInspectionTransforms(is_museum)
            if is_museum
                % IF ANYTHING CHANGES IN THE SETUP (RELATIVE POSITIONS TO BASE OF ROBOT) 
                % THESE VALUES NEED TO BE CHANGED, 
                % THEY REPRESENT THE ORIGINS OF THE FRAMES OF EACH STATION WRT. THE BASE OF THE ROBOT

                H_pickup_station= eye(4); H_pickup_station(1:3,4) = [0.525, -0.475, 0.05]; %<== CHANGE/CHECK THESE VALUES WHEN MOVING TO MUSEUM 
                H_inspection_tunnel= eye(4); H_inspection_tunnel(1:3,4) = [0.575, 0.0, 0.05]; %<== CHANGE/CHECK THESE VALUES WHEN MOVING TO MUSEUM
                H_release_station= eye(4); H_release_station(1:3,4) = [0.575, 0.45, 0.05]; %<== CHANGE/CHECK THESE VALUES WHEN MOVING TO MUSEUM

                % If we keep the same relative distances between the stations but remove
                % the white tableop then we can simply remove the 5cm height
            else
                % TRANSFORMS FOR PENN SETUP IN FIGUEROA LAB
                H_pickup_station= eye(4); H_pickup_station(1:3,4) = [0.5, -0.5, 0.00];
                H_inspection_tunnel= eye(4); H_inspection_tunnel(1:3,4) = [0.5, 0.05, 0.00];
                H_release_station= eye(4); H_release_station(1:3,4) = [0.5, 0.55, 0.00];
            end
        end
    ```


5. Download the [auto-ds-learning](https://github.com/nbfigueroa/auto-ds-learning) code in your ``~./catkin_ws/src/``folder, this code runs in MATLAB and is used to automagically segment the trajectories for DS1: Reach-to-grasp Cubes, DS2: Inspection Task and Release, and learn the 2 DS models as lpv-DS. 
- Once downloaded in the MATLAB Command Window run `` setup_code.m``

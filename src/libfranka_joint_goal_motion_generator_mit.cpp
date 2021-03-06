// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Easy non-franka_ros executable to send robot to desired joint configurations. 
// Caution: It won't work when franka_ros/franka_control is running!

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

// #include <franka_motion_generators/libfranka_joint_motion_generator.h>
#include <libfranka_joint_motion_generator.h>

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

int main(int argc, char** argv) {

  std::string franka_ip = "172.16.0.2";

  // Check whether the required arguments were passed replace this with rosparam!
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <goal_id>" << std::endl;
    return -1;
  }

  try {
    
    // Connect to robot.
    franka::Robot robot(franka_ip);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});


    // First move the robot to a suitable joint configuration
    int goal_id = std::stod(argv[1]);

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    switch(goal_id) {
       case 1  :
          std::cout << "Selected q_home as goal" << std::endl;
          // q_goal = {{-0.13169961199844094, -0.2061586460920802, 0.03348877041015708, -2.1582989016750402, -0.005362026724136538, 2.053694872394686, 0.8156176816517178}}; 
          // q_goal = {{-0.10576196822576356, -0.3352823379667182, 0.07229093052145613, -1.9880429509648103, 0.0011565770285411011, 1.7324491872743322, 0.841189909406834}};          
          // q_goal = {{0, -0.11819894895009828, 0.31241208355469163, -2.0810536406332987, 0.032358223563513575, 2.024914488492917, 0.9259632611185911}};
          q_goal = {{0.0, -0.1516284550464292, 0.0, -2.1602991589328693, 0.0063609233109487425, 2.030401506252017, 0.8428664707532099}};
          break;

       case 2  :
        std::cout << "Selected q_pick_1_up as goal" << std::endl;
          q_goal = {{-0.661667437122652, 0.29157752645509677, -0.007866171296009966, -1.6218121079562002, -0.021423585925581565, 1.9639725402990975, 0.10225348567797078}};          
          break;

       case 3  :
          std::cout << "Selected q_pick_1 as goal" << std::endl;
          q_goal = {{-0.6645273388722563, 0.40625634067936944, -0.006472443358286204, -1.760182324727376, -0.02029987028737863, 2.191272726456324, 0.1430827652994129}};
          break;

       case 4  :
        std::cout << "Selected q_release as goal" << std::endl;
          q_goal = {{0.6232106392258092, 0.4991823822363217, 0.30459266481276853, -1.3196656081426748, -0.19566767448184402, 1.825254593404243, 1.6828069603029223}};
          break;

    }
  
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}

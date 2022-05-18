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
          // q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
          q_goal = {{0.009340305111597252, -0.36729619687063647, 0.05388360048542943, -2.3592505386694267, 0.016102603363234352, 2.0088766928513846, 0.8014595653550303}};

          break;
       case 2  :
        std::cout << "Selected q_init_scoop as goal" << std::endl;
          q_goal = {{-0.2587090488839568, -0.18067890287296676, -0.1481914834306951, -2.2218669233824073, 1.2397120203356886, 1.6055843360223088, -0.2564202219950203}};
          break;
       case 3  :
        std::cout << "Selected q_right_plate as goal" << std::endl;
          q_goal = {{-0.5133883270192566, 0.2710828751293255, -0.300302759789584, -1.807947067027922, 1.3988803114257669, 1.3803889200108581, -0.31572859715720264}};
          break;

       case 4  :
        std::cout << "Selected q_center_plate as goal" << std::endl;
          q_goal = {{-0.1478659114867632, 0.20867810028895994, -0.3032390865134677, -2.0419724096954726, 1.4192992324987155, 1.480286537010783, -0.4761567130958154}};
          break;

       case 5  :
        std::cout << "Selected q_left_table_setting as goal" << std::endl;
        q_goal = {{-0.024844449233219813, 0.21341741475306059, 0.1671374870475969, -1.9734624159963505, 1.6724220574752517, 2.054275230565774, -0.36437520284810354}};
        break;
          
       case 6 :
        std::cout << "Selected q_left_table_setting as goal" << std::endl;
        q_goal = {{-0.3359993156361998, -0.1753333669566437, 0.2292614262647741, -1.941019418460696, 0.08810859725369569, 1.9533451146157188, 0.7877697710477642}};
        break;

    }
  
    MotionGenerator motion_generator(0.6, q_goal);
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

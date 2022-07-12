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
          // q_goal = {{0.009340305111597252, -0.36729619687063647, 0.05388360048542943, -2.3592505386694267, 0.016102603363234352, 2.0088766928513846, 0.8014595653550303}};
          // q_goal = {{0.013076832091468467, 0.05892901891679095, 0.05233293692648427, -2.289458147450497, 0.014325451839091376, 2.3411785166360315, 0.8852154568417203}};
          //q_goal = {{-0.0026496768670445665, -0.09072233400129963, 0.05107856889461215, -2.412414035562883, -0.02818185633089807, 2.3957946495215094, 0.7987457616245822}};
          q_goal = {{-0.04125446628949098, -0.23194385116560415, 0.037441184571944, -2.1041153984069823, 0.004531050744156042, 1.8886842381954192, 0.796802966348411}};         
          break;
       case 2  :
        std::cout << "Selected q_pick_1_up as goal" << std::endl;
          // q_goal = {{-0.542943401353401, 0.28187752648403763, -0.36353780205626235, -1.6254522685168082, 0.12447580540091537, 1.9568485025564828, -0.2251955777760423}};
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
       // case 4  :
       //  std::cout << "Selected q_center_plate as goal" << std::endl;
       //    q_goal = {{-0.1478659114867632, 0.20867810028895994, -0.3032390865134677, -2.0419724096954726, 1.4192992324987155, 1.480286537010783, -0.4761567130958154}};
       //    break;

       // case 5  :
       //  std::cout << "Selected q_left_table_setting as goal" << std::endl;
       //  q_goal = {{-0.024844449233219813, 0.21341741475306059, 0.1671374870475969, -1.9734624159963505, 1.6724220574752517, 2.054275230565774, -0.36437520284810354}};
       //  break;
          
       // case 6 :
       //  std::cout << "Selected q_left_table_setting as goal" << std::endl;
       //  q_goal = {{-0.3359993156361998, -0.1753333669566437, 0.2292614262647741, -1.941019418460696, 0.08810859725369569, 1.9533451146157188, 0.7877697710477642}};
       //  break;

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

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
          // NEW (AT MIT MUSEUM)
          // q_goal = {{0.005264432735087578, -0.17525178575515743, 0.08762187454773385, -1.8310899806440901, 0.029428643955124744, 1.680440888034793, 0.9123517645864205}};
          q_goal = {{-0.04806250403248165, -0.37834688621146906, 0.1189895272266932, -1.8715291539992651, 0.050834695322646034, 1.5422987513009265, 0.912757925618026}};
          break;

       case 2  :
          std::cout << "Selected q_grasping_region as goal" << std::endl;
          q_goal = {{-0.5976609923274894, 0.1721807560612435, -0.0781738298007447, -1.2002956697987666, 0.025287481072851427, 1.387947767177923, 0.17344659271505508}};
          break;

       case 3  :
        std::cout << "Selected q_grasping_top as goal" << std::endl;
          q_goal = {{-0.2714547964501799, 0.2540787896587189, -0.3529668585945417, -1.5453996790823856, 0.0573947018395276, 1.82454251794786, 0.2228568423615843}};          
          break;

       case 4  :
        std::cout << "Selected q_grasping_bottom as goal" << std::endl;
          q_goal = {{-0.30771123454654425, -0.001541829403979998, -0.41856739124812575, -1.859159467392586, 0.032951128009292806, 1.836483217517164, 0.10710111550593635}};
          break;

       case 5  :
        std::cout << "Selected q_inspection_start as goal" << std::endl;
          q_goal = {{-0.30604656812194647, -0.19267341885143768, 0.04989642749880722, -1.9907493218839276, 0.03521259250552366, 1.7972743010520935, 0.5712542148705879}};
          break;

       case 6  :
        std::cout << "Selected q_release as goal" << std::endl;
          q_goal = {{0.3400289639525856, 0.3321357016521588, 0.5061410089013172, -1.397782833199752, -0.18531917733638592, 1.7465957514272623, 1.6471831874270462}};
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

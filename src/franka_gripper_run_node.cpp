#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "franka_gripper_run_node");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  actionlib::SimpleActionClient<franka_gripper::MoveAction> ac_move("/franka_gripper/move", true);
  actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp("/franka_gripper/grasp", true);

  ROS_INFO("Waiting for MoveAction server to start.");
  ROS_INFO("Waiting for GraspAction server to start.");
  ac_move.waitForServer();
  ac_grasp.waitForServer();
  franka_gripper::MoveGoal move_goal;
  franka_gripper::GraspGoal grasp_goal;

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << "<open/close>" << std::endl;
    return -1;
  }
  try {
    ROS_INFO("Action server started, waiting for goals.");
    
    std::stringstream ss(argv[1]);
    bool open_close;
    if (!(ss >> open_close)) {
      std::cerr << "<open/close> must be either 0 (close) or 1 (open)." << std::endl;
      return -1;
    }
    bool success (false);

    if (open_close) {

      // open gripper
      std::cout << "Opening gripper" << std::endl;

      // Creating goal for moving action
      move_goal.width = 0.08;
      move_goal.speed = 0.1;
      ac_move.sendGoal(move_goal);

      //wait for the action to return
      success = ac_move.waitForResult(ros::Duration(10.0));      

    } else {
      // close gripper
      std::cout << "Closing gripper" << std::endl;
      
      // Creating goal for grasping action with 30N force
      grasp_goal.width = 0.0;
      grasp_goal.speed = 0.1;
      grasp_goal.force = 30;
      ac_grasp.sendGoal(grasp_goal);
      
      //wait for the action to return
      success = ac_grasp.waitForResult(ros::Duration(10.0));
      
    }
  
    std::cout << "Result: " << success << std::endl;

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;

}
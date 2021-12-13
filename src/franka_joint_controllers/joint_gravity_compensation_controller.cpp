// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <joint_gravity_compensation_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <pseudo_inversion.h>
#include <hardware_interface/joint_command_interface.h>

namespace franka_interactive_controllers {

bool JointGravityCompensationController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  // Getting ROSParams
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("JointGravityCompensationController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointGravityCompensationController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Initialize variables for tool compensation from yaml config file
  activate_tool_compensation_ = true;
  std::vector<double> external_tool_compensation;
  if (!node_handle.getParam("external_tool_compensation", external_tool_compensation)) {
      ROS_ERROR(
          "JointGravityCompensationController: Invalid or no external_tool_compensation parameters provided, "
          "aborting controller init!");
      return false;
    }

  tool_compensation_force_.setZero();  
  for (size_t i = 0; i < 6; ++i) 
    tool_compensation_force_[i] = external_tool_compensation.at(i);
  ROS_INFO_STREAM("External tool compensation force: " << std::endl << tool_compensation_force_);
  // tool_compensation_force_ << 0.46, -0.17, -1.64, 0, 0, 0;  //read from yaml

  // Getting libranka control interfaces
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointGravityCompensationController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointGravityCompensationController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointGravityCompensationController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointGravityCompensationController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointGravityCompensationController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointGravityCompensationController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Getting Dynamic Reconfigure objects
  dynamic_reconfigure_gravity_compensation_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_gravity_compensation_param_node");

  dynamic_server_gravity_compensation_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_interactive_controllers::gravity_compensation_paramConfig>>(

      dynamic_reconfigure_gravity_compensation_param_node_);
  dynamic_server_gravity_compensation_param_->setCallback(
      boost::bind(&JointGravityCompensationController::gravitycompensationParamCallback, this, _1, _2));

  
  // Initialize variables for joint locks
  set_locked_joints_position_ = false;
  activate_lock_joint6_ = false;
  activate_lock_joint7_ = false;
  k_lock_      = 50; 
  q_locked_joints_.setZero();

  return true;
}

void JointGravityCompensationController::starting(const ros::Time& /*time*/) {
  // Get robot current/initial joint state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
}

void JointGravityCompensationController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());

  // compute control
  // allocate variables
  Eigen::VectorXd tau_d(7), tau_task(7), tau_nullspace(7), tau_tool(7);

  // pseudoinverse for nullspace handling kinematic pseudoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Set 0 torques for the controller
  tau_task.setZero();

  // Compute tool compensation (scoop/camera in scooping task)
  if (activate_tool_compensation_)
    tau_tool << jacobian.transpose() * tool_compensation_force_;
  else
    tau_tool.setZero();

  if (activate_lock_joint6_){
    double tau_task_6 = -k_lock_*(q[5] -  q_locked_joints_[5]) ;
    std::cout << "tau_task_6: " << tau_task_6 << std::endl;
    tau_task[5] = tau_task_6;
  }

  if (activate_lock_joint7_){
    double tau_task_7 = -k_lock_*(q[6] -  q_locked_joints_[6]) ;
    std::cout << "tau_task_7: " << tau_task_7 << std::endl;
    tau_task[6] = tau_task_7;
  }

  // Desired torque (Check this.. might not be necessary)
  tau_d << tau_task + coriolis - tau_tool;

  // Alternative 
  // tau_d.setZero();

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
}

Eigen::Matrix<double, 7, 1> JointGravityCompensationController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void JointGravityCompensationController::gravitycompensationParamCallback(
    franka_interactive_controllers::gravity_compensation_paramConfig& config,
    uint32_t /*level*/) {
  
  // To activate external tool compensation
  activate_tool_compensation_ = config.activate_tool_compensation;
  
  // To lock a specific joint
  activate_lock_joint6_                = config.activate_lock_joint6;
  activate_lock_joint7_                = config.activate_lock_joint7;
  
  set_locked_joints_position_ = config.set_locked_joints_position;
  if (set_locked_joints_position_){
      franka::RobotState locked_state = state_handle_->getRobotState();
      Eigen::Map<Eigen::Matrix<double, 7, 1>> q_locked_joints(locked_state.q.data());
      q_locked_joints_ = q_locked_joints;
      ROS_INFO_STREAM("Locked Joints Set to: " << q_locked_joints_);
  }
}


}  // namespace franka_interactive_controllers

PLUGINLIB_EXPORT_CLASS(franka_interactive_controllers::JointGravityCompensationController,
                       controller_interface::ControllerBase)

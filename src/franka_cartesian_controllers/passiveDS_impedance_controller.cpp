// This code was derived from franka_example controllers
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license.
// Current development and modification of this code by Nadia Figueroa (MIT) 2021.

#include <passiveDS_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <pseudo_inversion.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/filters.h>


namespace franka_interactive_controllers {

bool PassiveDSImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;


  sub_desired_twist_ = node_handle.subscribe(
      "/cartesian_impedance_controller/desired_twist", 20, &PassiveDSImpedanceController::desiredTwistCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Getting ROSParams
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("PassiveDSImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Initialize variables for tool compensation from yaml config file
  activate_tool_compensation_ = true;
  tool_compensation_force_.setZero();
  std::vector<double> external_tool_compensation;
  // tool_compensation_force_ << 0.46, -0.17, -1.64, 0, 0, 0;  //read from yaml
  if (!node_handle.getParam("external_tool_compensation", external_tool_compensation) || external_tool_compensation.size() != 6) {
      ROS_ERROR(
          "PassiveDSImpedanceController: Invalid or no external_tool_compensation parameters provided, "
          "aborting controller init!");
      return false;
    }
  for (size_t i = 0; i < 6; ++i) 
    tool_compensation_force_[i] = external_tool_compensation.at(i);
  ROS_INFO_STREAM("External tool compensation force: " << std::endl << tool_compensation_force_);

  // Initialize variables for nullspace control from yaml config file
  q_d_nullspace_.setZero();
  std::vector<double> q_nullspace;
  if (node_handle.getParam("q_nullspace", q_nullspace)) {
    q_d_nullspace_initialized_ = true;
    if (q_nullspace.size() != 7) {
      ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no q_nullspace parameters provided, "
        "aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 7; ++i) 
      q_d_nullspace_[i] = q_nullspace.at(i);
    ROS_INFO_STREAM("Desired nullspace position (from YAML): " << std::endl << q_d_nullspace_);
  }


  if (!node_handle.getParam("nullspace_stiffness", nullspace_stiffness_target_) || nullspace_stiffness_target_ <= 0) {
    ROS_ERROR(
      "PassiveDSImpedanceController: Invalid or no nullspace_stiffness parameters provided, "
      "aborting controller init!");
    return false;
  }
  ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);


  // Getting libranka control interfaces
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PassiveDSImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PassiveDSImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // // Getting Dynamic Reconfigure objects == CHANGE THIS TO THE PASSIVE DS PARAMS!!!
  // dynamic_reconfigure_compliance_param_node_ =
  //     ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  // dynamic_server_compliance_param_ = std::make_unique<
  //     dynamic_reconfigure::Server<franka_interactive_controllers::passive_ds_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
  // dynamic_server_compliance_param_->setCallback(
  //     boost::bind(&PassiveDSImpedanceController::complianceParamCallback, this, _1, _2));


  // Initialize Passive DS controller
  passive_ds_controller.reset(new DSController(3,50.0,50.0));

  /// Getting Dynamic Reconfigure objects for controllers
  dynamic_reconfigure_passive_ds_param_node_ =
    ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_passive_ds_param_node");
  dynamic_server_passive_ds_param_ = std::make_unique<
    dynamic_reconfigure::Server<franka_interactive_controllers::passive_ds_paramConfig>>(dynamic_reconfigure_passive_ds_param_node_);
    //This line might not be necessary
  // dynamic_server_passive_ds_param_.reset(new dynamic_reconfigure::Server<franka_interactive_controllers::passive_ds_paramConfig>(dynamic_reconfigure_passive_ds_param_node_));
  dynamic_server_passive_ds_param_->setCallback(
    boost::bind(&PassiveDSImpedanceController::passiveDSParamCallback, this, _1, _2));
  dynamic_server_passive_ds_param_->getConfigDefault(config_cfg);

  // Passive DS Variable Initializatin=on
  dx_linear_des_.resize(3);
  dx_linear_msr_.resize(3);
  orient_error.resize(3);
  F_linear_des_.resize(3);
  F_angular_des_.resize(3);
  F_ee_des_.resize(6);

  // Setting with default paramsss
  rot_stiffness = config_cfg.rot_stiffness;
  rot_damping   = config_cfg.rot_damping;
  bDebug      = config_cfg.debug;
  bSmooth     = config_cfg.bSmooth;
  smooth_val_ = config_cfg.smooth_val;
  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner(3, 3) << config_cfg.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) << config_cfg.rotational_stiffness * Eigen::Matrix3d::Identity();


  // Initializing variables
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  velocity_d_.setZero();

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void PassiveDSImpedanceController::starting(const ros::Time& /*time*/) {

  // Get robot current/initial joint state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set desired point to current state
  position_d_           = initial_transform.translation();
  orientation_d_        = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_    = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  if (!q_d_nullspace_initialized_) {
    q_d_nullspace_ = q_initial;
    q_d_nullspace_initialized_ = true;
    ROS_INFO_STREAM("Desired nullspace position (from q_initial): " << std::endl << q_d_nullspace_);
  }

  // To compute 0 velocities if no command has been given
  elapsed_time    = ros::Duration(0.0);
  last_cmd_time   = 0.0;
  vel_cmd_timeout = 0.1;

}

void PassiveDSImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // Current and Desired EE velocity
  Eigen::Matrix<double, 6, 1> velocity;
  Eigen::Matrix<double, 6, 1> velocity_desired_;
  velocity << jacobian * dq;
  velocity_desired_.setZero();
  velocity_desired_.head(3) << velocity_d_;
  // ROS_INFO_STREAM("current linear velocity: " << velocity.head(3));

  // Check velocity command
  elapsed_time += period;
  if(ros::Time::now().toSec() - last_cmd_time > vel_cmd_timeout){
    velocity_d_.setZero();
    ROS_WARN_STREAM_THROTTLE(1,"Command Timeout!");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////              COMPUTING TASK CONTROL TORQUE           //////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_task_passive(7), tau_nullspace(7), tau_d(7), tau_tool(7);


  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //++++++++++++++ PASSIVE DS CONTROL FOR CARTESIAN COMMAND ++++++++++++++//
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  F_ee_des_.setZero();

  /// set desired linear velocity
  dx_linear_des_(0)   = velocity_d_(0);
  dx_linear_des_(1)   = velocity_d_(1);
  dx_linear_des_(2)   = velocity_d_(2);


  /// set measured linear and angular velocity
  if(bSmooth)
  {
  dx_linear_msr_(0)   = velocity(0);
  dx_linear_msr_(1)   = velocity(1);
  dx_linear_msr_(2)   = velocity(2);

  // dx_angular_msr_(0)  = x_msr_vel_.rot(0);
  // dx_angular_msr_(1)  = x_msr_vel_.rot(1);
  // dx_angular_msr_(2)  = x_msr_vel_.rot(2);
  }else{
  dx_linear_msr_(0)    = filters::exponentialSmoothing(velocity(0), dx_linear_msr_(0),smooth_val_);
  dx_linear_msr_(1)    = filters::exponentialSmoothing(velocity(1), dx_linear_msr_(2),smooth_val_);
  dx_linear_msr_(2)    = filters::exponentialSmoothing(velocity(2), dx_linear_msr_(1),smooth_val_);

  // dx_angular_msr_(0)   = filters::exponentialSmoothing(x_msr_vel_.rot(0), dx_angular_msr_(0),smooth_val_);
  // dx_angular_msr_(1)   = filters::exponentialSmoothing(x_msr_vel_.rot(1), dx_angular_msr_(2),smooth_val_);
  // dx_angular_msr_(2)   = filters::exponentialSmoothing(x_msr_vel_.rot(2), dx_angular_msr_(1),smooth_val_);
  }


  // ----------------- Linear Velocity Error -> Force -----------------------//

  passive_ds_controller->Update(dx_linear_msr_,dx_linear_des_);
  F_linear_des_ = passive_ds_controller->control_output(); // (3 x 1)
  F_ee_des_.head(3) = F_linear_des_;

  // ----------------- Debug -----------------------//
  ROS_WARN_STREAM_THROTTLE(1, "Desired Velocity:" << dx_linear_des_(0) << " " << dx_linear_des_(1) <<  " " << dx_linear_des_(2));
  Vec dx_error_;
  dx_error_.resize(3);
  dx_error_ = dx_linear_msr_ - dx_linear_des_;

  // ROS_WARN_STREAM_THROTTLE(1, "Current Velocity :" << dx_linear_msr_(0) << " " << dx_linear_msr_(1) <<  " " << dx_linear_msr_(2));
  ROS_WARN_STREAM_THROTTLE(1, "Current Velocity Error:" << dx_error_.norm());
  ROS_WARN_STREAM_THROTTLE(1, "Linear Control Forces :" << F_ee_des_(0) << " " << F_ee_des_(1) << " " << F_ee_des_(2));    

  // ----------------- Orientation Error -> Force -----------------------// 
  //(Potentially change to stiffness+damping orientation controller as in kuka-lwr controller)

  // Vec orient_error;
  orient_error.setZero();

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
  orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  orient_error << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

  F_angular_des_ << -cartesian_stiffness_.bottomRightCorner(3, 3)*orient_error;
  F_ee_des_.tail(3) = F_angular_des_; 

  // ----------------- Debug -----------------------//
  // ROS_WARN_STREAM_THROTTLE(1, "Desired Orientation Error :" << orient_error(0) << " " << orient_error(1) <<  " " << orient_error(2));
  // ROS_WARN_STREAM_THROTTLE(1, "Angular Control Force :" << F_ee_des_.tail(3).norm());

  // Convert control wrench to torque
  ROS_WARN_STREAM_THROTTLE(1, "Passive DS Linear Control Force:" << F_ee_des_.head(3).norm());
  tau_task_passive << jacobian.transpose() *F_ee_des_;

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //++++++++++++++ CLASSICAL IMPEDANCE CONTROL FOR CARTESIAN COMMAND ++++++++++++++//
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  // Compute task-space errors
  Eigen::Matrix<double, 6, 1> pose_error;
  pose_error.setZero();

  // --- Pose Error  --- //     
  position_d_        << position + velocity_d_*dt_*200; //Scaling to make it faster!
  pose_error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  // Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  pose_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  
  // Transform to base frame
  pose_error.tail(3) << -transform.linear() * pose_error.tail(3);
  // ROS_WARN_STREAM_THROTTLE(1,"orient error: " << pose_error.norm());

  // Computing control torque from cartesian pose error form integrated velocity command
  F_ee_des_ << -cartesian_stiffness_ * pose_error - cartesian_damping_ * velocity;
  tau_task << jacobian.transpose() * F_ee_des_;
  ROS_WARN_STREAM_THROTTLE(1, "Classic Linear Control Force:" << F_ee_des_.head(3).norm());
  ROS_WARN_STREAM_THROTTLE(1, "Classic Angular Control Force :" << F_ee_des_.tail(3).norm());
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //++++++++++++++ ADDITIONAL CONTROL TORQUES (NULLSPACE AND TOOL COMPENSATION) ++++++++++++++//
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


  // pseudoinverse for nullspace handling
  // kinematic pseudoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  double tau_nullspace_0 = tau_nullspace(0);
  tau_nullspace.setZero();
  tau_nullspace[0] = tau_nullspace_0;                     
                       
  // Compute tool compensation (scoop/camera in scooping task)
  if (activate_tool_compensation_)
    tau_tool << jacobian.transpose() * tool_compensation_force_;
  else
    tau_tool.setZero();

  // FINAL DESIRED CONTROL TORQUE SENT TO ROBOT
  tau_d << tau_task + tau_nullspace + coriolis - tau_tool;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> PassiveDSImpedanceController::saturateTorqueRate(
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

void PassiveDSImpedanceController::passiveDSParamCallback(
    franka_interactive_controllers::passive_ds_paramConfig& config,
    uint32_t /*level*/) {

  // Passive DS params 
  passive_ds_controller->set_damping_eigval(config.damping_eigval0,config.damping_eigval1);
  rot_stiffness = config.rot_stiffness;
  rot_damping   = config.rot_damping;
  bDebug        = config.debug;
  bSmooth       = config.bSmooth;
  smooth_val_   = config.smooth_val;
  config_cfg    = config;


  // Standard impedance params
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  

  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  
  nullspace_stiffness_target_ = config.nullspace_stiffness;

  activate_tool_compensation_ = config.activate_tool_compensation;
}


void PassiveDSImpedanceController::desiredTwistCallback(
    const geometry_msgs::TwistConstPtr& msg) {

  
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());

  velocity_d_      << msg->linear.x, msg->linear.y, msg->linear.z;
  last_cmd_time    = ros::Time::now().toSec();

  // ROS_INFO_STREAM("[CALLBACK] Desired velocity from DS: " << velocity_d_);
  // ROS_INFO_STREAM("[CALLBACK] Desired ee position from DS: " << position_d_target_);


  // position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  
  // Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  // orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
  //     msg->pose.orientation.z, msg->pose.orientation.w;
  
  // if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
  //   orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  // }
}

}  // namespace franka_interactive_controllers

PLUGINLIB_EXPORT_CLASS(franka_interactive_controllers::PassiveDSImpedanceController,
                       controller_interface::ControllerBase)

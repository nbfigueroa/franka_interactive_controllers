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

  // // Initialize variables for tool compensation from yaml config file
  // activate_tool_compensation_ = true;
  // tool_compensation_force_.setZero();
  // std::vector<double> external_tool_compensation;
  // // tool_compensation_force_ << 0.46, -0.17, -1.64, 0, 0, 0;  //read from yaml
  // if (!node_handle.getParam("external_tool_compensation", external_tool_compensation) || external_tool_compensation.size() != 6) {
  //     ROS_ERROR(
  //         "PassiveDSImpedanceController: Invalid or no external_tool_compensation parameters provided, "
  //         "aborting controller init!");
  //     return false;
  //   }
  // for (size_t i = 0; i < 6; ++i) 
  //   tool_compensation_force_[i] = external_tool_compensation.at(i);
  // ROS_INFO_STREAM("External tool compensation force: " << std::endl << tool_compensation_force_);

  // // Initialize variables for nullspace control from yaml config file
  // q_d_nullspace_.setZero();
  // std::vector<double> q_nullspace;
  // if (node_handle.getParam("q_nullspace", q_nullspace)) {
  //   q_d_nullspace_initialized_ = true;
  //   if (q_nullspace.size() != 7) {
  //     ROS_ERROR(
  //       "PassiveDSImpedanceController: Invalid or no q_nullspace parameters provided, "
  //       "aborting controller init!");
  //     return false;
  //   }
  //   for (size_t i = 0; i < 7; ++i) 
  //     q_d_nullspace_[i] = q_nullspace.at(i);
  //   ROS_INFO_STREAM("Desired nullspace position (from YAML): " << std::endl << q_d_nullspace_);
  // }


  // if (!node_handle.getParam("nullspace_stiffness", nullspace_stiffness_target_) || nullspace_stiffness_target_ <= 0) {
  //   ROS_ERROR(
  //     "PassiveDSImpedanceController: Invalid or no nullspace_stiffness parameters provided, "
  //     "aborting controller init!");
  //   return false;
  // }
  // ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);


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


  ///////////////////////////////////////////////////////////////////////////
  ////////////////  Parameter Initialization from YAML FILES!!!     /////////
  ///////////////////////////////////////////////////////////////////////////
  update_impedance_params_    = false; // When set to true from dynamic reconfigure will overwrite yaml file values
  
  // Initialize classical impedance stiffness and damping gains
  cartesian_stiffness_target_.setIdentity();
  cartesian_damping_target_.setIdentity();
  std::vector<double> cartesian_stiffness_target_yaml;
  if (!node_handle.getParam("cartesian_stiffness_target", cartesian_stiffness_target_yaml) || cartesian_stiffness_target_yaml.size() != 6) {
    ROS_ERROR(
      "PassiveDSImpedanceController: Invalid or no cartesian_stiffness_target_yaml parameters provided, "
      "aborting controller init!");
    return false;
  }
  for (int i = 0; i < 6; i ++) {
    cartesian_stiffness_target_(i,i) = cartesian_stiffness_target_yaml[i];
  }
  // Damping ratio = 1
  default_cart_stiffness_target_ << 300, 300, 300, 50, 50, 50;
  for (int i = 0; i < 6; i ++) {
    if (cartesian_stiffness_target_yaml[i] == 0.0)
      cartesian_damping_target_(i,i) = 2.0 * sqrt(default_cart_stiffness_target_[i]);
    else
      cartesian_damping_target_(i,i) = 2.0 * sqrt(cartesian_stiffness_target_yaml[i]);
  }
  ROS_INFO_STREAM("cartesian_stiffness_target_: " <<  cartesian_stiffness_target_.transpose());
  ROS_INFO_STREAM("cartesian_damping_target_: " << cartesian_damping_target_.transpose());

  // Initialize PassiveDS params
  damping_eigvals_yaml_.setZero();
  std::vector<double> damping_eigvals;
  if (node_handle.getParam("damping_eigenvalues", damping_eigvals)) {
    if (damping_eigvals.size() != 3) {
      ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no damping_eigenvalues parameters provided, "
        "aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 3; ++i) 
      damping_eigvals_yaml_[i] = damping_eigvals.at(i);
    ROS_INFO_STREAM("Damping Matrix Eigenvalues (from YAML): " << damping_eigvals_yaml_);
  }

  // Initialize Passive DS controller
  damping_eigval0_ = damping_eigvals_yaml_(0);
  damping_eigval1_ = damping_eigvals_yaml_(1);
  passive_ds_controller.reset(new DSController(3,damping_eigval0_,damping_eigval1_)); 

  // Initialize nullspace params
  if (!node_handle.getParam("nullspace_stiffness", nullspace_stiffness_target_) || nullspace_stiffness_target_ <= 0) {
    ROS_ERROR(
      "PassiveDSImpedanceController: Invalid or no nullspace_stiffness parameters provided, "
      "aborting controller init!");
    return false;
  }
  ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);

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


  /// Getting Dynamic Reconfigure objects for controllers
  dynamic_reconfigure_passive_ds_param_node_ =
    ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_passive_ds_param_node");
  dynamic_server_passive_ds_param_ = std::make_unique<
    dynamic_reconfigure::Server<franka_interactive_controllers::passive_ds_paramConfig>>(dynamic_reconfigure_passive_ds_param_node_);
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
  // damped_reduced_ = false;

  rot_stiffness = config_cfg.rot_stiffness;
  rot_damping   = config_cfg.rot_damping;
  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner(3, 3) << 5.0 * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) << 5.0 * Eigen::Matrix3d::Identity();

  // Initializing variables
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  velocity_d_.setZero();
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  F_ext_hat_.setZero();

  return true;
}

void PassiveDSImpedanceController::starting(const ros::Time& /*time*/) {

  // Get robot current/initial joint state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());

  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(initial_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;


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

  // // set desired point to current state
  // position_d_           = initial_transform.translation();
  // position_d_target_    = initial_transform.translation();

  // // This should be read from yaml file!!
  // orientation_d_.coeffs() << 0.9989915021247955, -0.03477723571985789, 0.02831023345326906, 0.00048384793621545193;
  // orientation_d_target_.coeffs() << 0.9989915021247955, -0.03477723571985789, 0.02831023345326906, 0.00048384793621545193;
  // q_d_nullspace_initialized_ = true;
  
  // // q_d_nullspace_left <<-0.31300278815679383, 0.5133411809408186, -0.3790919490278813, -1.5272546393582824, 0.17815104643850238, 2.054741965134938, 0.14825493200902926;
  // // q_d_nullspace_lower << 0.012401767144349107, 0.3831698678794147, 0.0014386005743168813, -2.2988653527867853, -0.0057766905587859005, 2.703733228842417, 0.8413876965092457;
  // // q_d_nullspace_right << 0.585201289471827, 0.2947303907202001, 0.050711005141697844, -1.7554985676413721, -0.028200046750456013, 2.1255105135858368, 1.4814738877195295;
  // // q_d_nullspace_back << 0.0430895791638293, -0.866528953435129, -0.02408459785052106, -2.5777155770609403, -0.015940650400556162, 1.7565315657592069, 0.8750898850364985;
  // // q_d_nullspace_home << 0.0, -0.1516284550464292, 0.0, -2.1602991589328693, 0.0063609233109487425, 2.030401506252017, 0.8428664707532099;
  // // q_d_nullspace_ << -0.05, -0.3527, 0, -2.052, 0.0749, 2.23, 0.814; // Average nullspace values
  // ROS_INFO_STREAM("Desired nullspace position (from YAML): " << std::endl << q_d_nullspace_);

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
  Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext_hat(robot_state.O_F_ext_hat_K.data());
  F_ext_hat_ << F_ext_hat; // This should be done in a more memory-efficient way
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

  // Check velocity command
  elapsed_time += period;
  if(ros::Time::now().toSec() - last_cmd_time > vel_cmd_timeout){
    velocity_d_.setZero();
    // ROS_WARN_STREAM_THROTTLE(0.5,"Command Timeout!");
    bSmooth = false;
  }else{
    bSmooth = true;
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////              COMPUTING TASK CONTROL TORQUE           //////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // compute control
  // allocate control torque variables to compute and aggregate
  Eigen::VectorXd tau_task_passive(7), tau_nullspace(7), tau_nullspace_error(7), tau_d(7), tau_tool(7);


  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //++++++++++++++ PASSIVE DS CONTROL FOR LINEAR CARTESIAN COMMAND +++++++++++++++//
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  /// set desired linear velocity
  dx_linear_des_(0)   = velocity_d_(0);
  dx_linear_des_(1)   = velocity_d_(1);
  dx_linear_des_(2)   = velocity_d_(2);

  /// set measured linear velocity
  dx_linear_msr_(0)   = velocity(0);
  dx_linear_msr_(1)   = velocity(1);
  dx_linear_msr_(2)   = velocity(2);

  ROS_WARN_STREAM_THROTTLE(0.5, "Desired Velocity:" << dx_linear_des_(0) << " " << dx_linear_des_(1) <<  " " << dx_linear_des_(2));

  // // Limit desired velocity based on workspace limit (radius 0.75 centered at robot base)
  // double gamma_workspace = position.norm() - 0.90;
  // ROS_WARN_STREAM_THROTTLE(0.5, "Gamma Workspace" << gamma_workspace);
  // Eigen::Vector3d gamma_workspace_grad;
  // gamma_workspace_grad << 1/position.norm() * position;
  // ROS_WARN_STREAM_THROTTLE(0.5, "Grad Gamma Workspace:" << gamma_workspace_grad(0) << " " << gamma_workspace_grad(1) <<  " " << gamma_workspace_grad(2));
  // if (gamma_workspace > 0){
  //   ROS_WARN_STREAM_THROTTLE(0.5, "Limit Reached" << gamma_workspace);
  //   // dx_linear_des_ *= 0.1; // Should apply workspace limit!
  // }


  // ----------------- Linear Velocity Error -> Force -----------------------//
  ROS_WARN_STREAM_THROTTLE(0.5, "Desired Velocity Norm:" << dx_linear_des_.norm());
  ROS_WARN_STREAM_THROTTLE(0.5, "Current Velocity Norm:" << dx_linear_msr_.norm());
  F_ee_des_.setZero();
  if(dx_linear_des_.norm() > 1e-6){
    passive_ds_controller->Update(dx_linear_msr_,dx_linear_des_);
    F_linear_des_ = passive_ds_controller->control_output(); // (3 x 1)
    ROS_WARN_STREAM_THROTTLE(0.5, "Damping Eigenvalues:" << damping_eigval0_ << " " << damping_eigval1_);
    ROS_WARN_STREAM_THROTTLE(0.5, "PassiveDS Velocity Control Forces:" << F_linear_des_(0) << " " << F_linear_des_(1) << " " << F_linear_des_(2));  
  }
  else{
    // Switch controller to cartesian impedance
    Eigen::Vector3d tmp_position_error;
    tmp_position_error.setZero();
    tmp_position_error << position - position_d_;
    // Computing control torque from cartesian pose error from integrated velocity command
    F_linear_des_ << -cartesian_stiffness_.topLeftCorner(3,3) * tmp_position_error - cartesian_damping_.topLeftCorner(3,3) * dx_linear_msr_;
    ROS_WARN_STREAM_THROTTLE(0.5, "Set-point Control Forces:" << F_linear_des_(0) << " " << F_linear_des_(1) << " " << F_linear_des_(2)); 
  }  
  
  F_ee_des_.head(3) = F_linear_des_;

  // ----------------- Debug -----------------------//
  ROS_WARN_STREAM_THROTTLE(0.5, "Estimated F_ext linear Norm:" << F_ext_hat_.head(3).norm());  
  ROS_WARN_STREAM_THROTTLE(0.5, "Linear Control Force:" << F_ee_des_.head(3).norm());

  // ----------------- Orientation Error -> Force -----------------------//
  // Compute task-space errors
  Eigen::VectorXd F_ee_des_ang_ ;
  F_ee_des_ang_.resize(6);
  Eigen::Vector3d orient_error;
  orient_error.setZero();

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  orient_error << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  orient_error << -transform.linear() * orient_error;

  // Computing control force from cartesian orientation error and ff damped velocity (to damp any rotational motion!)
  F_ee_des_.tail(3) << -cartesian_stiffness_.bottomRightCorner(3,3) * orient_error - cartesian_damping_.bottomRightCorner(3,3) * velocity.tail(3);

  // Convert control wrench to torque
  ROS_WARN_STREAM_THROTTLE(0.5, "Angular Control Force :" << F_ee_des_.tail(3).norm());
  tau_task_passive << jacobian.transpose() * F_ee_des_;

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //++++++++++++++ ADDITIONAL CONTROL TORQUES (NULLSPACE AND TOOL COMPENSATION) ++++++++++++++//
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


  // pseudoinverse for nullspace handling
  // kinematic pseudoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // nullspace PD control with damping ratio = 1
  ROS_WARN_STREAM_THROTTLE(0.5, "Nullspace stiffness:" << nullspace_stiffness_);
  tau_nullspace.setZero();
  Eigen::VectorXd nullspace_stiffness_vec(7);
  nullspace_stiffness_vec << nullspace_stiffness_target_, nullspace_stiffness_target_, nullspace_stiffness_target_, 
  nullspace_stiffness_target_, nullspace_stiffness_target_,nullspace_stiffness_target_,nullspace_stiffness_target_; //Same nullspace stiffness for all

  // My intents to figure out good gains!.. this could be learned...
  // nullgains << 1.,60,10.,40,5.,1.,1.; // These are optimal values for KUKA IIWA
  // double nominal_stiffness = 0.1; // This could be read from yaml file
  // These values are what was psuedo-working in the real robot
  // nullspace_stiffness_vec <<  0.05*nominal_stiffness, 0.5*nominal_stiffness, 5*nominal_stiffness, 0.15*nominal_stiffness, 0.5*nominal_stiffness, 0.01*nominal_stiffness, 0.01*nominal_stiffness;
  // nullspace_stiffness_vec <<  0.05*nominal_stiffness, 0.5*nominal_stiffness, 5*nominal_stiffness, 0.15*nominal_stiffness, 0.5*nominal_stiffness, 0.01*nominal_stiffness, 0.01*nominal_stiffness;

  for (int i=0; i<7; i++)
    tau_nullspace_error(i) = nullspace_stiffness_vec(i) * (q_d_nullspace_(i) - q(i));
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (tau_nullspace_error - (2.0 * sqrt(nullspace_stiffness_target_)) * dq);

  ROS_WARN_STREAM_THROTTLE(0.5, "Nullspace torques:" << tau_nullspace.transpose());                         
  // Compute tool compensation (scoop/camera in scooping task)
  if (activate_tool_compensation_)
    tau_tool << jacobian.transpose() * tool_compensation_force_;
  else
    tau_tool.setZero();

  // FINAL DESIRED CONTROL TORQUE SENT TO ROBOT
  // tau_d << tau_task_passive + tau_nullspace + coriolis - tau_tool + tau_ext_initial_; //Might not need the external..
  tau_d << tau_task_passive + tau_nullspace +  coriolis - tau_tool; 

  ROS_WARN_STREAM_THROTTLE(0.5, "Desired control torque:" << tau_d.transpose());

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ = cartesian_stiffness_target_ ;
  cartesian_damping_   = cartesian_damping_target_;
  nullspace_stiffness_  = nullspace_stiffness_target_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
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

  config_cfg    = config;
  activate_tool_compensation_ = config.activate_tool_compensation;  
  update_impedance_params_    = config.update_impedance_params;

  if (update_impedance_params_){
      cartesian_stiffness_target_.bottomRightCorner(3, 3)
          << config.rotational_stiffness * Eigen::Matrix3d::Identity();
      // Damping ratio = 1
      cartesian_damping_target_.bottomRightCorner(3, 3)
          << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
      nullspace_stiffness_target_ = config.nullspace_stiffness;   

      damping_eigval0_ = config.damping_eigval0;
      damping_eigval1_ = config.damping_eigval1;
      passive_ds_controller->set_damping_eigval(damping_eigval0_,damping_eigval1_);

      rot_stiffness = config.rot_stiffness;
      rot_damping   = config.rot_damping;
  }


}


void PassiveDSImpedanceController::desiredTwistCallback(
    const geometry_msgs::TwistConstPtr& msg) {

  velocity_d_      << msg->linear.x, msg->linear.y, msg->linear.z;
  last_cmd_time    = ros::Time::now().toSec();

  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());

  double dt_call = 1./1000;
  double int_gain = 200;
  position_d_target_ << position + velocity_d_*dt_call*int_gain; //Int_gain: Scaling to make it faster! (200 goes way faster than the desired

}

}  // namespace franka_interactive_controllers

PLUGINLIB_EXPORT_CLASS(franka_interactive_controllers::PassiveDSImpedanceController,
                       controller_interface::ControllerBase)

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
#include <kinematics_utils.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/filters.h>



namespace franka_interactive_controllers {
//*************************************************************************************
// PassiveDS Class taken from https://github.com/epfl-lasa/dual_iiwa_toolkit.git
//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Farshad Khadivr (maintainer)
//|    email:   farshad.khadivar@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of iiwa_toolkit.
//|
//|    iiwa_toolkit is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_toolkit is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
PassiveDS::PassiveDS(const double& lam0, const double& lam1):eigVal0(lam0),eigVal1(lam1){
    set_damping_eigval(lam0,lam1);
}

PassiveDS::~PassiveDS(){}
void PassiveDS::set_damping_eigval(const double& lam0, const double& lam1){
    if((lam0 > 0)&&(lam1 > 0)){
        eigVal0 = lam0;
        eigVal1 = lam1;
        damping_eigval(0,0) = eigVal0;
        damping_eigval(1,1) = eigVal1;
        damping_eigval(2,2) = eigVal1;
    }else{
        std::cerr << "wrong values for the eigenvalues"<<"\n";
    }
}
void PassiveDS::updateDampingMatrix(const Eigen::Vector3d& ref_vel){ 

    if(ref_vel.norm() > 1e-6){
        baseMat.setRandom();
        baseMat.col(0) = ref_vel.normalized();
        for(uint i=1;i<3;i++){
            for(uint j=0;j<i;j++)
                baseMat.col(i) -= baseMat.col(j).dot(baseMat.col(i))*baseMat.col(j);
            baseMat.col(i).normalize();
        }
        Dmat = baseMat*damping_eigval*baseMat.transpose();
    }else{
        Dmat = Eigen::Matrix3d::Identity();
    }
    // otherwise just use the last computed basis
}

void PassiveDS::update(const Eigen::Vector3d& vel, const Eigen::Vector3d& des_vel){
    // compute damping
    updateDampingMatrix(des_vel);
    // dissipate
    control_output = - Dmat * vel;
    // compute control
    control_output += eigVal0*des_vel;
}
Eigen::Vector3d PassiveDS::get_output(){ return control_output;}

//*************************************************************************************

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
  

  // Initialize classical impedance stiffness stiffness matrices
  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_setpoint_ctrl_.setIdentity();
  cartesian_stiffness_grav_comp_.setIdentity(); 

  std::vector<double> cartesian_stiffness_setpoint_ctrl_yaml;
  if (!node_handle.getParam("cartesian_stiffness_setpoint_ctrl", cartesian_stiffness_setpoint_ctrl_yaml) || cartesian_stiffness_setpoint_ctrl_yaml.size() != 6) {
    ROS_ERROR(
      "PassiveDSImpedanceController: Invalid or no cartesian_stiffness_setpoint_ctrl parameters provided, "
      "aborting controller init!");
    return false;
  }
  for (int i = 0; i < 6; i ++)
    cartesian_stiffness_setpoint_ctrl_(i,i) = cartesian_stiffness_setpoint_ctrl_yaml[i];

  std::vector<double> cartesian_stiffness_grav_comp_yaml;
  if (!node_handle.getParam("cartesian_stiffness_grav_comp", cartesian_stiffness_grav_comp_yaml) || cartesian_stiffness_grav_comp_yaml.size() != 6) {
    ROS_ERROR(
      "PassiveDSImpedanceController: Invalid or no cartesian_stiffness_grav_comp parameters provided, "
      "aborting controller init!");
    return false;
  }
  for (int i = 0; i < 6; i ++)
    cartesian_stiffness_grav_comp_(i,i) = cartesian_stiffness_grav_comp_yaml[i];
  
  // Set the initial cartesian stiffness with grav comp values!
  cartesian_stiffness_ << cartesian_stiffness_grav_comp_;


  // Damping ratio = 1
  cartesian_damping_.setIdentity();
  default_cart_stiffness_target_ << 300, 300, 300, 50, 50, 50;
  for (int i = 0; i < 6; i ++) {
    if (cartesian_stiffness_(i) == 0.0)
      cartesian_damping_(i,i) = 2.0 * sqrt(default_cart_stiffness_target_(i));
    else
      cartesian_damping_(i,i) = 2.0 * sqrt(cartesian_stiffness_(i));
  }
  ROS_INFO_STREAM("cartesian_stiffness_: " <<  cartesian_stiffness_.transpose());
  ROS_INFO_STREAM("cartesian_damping_: " << cartesian_damping_.transpose());

  // Initialize PassiveDS params
  damping_eigvals_yaml_.setZero();
  std::vector<double> damping_eigvals;
  if (node_handle.getParam("linear_damping_eigenvalues", damping_eigvals)) {
    if (damping_eigvals.size() != 3) {
      ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no linear_damping_eigenvalues parameters provided, "
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
  passive_ds_controller = std::make_unique<PassiveDS>(100., 100.);
  passive_ds_controller->set_damping_eigval(damping_eigval0_,damping_eigval1_);


  //**** Initialize ANGULAR PassiveDS params ****//
  ang_damping_eigvals_yaml_.setZero();
  std::vector<double> ang_damping_eigvals;
  if (node_handle.getParam("angular_damping_eigenvalues", damping_eigvals)) {
    if (damping_eigvals.size() != 3) {
      ROS_ERROR(
        "PassiveDSImpedanceController: Invalid or no angular_damping_eigenvalues parameters provided, "
        "aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 3; ++i) 
      ang_damping_eigvals_yaml_[i] = damping_eigvals.at(i);
    ROS_INFO_STREAM("Angular Damping Matrix Eigenvalues (from YAML): " << ang_damping_eigvals_yaml_);
  }
  // Initialize Passive DS controller
  ang_damping_eigval0_ = ang_damping_eigvals_yaml_(0);
  ang_damping_eigval1_ = ang_damping_eigvals_yaml_(1);
  ang_passive_ds_controller = std::make_unique<PassiveDS>(5., 5.);
  ang_passive_ds_controller->set_damping_eigval(ang_damping_eigval0_,ang_damping_eigval1_);


  // Initialize nullspace params
  if (!node_handle.getParam("nullspace_stiffness", nullspace_stiffness_target_) || nullspace_stiffness_target_ <= 0) {
    ROS_ERROR(
      "PassiveDSImpedanceController: Invalid or no nullspace_stiffness parameters provided, "
      "aborting controller init!");
    return false;
  }
  ROS_INFO_STREAM("nullspace_stiffness_target_: " << std::endl <<  nullspace_stiffness_target_);

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

  // set desired point to current state (THIS SHOULD BE READY FROM YAML FILE!!!)
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
  // dx_linear_des_(0)   = velocity_d_(0);
  // dx_linear_des_(1)   = velocity_d_(1);
  // dx_linear_des_(2)   = velocity_d_(2);
  dx_linear_des_ << velocity_d_;

  /// set measured linear velocity
  // dx_linear_msr_(0)   = velocity(0);
  // dx_linear_msr_(1)   = velocity(1);
  // dx_linear_msr_(2)   = velocity(2);

  dx_linear_msr_ << velocity.head(3);
  dx_angular_msr_ << velocity.tail(3); 

  ROS_WARN_STREAM_THROTTLE(0.5, "Desired Velocity:" << dx_linear_des_(0) << " " << dx_linear_des_(1) <<  " " << dx_linear_des_(2));

  // ------------------------------------------------------------------------//
  // ----------------- Linear Velocity Error -> Force -----------------------//
  // ------------------------------------------------------------------------//
  ROS_WARN_STREAM_THROTTLE(0.5, "Desired Velocity Norm:" << dx_linear_des_.norm());
  ROS_WARN_STREAM_THROTTLE(0.5, "Current Velocity Norm:" << dx_linear_msr_.norm());
  F_ee_des_.setZero();

  if(dx_linear_des_.norm() > 1e-6){
  
    // Passive DS Impedance Contoller for Linear Velocity Error
    passive_ds_controller->update(dx_linear_msr_,dx_linear_des_);
    F_linear_des_ << passive_ds_controller->get_output(); 

    ROS_WARN_STREAM_THROTTLE(0.5, "Damping Eigenvalues:" << damping_eigval0_ << " " << damping_eigval1_);
    ROS_WARN_STREAM_THROTTLE(0.5, "PassiveDS Velocity Control Forces:" << F_linear_des_(0) << " " << F_linear_des_(1) << " " << F_linear_des_(2));  

  }
  else{
    // Switch controller to cartesian impedance (Do this in a better way, should read target and switch to setpoint control)
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

  // ------------------------------------------------------------------------//
  // ----------------- Orientation Error -> Force ---------------------------//
  // ------------------------------------------------------------------------//
  if (bPassiveOrient_ && dx_linear_des_.norm() > 1e-6){
   
    //***** Using PassiveDS control law for angular velocity trackin given desired quaternion_d_ 

    // Converting quaternion representations
    Eigen::Vector4d _ee_quat; _ee_quat.setZero();
    _ee_quat[0] = orientation.w(); _ee_quat.segment(1,3) = orientation.vec();
    Eigen::Vector4d _ee_des_quat; _ee_des_quat.setZero();
    _ee_des_quat[0] = orientation_d_.w(); _ee_des_quat.segment(1,3) = orientation_d_.vec();

    // Computing desired Angular Velocity from desired "fixed" quaternion
    Eigen::Vector4d dqd = KinematicsUtils<double>::slerpQuaternion(_ee_quat, _ee_des_quat, 0.5);    
    Eigen::Vector4d deltaQ = dqd - _ee_quat;

    Eigen::Vector4d qconj = _ee_quat;
    qconj.segment(1,3) = -1 * qconj.segment(1,3);
    Eigen::Vector4d temp_angVel = KinematicsUtils<double>::quaternionProduct(deltaQ, qconj);

    Eigen::Vector3d tmp_angular_vel = temp_angVel.segment(1,3);
    double maxDq(0.2), dsGain_ori (3.0);
    if (tmp_angular_vel.norm() > maxDq)
        tmp_angular_vel = maxDq * tmp_angular_vel.normalized();

    double theta_gq = (-.5/(4*maxDq*maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
    dx_angular_des_  = 2 * dsGain_ori*(1+std::exp(theta_gq)) * tmp_angular_vel;

    // Passive DS Impedance Contoller for Angular Velocity Error
    ang_passive_ds_controller->update(dx_angular_msr_,dx_angular_des_);
    F_angular_des_ << ang_passive_ds_controller->get_output(); 

    // ----------------- Debug -----------------------//
    ROS_WARN_STREAM_THROTTLE(0.5, "Desired Angular Velocity Norm:" << dx_angular_des_.norm());
    ROS_WARN_STREAM_THROTTLE(0.5, "Current Angular Velocity Norm:" << dx_angular_msr_.norm());


  }else{

    //***** Using Classic Orientation Impedance control with FF Damped vel to track desired quaternion_d_ 

    // Change stiffness values to setpoint control from yaml
    cartesian_stiffness_ << cartesian_stiffness_setpoint_ctrl_;
    ROS_WARN_STREAM_THROTTLE(0.5, "Cartesian Stiffness:" << cartesian_stiffness_(1,1) << " " << cartesian_stiffness_(2,2) << " " << cartesian_stiffness_(3,3));
 
    // Computing orientation error
    // Eigen::Vector3d orient_error;
    orient_error.setZero();
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    orient_error << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

    // Transform to base frame
    orient_error << -transform.linear() * orient_error;

    // Computing control force from cartesian orientation error and ff damped velocity (to damp any rotational motion!)
    F_angular_des_ << -cartesian_stiffness_.bottomRightCorner(3,3) * orient_error - cartesian_damping_.bottomRightCorner(3,3) * dx_angular_msr_;
  }

  // Feed angular component control wrench 
  F_ee_des_.tail(3) << F_angular_des_;
  ROS_WARN_STREAM_THROTTLE(0.5, "Angular Control Force :" << F_ee_des_.tail(3).norm());
  
  // Convert full control wrench to torque
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
  nullspace_stiffness_  = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_)*nullspace_stiffness_;
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
  bPassiveOrient_             = config.activate_angular_passiveDS;  
  update_impedance_params_    = config.update_impedance_params;

  if (update_impedance_params_){
      cartesian_stiffness_.bottomRightCorner(3, 3)
          << config.rotational_stiffness * Eigen::Matrix3d::Identity();
      // Damping ratio = 1
      cartesian_damping_.bottomRightCorner(3, 3)
          << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
      nullspace_stiffness_target_ = config.nullspace_stiffness;   

      damping_eigval0_ = config.damping_eigval0;
      damping_eigval1_ = config.damping_eigval1;
      passive_ds_controller->set_damping_eigval(damping_eigval0_,damping_eigval1_);

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

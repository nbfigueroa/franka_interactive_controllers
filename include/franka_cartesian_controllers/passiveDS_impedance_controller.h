// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka_interactive_controllers/passive_ds_paramConfig.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Eigen>
#include <dynamic_reconfigure/server.h>

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
class PassiveDS
{
private:
    double eigVal0;
    double eigVal1;
    double desired_damping;
    Eigen::Matrix3d damping_eigval = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d baseMat = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Dmat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d control_output = Eigen::Vector3d::Zero();
    void updateDampingMatrix(const Eigen::Vector3d& ref_vel);
public:
    PassiveDS(const double& lam0, const double& lam1);
    ~PassiveDS();
    void set_damping_eigval(const double& lam0, const double& lam1);
    void update(const Eigen::Vector3d& vel, const Eigen::Vector3d& des_vel);
    Eigen::Vector3d get_output();
};
///////////////////////////////////////////////////////////////////////////////////



class PassiveDSImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double dt_ = 0.001;
  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};

  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_setpoint_ctrl_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_grav_comp_;  
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 6, 1> F_ext_hat_;
  Eigen::Matrix<double, 3, 1> damping_eigvals_yaml_; 
  Eigen::Matrix<double, 3, 1> ang_damping_eigvals_yaml_;
  
  // whether to load from yaml or use initial robot config
  bool q_d_nullspace_initialized_ = false;
  
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Vector3d velocity_d_;
  Eigen::Vector3d velocity_;

  // Timing
  ros::Duration elapsed_time;
  double last_cmd_time;
  double last_msg_time;
  double vel_cmd_timeout;

  Eigen::Matrix<double, 6, 1> tool_compensation_force_;
  bool activate_tool_compensation_;
  bool update_impedance_params_;
  bool bPassiveOrient_;

  // For external torque and gravity compensation
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;

  // Initialize DS controller
  Eigen::Vector3d     dx_linear_des_;
  Eigen::Vector3d     dx_linear_msr_;
  Eigen::Vector3d     dx_angular_des_;
  Eigen::Vector3d     dx_angular_msr_;
  
  Eigen::Vector3d     F_linear_des_;     // desired linear force 
  Eigen::Vector3d     F_angular_des_;    // desired angular force
  Eigen::VectorXd     F_ee_des_;         // desired end-effector force
  Eigen::Vector3d     orient_error;


  double              damping_eigval0_;
  double              damping_eigval1_;
  double              real_damping_eigval0_;
  double              real_damping_eigval1_;
  double              ang_damping_eigval0_;
  double              ang_damping_eigval1_;
 
  // UNUSED SHOULD CLEAN UP!
  bool                bVelCommand;
  bool                bDebug;
  double              smooth_val_;
  double              rot_stiffness;
  double              rot_damping;
  Eigen::Matrix<double, 6, 1> default_cart_stiffness_target_;
  int                 cartesian_stiffness_mode_; // 0: grav-comp, 1: setpoint-track (NOT USED ANYMORE)

  // Instantiate DS controller class
  std::unique_ptr<PassiveDS> passive_ds_controller;
  std::unique_ptr<PassiveDS> ang_passive_ds_controller;
  double                     desired_damp_eigval_cb_;
  double                     desired_damp_eigval_cb_prev_;
  bool                       new_damping_msg_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_interactive_controllers::passive_ds_paramConfig>>
      dynamic_server_passive_ds_param_;
  ros::NodeHandle dynamic_reconfigure_passive_ds_param_node_;
  void passiveDSParamCallback(franka_interactive_controllers::passive_ds_paramConfig& config,
                               uint32_t level);

  franka_interactive_controllers::passive_ds_paramConfig config_cfg;

  // Desired twist subscriber (To take in desired DS velocity)
  ros::Subscriber sub_desired_twist_;
  ros::Subscriber sub_desired_damping_;
  void desiredTwistCallback(const geometry_msgs::TwistConstPtr& msg);
  void desiredDampingCallback(const std_msgs::Float32Ptr& msg); // In case damping values want to be changed!

};

}  // namespace franka_interactive_controllers

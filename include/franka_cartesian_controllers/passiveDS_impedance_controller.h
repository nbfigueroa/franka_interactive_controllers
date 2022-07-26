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
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka_interactive_controllers/passive_ds_paramConfig.h>
// #include <passive_ds_controller.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Eigen>
#include <dynamic_reconfigure/server.h>

namespace franka_interactive_controllers {


class PassiveDS
{
private:
    double eigVal0;
    double eigVal1;
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
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 6, 1> F_ext_hat_;
  Eigen::Matrix<double, 3, 1> damping_eigvals_yaml_;
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
  double vel_cmd_timeout;


  // Variables for initialization and tool compensation
  // bool _goto_home;
  // double jointDS_epsilon_;
  // double dq_filter_params_;
  // Eigen::Matrix<double, 7, 1> q_home_;
  // Eigen::Matrix<double, 7, 7> A_jointDS_home_;
  // Eigen::Matrix<double, 7, 7> k_joint_gains_;
  // Eigen::Matrix<double, 7, 7> d_joint_gains_;
  // Eigen::Matrix<double, 7, 7> d_ff_joint_gains_;
  Eigen::Matrix<double, 6, 1> tool_compensation_force_;
  bool activate_tool_compensation_;
  bool update_impedance_params_;

  // For external torque and gravity compensation
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;

  // Initialize DS controller
  Eigen::Vector3d     dx_linear_des_;
  Eigen::Vector3d     dx_linear_msr_;
  Eigen::Vector3d     F_linear_des_;     // desired linear force 
  Eigen::Vector3d     F_angular_des_;     // desired angular force
  Eigen::VectorXd     F_ee_des_;         // desired end-effector force
  Eigen::Vector3d     orient_error;
  bool                bDebug;
  bool                bSmooth;
  double              smooth_val_;
  double              rot_stiffness;
  double              rot_damping;
  double            damping_eigval0_;
  double            damping_eigval1_;
  Eigen::Matrix<double, 6, 1> default_cart_stiffness_target_;


  // Instantiate DS controller class
  // boost::scoped_ptr<DSController>   passive_ds_controller;
  std::unique_ptr<PassiveDS> passive_ds_controller;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_interactive_controllers::passive_ds_paramConfig>>
      dynamic_server_passive_ds_param_;
  ros::NodeHandle dynamic_reconfigure_passive_ds_param_node_;
  void passiveDSParamCallback(franka_interactive_controllers::passive_ds_paramConfig& config,
                               uint32_t level);

  franka_interactive_controllers::passive_ds_paramConfig config_cfg;

  // Desired twist subscriber (To take in desired DS velocity)
  ros::Subscriber sub_desired_twist_;
  void desiredTwistCallback(const geometry_msgs::TwistConstPtr& msg);
};

}  // namespace franka_interactive_controllers

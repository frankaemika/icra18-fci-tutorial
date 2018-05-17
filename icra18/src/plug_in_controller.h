// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <icra18/plug_in_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace icra18_controllers {

class PlugInController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Force control PI
  double desired_force_{0.0};
  double target_force_{0.0};
  double k_p_{0.0};
  double k_i_{0.0};
  Eigen::Matrix<double, 6, 1> force_ext_initial_;
  Eigen::Matrix<double, 6, 1> force_error_;
  static constexpr double kDeltaTauMax{1.0};

  // Wiggle motions
  double time_{0.0};
  double wiggle_frequency_x_{0.5};
  double wiggle_frequency_y_{0.5};
  double amplitude_wiggle_x_{0.1};
  double amplitude_wiggle_y_{0.1};

  // Cartesian impedance
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

  // Dynamic reconfigure
  double target_k_p_{0.0};
  double target_k_i_{0.0};
  double wiggle_frequency_x_target_{0.5};
  double wiggle_frequency_y_target_{0.5};
  double amplitude_wiggle_x_target_{0.1};
  double amplitude_wiggle_y_target_{0.1};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  double filter_params_{0.005};
  void updateDynamicReconfigure();

  std::unique_ptr<dynamic_reconfigure::Server<icra18::plug_in_paramConfig>>
      dynamic_server_plug_in_param_;
  ros::NodeHandle dynamic_reconfigure_plug_in_param_node_;
  void plugInParamCallback(icra18::plug_in_paramConfig& config,
                               uint32_t level);
};

}  // namespace icra18_controllers

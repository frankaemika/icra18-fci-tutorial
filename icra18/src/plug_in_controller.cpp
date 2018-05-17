// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "plug_in_controller.h"

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace icra18_controllers {

bool PlugInController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "PlugInController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("PlugInController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "PlugInController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("PlugInController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PlugInController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("PlugInController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PlugInController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("PlugInController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("PlugInController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_plug_in_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_plug_in_param_node");
  dynamic_server_plug_in_param_.reset(
      new dynamic_reconfigure::Server<icra18::plug_in_paramConfig>(
          dynamic_reconfigure_plug_in_param_node_));
  dynamic_server_plug_in_param_->setCallback(
      boost::bind(&PlugInController::plugInParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void PlugInController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 6, 1> > force_ext(robot_state.O_F_ext_hat_K.data());
  // Bias correction for the current external torque
  force_ext_initial_ = force_ext;
  force_error_.setZero();
  time_ = 0.0;

  // convert to eigen
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
}

void PlugInController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  time_ += period.toSec();
  franka::RobotState robot_state = state_handle_->getRobotState();

  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1> > force_ext(robot_state.O_F_ext_hat_K.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  Eigen::VectorXd desired_force_torque(6), tau_force(7),
      tau_task(7), tau_cartesian_impedance(7), tau_cmd(7), force_control(6);

  // FORCE CONTROL
  desired_force_torque.setZero();
  desired_force_torque(2) = -desired_force_;
  force_error_ = force_error_ + period.toSec() * (desired_force_torque - force_ext + force_ext_initial_);
  force_control = (desired_force_torque + k_p_ * (desired_force_torque - force_ext + force_ext_initial_) +
                                        k_i_ * force_error_);
  force_control << 0, 0, force_control(2), 0, 0, 0;
  // FF + PI control
  tau_force = jacobian.transpose() * force_control;

  ROS_INFO_STREAM_THROTTLE(1.0, "force_error:" << force_error_.transpose());

  // WIGGLE MOTION
  Eigen::AngleAxisd angle_axis_wiggle_x;
  angle_axis_wiggle_x.axis() << 1, 0, 0;
  angle_axis_wiggle_x.angle() = sin(2.0 * M_PI * time_ * wiggle_frequency_x_) * amplitude_wiggle_x_;
  Eigen::AngleAxisd angle_axis_wiggle_y;
  angle_axis_wiggle_y.axis() << 0, 1, 0;
  angle_axis_wiggle_y.angle() = sin(2.0 * M_PI * time_ * wiggle_frequency_y_) * amplitude_wiggle_y_;

  Eigen::Quaterniond wiggle_x(angle_axis_wiggle_x);
  Eigen::Quaterniond wiggle_y(angle_axis_wiggle_y);
  Eigen::Quaterniond orientation_d(wiggle_y*(wiggle_x*orientation_d_));

  // CARTESIAN IMPEDANCE
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // Cartesian PD control
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // Desired torque
  tau_cartesian_impedance << tau_task + coriolis;


  tau_cmd << tau_cartesian_impedance + tau_force;
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }
  updateDynamicReconfigure();
}


void PlugInController::updateDynamicReconfigure() {
  // Update signals changed online through dynamic reconfigure
  desired_force_ = filter_params_ * target_force_ + (1 - filter_params_) * desired_force_;
  k_p_ = filter_params_ * target_k_p_ + (1 - filter_params_) * k_p_;
  k_i_ = filter_params_ * target_k_i_ + (1 - filter_params_) * k_i_;
  wiggle_frequency_x_ = filter_params_ * wiggle_frequency_x_target_ + (1 - filter_params_) * wiggle_frequency_x_;
  wiggle_frequency_y_ = filter_params_ * wiggle_frequency_y_target_ + (1 - filter_params_) * wiggle_frequency_y_;
  amplitude_wiggle_x_ = filter_params_ * amplitude_wiggle_x_target_ + (1 - filter_params_) * amplitude_wiggle_x_;
  amplitude_wiggle_y_ = filter_params_ * amplitude_wiggle_y_target_ + (1 - filter_params_) * amplitude_wiggle_y_;
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
}

void PlugInController::plugInParamCallback(
    icra18::plug_in_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  cartesian_stiffness_target_(2,2) = 0.0;
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_(2,2) = 0.0;
  target_force_ = config.desired_force;
  target_k_p_ = config.k_p;
  target_k_i_ = config.k_i;
  wiggle_frequency_x_target_ = config.frequency_wiggle_x;
  wiggle_frequency_y_target_ = config.frequency_wiggle_y;
  amplitude_wiggle_x_target_ = config.amplitude_wiggle_x;
  amplitude_wiggle_y_target_ = config.amplitude_wiggle_y;
}

}   // namespace icra18_controllers

PLUGINLIB_EXPORT_CLASS(icra18_controllers::PlugInController,
                       controller_interface::ControllerBase)

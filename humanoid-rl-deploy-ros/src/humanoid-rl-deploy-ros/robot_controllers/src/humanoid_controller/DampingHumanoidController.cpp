// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "humanoid_controller/DampingHumanoidController.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32MultiArray.h>
#include <numeric>

namespace controller {

// Initialize the controller
bool DampingHumanoidController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
  if (!loadDampingCfg()) {
    ROS_ERROR("Failed to load damping configuration");
    return false;
  }
  return ControllerBase::init(robot_hw, nh);
}

// Perform initialization when the controller starts
void DampingHumanoidController::starting(const ros::Time &time) {
  loop_count = 0;
}

bool DampingHumanoidController::loadDampingCfg() {
  // Load the configuration for the stand
  ros::NodeHandle nh_global;
  nh_global.getParam("/damping_kd", damping_kd_);
  return true;
}

// Update function called periodically
void DampingHumanoidController::update(const ros::Time &time, const ros::Duration &period) {
  for (size_t i = 0; i < num_of_actuation(); i++) {
    SingleJointCtrl(i, 0, 0, 0, damping_kd_[i], 0, 0);
  }

  loop_count++;
}

} // namespace

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(controller::DampingHumanoidController, controller_interface::ControllerBase)
// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "humanoid_controller/StandHumanoidController.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <numeric>

namespace controller {

// Initialize the controller
bool StandHumanoidController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
  loop_count = 0;
  if (initJointAngles_.size() == 0) {
    initJointAngles_.resize(num_of_actuation());
  }
  loadStandCfg();
  // Initialize the stand position
  return controller::ControllerBase::init(robot_hw, nh);
}

// Perform initialization when the controller starts
void StandHumanoidController::starting(const ros::Time &time) {
  loop_count = 0;
  controller::ControllerBase::starting(time);
  for (size_t i = 0; i < num_of_actuation(); i++) {
    initJointAngles_[i] = joint_pos()[i];
    ROS_INFO_STREAM("Initialized joint " << i << " to position " << initJointAngles_[i]);
  }
  ROS_INFO("StandHumanoidController starting");
}

bool StandHumanoidController::loadStandCfg() {
  // Load the configuration for the stand
  ROS_INFO_STREAM("Loading stand configuration");
  ik_stand_kd_.resize(31);
  ik_stand_kp_.resize(31);
  ik_stand_desire_pos_.resize(31);
  ik_standDesirejoints_.resize(31);
  ros::NodeHandle nh_global;
  nh_global.getParam("/stand_kd", ik_stand_kd_);
  nh_global.getParam("/stand_kp", ik_stand_kp_);
  nh_global.getParam("/stand_Pos", ik_stand_desire_pos_);
  ik_standDesirejoints_.setZero(31);
  for (size_t i = 0; i < ik_stand_desire_pos_.size(); i++) {
      ik_standDesirejoints_[i] = ik_stand_desire_pos_[i];
  }
  for (size_t i = 0; i < ik_stand_kp_.size(); i++) {
    ROS_INFO("ik_stand_kp_[%zu]: %f", i, ik_stand_kp_[i]);
  }
  for (size_t i = 0; i < ik_stand_kd_.size(); i++) {
    ROS_INFO("ik_stand_kd_[%zu]: %f", i, ik_stand_kd_[i]);
  }
  for (size_t i = 0; i < ik_stand_desire_pos_.size(); i++) {
    ROS_INFO("ik_stand_desire_pos_[%zu]: %f", i, ik_stand_desire_pos_[i]);
  }
  return true;
}

// Update function called periodically
void StandHumanoidController::update(const ros::Time &time, const ros::Duration &period) {
  controller::ControllerBase::update(time, period);
  if (loop_count < totalLoopCount_) {
    ik_standPercent_ = loop_count / static_cast<double>(totalLoopCount_);
  } else {
    ik_standPercent_ = 1.0;
  }

  for (size_t i = 0; i < num_of_actuation(); i++) {
    scalar_t pos_des = initJointAngles_[i] * (1 - ik_standPercent_) + ik_standDesirejoints_[i] * ik_standPercent_;
    SingleJointCtrl(i, pos_des, 0, ik_stand_kp_[i], ik_stand_kd_[i], 0, 0);
  }
}

} // namespace

// Export the class as a plugin.
PLUGINLIB_EXPORT_CLASS(controller::StandHumanoidController, controller_interface::ControllerBase)
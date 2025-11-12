// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_HUMANOID_CONTROLLER_H_
#define _LIMX_HUMANOID_CONTROLLER_H_

#include "ros/ros.h"
#include "humanoid_controller/ControllerBase.h"
#include "limxsdk/humanoid.h"

namespace controller {
// Class for controlling a biped robot with point foot
class StandHumanoidController : public ControllerBase {
  using tensor_element_t = float; // Type alias for tensor elements
  using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>; // Type alias for matrices

public:
  StandHumanoidController(){
    // Constructor implementation
    ROS_INFO("StandHumanoidController constructor");
  } // Default constructor

  ~StandHumanoidController() {
    ROS_INFO("StandHumanoidController destructor");
  } // Destructor

  // Initialize the controller
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;

  // Perform actions when the controller starts
  void starting(const ros::Time &time) override;

  // Update the controller
  void update(const ros::Time &time, const ros::Duration &period) override;

protected:

  // Load Stand parameters
  bool loadStandCfg();

private:
  std::vector<scalar_t> ik_stand_kp_, ik_stand_kd_, ik_stand_desire_pos_;
  vector_t ik_standDesirejoints_;
  int64_t totalLoopCount_{2000};
  double ik_standPercent_{0.0};
  std::vector<scalar_t> initJointAngles_;
};

} // namespace controller

#endif //_LIMX_HUMANOID_CONTROLLER_H_

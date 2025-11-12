// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#include "ros/ros.h"
#include "robot_hw/RobotHWLoop.h"
#include "robot_hw/HumanoidHW.h"
#include "limxsdk/humanoid.h"

// Main function
int main(int argc, char **argv) {
  ros::init(argc, argv, "humanoid_hw");

  std::string robot_ip = "127.0.0.1"; // Default robot IP address
  if (argc > 1) {
    robot_ip = argv[1]; // Use command-line argument as robot IP address if provided
  }

  // Initialize Humanoid instance with robot IP
  if (!limxsdk::Humanoid::getInstance()->init(robot_ip)) {
    ROS_ERROR("Failed to connect to the robot: %s", robot_ip.c_str());
    abort();
  } else {
    ROS_WARN("Connect to the robot: %s", robot_ip.c_str());
  }

  try {
    ros::NodeHandle nh;
    ros::NodeHandle robot_hw_nh("~");

    // Create and initialize HumanoidHW instance
    std::shared_ptr<hw::HumanoidHW> hw = std::make_shared<hw::HumanoidHW>();
    hw->init(nh, robot_hw_nh);

    // Create and initialize RobotHWLoop instance
    hw::RobotHWLoop loop(nh, robot_hw_nh, hw);
    loop.StartControlLoop(robot_hw_nh);
    // Start ROS event loop
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR("Error in the hardware interface: %s", e.what());
    return 1;
  }

  return 0;
}

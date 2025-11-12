// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#pragma once

#include <ros/ros.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <robot_common/hardware_interface/ContactSensorInterface.h>
#include <robot_common/hardware_interface/HybridJointInterface.h>
#include <robot_common/types.h>
#include <std_msgs/Float32MultiArray.h>
#include <onnxruntime_cxx_api.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>

namespace controller {

using scalar_t = double;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;

struct IMUData {
  scalar_t time_stamp{0.0};
  quaternion_t quat{quaternion_t::Identity()};
  vector3_t angular_vel_local{vector3_t::Zero()};
  vector3_t linear_acc_local{vector3_t::Zero()};
  matrix3_t orientation_cov{matrix3_t::Identity()};
  matrix3_t angular_vel_cov{matrix3_t::Identity()};
  matrix3_t linear_acc_cov{matrix3_t::Identity()};
};
struct RLRobotCfg {
    struct HumanoidControlCfg {
        int actionsSize;
        int observationSize;
        int obsHistoryLength;
        int decimation;
        int l_encoder_output_size_;
        int p_encoder_output_size_;
        int c_encoder_output_size_;
        std::vector<scalar_t> user_torque_limit;
        std::vector<scalar_t> action_scale_vec, stiffness_vec, damping_vec;
        std::vector<int> encoder_output_size;
        scalar_t clipActions;
        scalar_t clipObs;
        void print() {
            ROS_INFO("========start ControlCfg========");
            ROS_INFO("actionsSize: %d", actionsSize);
            ROS_INFO("observationSize: %d", observationSize);
            ROS_INFO("obsHistoryLength: %d", obsHistoryLength);
            ROS_INFO("decimation: %d", decimation);
            ROS_INFO("user_torque_limit: %s", user_torque_limit.size() ? "loaded" : "not loaded");
            ROS_INFO("action_scale_vec: %s", action_scale_vec.size() ? "loaded" : "not loaded");
            ROS_INFO("stiffness_vec: %s", stiffness_vec.size() ? "loaded" : "not loaded");
            ROS_INFO("damping_vec: %s", damping_vec.size() ? "loaded" : "not loaded");
            ROS_INFO("clipActions: %f", clipActions);
            ROS_INFO("clipObs: %f", clipObs);
            ROS_INFO("encoder_output_size: %s", encoder_output_size.size() ? "loaded" : "not loaded");
            ROS_INFO("l_encoder_output_size: %d", l_encoder_output_size_);
            ROS_INFO("p_encoder_output_size: %d", p_encoder_output_size_);
            ROS_INFO("c_encoder_output_size: %d", c_encoder_output_size_);
            ROS_INFO("========end ControlCfg========");
        }
    };

    struct HumanoidObsScales {
        scalar_t linVel;
        scalar_t angVel;
        scalar_t dofPos;
        scalar_t dofVel;
        scalar_t baseHeight;
        void print() {
            ROS_INFO("=======start ObsScales========");
            ROS_INFO("linVel: %f", linVel);
            ROS_INFO("angVel: %f", angVel);
            ROS_INFO("dofPos: %f", dofPos);
            ROS_INFO("dofVel: %f", dofVel);
            ROS_INFO("baseHeight: %f", baseHeight);
            ROS_INFO("=======end ObsScales========");
        }
    };

    HumanoidObsScales humanoidObsScales;
    HumanoidControlCfg humanoidControlCfg;
};

class ControllerBase
  : public controller_interface::MultiInterfaceController<robot_common::HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                            robot_common::ContactSensorInterface> {
   public:
    ControllerBase();
    ~ControllerBase() override;

    // Initialize the controller
    virtual bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void stopping(const ros::Time& /*time*/) override;
    ros::NodeHandle nh_global;

   protected:
    void UpdateIMU();
    void ZeroTorque();
    void UpdateJointState();
    virtual void SetupHardwareHandle(hardware_interface::RobotHW* robot_hw);
    void SingleJointCtrl(const int joint_index,
                         const double q_des,
                         const double dq_des,
                         const double Kp,
                         const double Kd,
                         const double tau_ff,
                         const uint8_t mode);
    void MultiJointCtrl(const vector_t& q_des,
                        const vector_t& dq_des,
                        const vector_t& Kp,
                        const vector_t& Kd,
                        const vector_t& tau_ff,
                        const std::vector<uint8_t>& mode);
    void SetControllerState(const int& state, const std::string& description);
    inline int num_of_actuation() const { return 31; }
    inline vector_t joint_pos() const { return joint_pos_; }
    inline vector_t joint_vel() const { return joint_vel_; }
    inline vector_t joint_tau() const { return joint_tau_; }
    inline IMUData imu_data() const { return imu_data_; }

    RLRobotCfg robotCfg{};
    int64_t loop_count{0};
    std::string controller_name;
    std::string rbtSN, robotType_;
    std::vector<std::string> joint_names_;

   private:
    std::vector<robot_common::HybridJointHandle> hybrid_joint_handles_;
    hardware_interface::ImuSensorHandle imu_sensor_handle_;
    vector_t joint_pos_, joint_vel_, joint_tau_;
    IMUData imu_data_;
};
}  // namespace controller

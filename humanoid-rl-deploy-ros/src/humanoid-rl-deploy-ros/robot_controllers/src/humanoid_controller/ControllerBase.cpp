#include "humanoid_controller/ControllerBase.h"
#include "pluginlib/class_list_macros.hpp"
#include <std_msgs/Float32MultiArray.h>

namespace controller {

ControllerBase::ControllerBase() : controller_name("BaseController") {
    joint_pos_ = vector_t::Zero(num_of_actuation());
    joint_vel_ = vector_t::Zero(num_of_actuation());
    joint_tau_ = vector_t::Zero(num_of_actuation());
}

ControllerBase::~ControllerBase() {}

bool ControllerBase::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
    ROS_INFO("ControllerBase::init()");
    /** Init Hardware Interface **/
    SetupHardwareHandle(robot_hw);
    return true;
}

void ControllerBase::starting(const ros::Time& time) {
    UpdateJointState();
    UpdateIMU();
}

void ControllerBase::stopping(const ros::Time& time) {
    ZeroTorque();
}

void ControllerBase::update(const ros::Time& time, const ros::Duration& period) {
    UpdateJointState();
    UpdateIMU();

    loop_count++;
}

void ControllerBase::UpdateIMU() {
    // quaternion
    for (int i = 0; i < 4; ++i) {
        imu_data_.quat.coeffs()(i) = imu_sensor_handle_.getOrientation()[i];
    }
    // angular velocity & acceleration
    for (int i = 0; i < 3; ++i) {
        imu_data_.angular_vel_local(i) = imu_sensor_handle_.getAngularVelocity()[i];
        imu_data_.linear_acc_local(i) = imu_sensor_handle_.getLinearAcceleration()[i];
    }

    /** estimation parameters **/
    for (size_t i = 0; i < 9; ++i) {
        imu_data_.orientation_cov(i) = imu_sensor_handle_.getOrientationCovariance()[i];
        imu_data_.angular_vel_cov(i) = imu_sensor_handle_.getAngularVelocityCovariance()[i];
        imu_data_.linear_acc_cov(i) = imu_sensor_handle_.getLinearAccelerationCovariance()[i];
    }
}

void ControllerBase::UpdateJointState() {
    /** update joint data **/
    for (int i = 0; i < ControllerBase::num_of_actuation(); ++i) {
        joint_pos_(i) = hybrid_joint_handles_[i].getPosition();
        joint_vel_(i) = hybrid_joint_handles_[i].getVelocity();
        joint_tau_(i) = hybrid_joint_handles_[i].getEffort();
    }
}

void ControllerBase::ZeroTorque() {
    for (int i = 0; i < ControllerBase::num_of_actuation(); ++i) {
        SingleJointCtrl(i, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
    }
}

void ControllerBase::SingleJointCtrl(const int joint_index,
                                     const double q_des,
                                     const double dq_des,
                                     const double Kp,
                                     const double Kd,
                                     const double tau_ff,
                                     const uint8_t mode) {
    hybrid_joint_handles_[joint_index].setCommand(q_des, dq_des, Kp, Kd, tau_ff, mode);
}

void ControllerBase::MultiJointCtrl(const vector_t& q_des,
                                    const vector_t& dq_des,
                                    const vector_t& Kp,
                                    const vector_t& Kd,
                                    const vector_t& tau_ff,
                                    const std::vector<uint8_t>& mode) {
    for (int i = 0; i < ControllerBase::num_of_actuation(); ++i) {
        hybrid_joint_handles_[i].setCommand(q_des[i], dq_des[i], Kp[i], Kd[i], tau_ff[i], mode[i]);
    }
}

void ControllerBase::SetupHardwareHandle(hardware_interface::RobotHW* robot_hw) {
    // joint hardware handle
    // Hardware interface
    auto *hybridJointInterface = robot_hw->get<robot_common::HybridJointInterface>();

    // initJointAngles_.resize(initState.size());
    // Note: The order of joint angles below should match the order during training.
    std::string jointNames;
    ros::NodeHandle nh_global;
    nh_global.getParam("joints_name", joint_names_);
    for (size_t i = 0; i < joint_names_.size(); i++) {
      hybrid_joint_handles_.push_back(hybridJointInterface->getHandle(joint_names_[i]));
      jointNames += joint_names_[i];
      if (i != joint_names_.size() -1) {
        jointNames += ", ";
      }
    }
    ROS_INFO("hybridJointHandles jointNames: [%s]", jointNames.c_str());

    // imu handle
    imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("limx_imu");
}

}  // namespace controller
PLUGINLIB_EXPORT_CLASS(controller::ControllerBase, controller_interface::ControllerBase)
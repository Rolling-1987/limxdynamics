// Copyright information
//
// © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

#ifndef _LIMX_MIMICHUMANOID_CONTROLLER_H_
#define _LIMX_MIMICHUMANOID_CONTROLLER_H_

#include "ros/ros.h"
#include "humanoid_controller/ControllerBase.h"
#include "limxsdk/humanoid.h"

namespace controller {
class MimicHumanoidController : public ControllerBase {
    using tensor_element_t = float;

   public:
    MimicHumanoidController();
    ~MimicHumanoidController();
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void LoadRLParameters();

   protected:
    bool loadModel();
    bool loadRLCfg();
    void computeActions();
    void computeEncoder(const size_t index);
    void computeObservation();
    void computeMotionCurrentRef(const ros::Time& time);
    void Mimic(const ros::Time& time);

   private:
   void variableInit_();
    std::string policy_file_path_;
    std::shared_ptr<Ort::Env> onnxEnvPrt_;
    std::vector<std::vector<int64_t>> policyInputShapes_;
    std::vector<std::vector<int64_t>> policyOutputShapes_;
    std::unique_ptr<Ort::Session> policySessionPtr_;
    std::vector<const char*> policyInputNames_;
    std::vector<const char*> policyOutputNames_;

    bool isfirstRecObs_{true}, start_motion_{false};
    // vector3_t baseLinVel_;
    // vector3_t basePosition_;
    vector_t lastActions_;
    vector3_t command_filtered_;
    std::vector<int> joint_order_;
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> observations_;
    std::vector<tensor_element_t> action_filtered_;

    std::vector<std::vector<tensor_element_t>> encoder_output_;
    std::vector<std::unique_ptr<Ort::Session>> encoder_session_ptr_;
    std::vector<std::vector<const char*>> encoder_input_names_;
    std::vector<std::vector<const char*>> encoder_output_names_;
    std::vector<std::vector<std::vector<int64_t>>> encoder_input_shapes_;
    std::vector<std::vector<std::vector<int64_t>>> encoder_output_shapes_;

    double motion_phase_{0.}, motion_times_{5.0};
    int motion_frames_{1000}, motion_iter_{0};
    vector_t motion_cur_ref_;
    std::vector<vector_t> motion_refs_;
    vector3_t linear_velocity_estimated_{vector3_t::Zero()};

    Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
    Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> encoder_input_;

    controller::RLRobotCfg::HumanoidControlCfg& controlcfg_ = robotCfg.humanoidControlCfg;
    controller::RLRobotCfg::HumanoidObsScales& obscale_ = robotCfg.humanoidObsScales;
};

}  // namespace humanoid

#endif //_LIMX_HUMANOID_CONTROLLER_H_
#include "humanoid_controller/MimicHumanoidController.h"
#include <pluginlib/class_list_macros.hpp>

namespace controller {

MimicHumanoidController::MimicHumanoidController() {
    controller_name = "MimicHumanoidController";
}

MimicHumanoidController::~MimicHumanoidController() {}

bool MimicHumanoidController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
    controller::ControllerBase::init(robot_hw, nh);
    LoadRLParameters();
    return true;
}

void MimicHumanoidController::starting(const ros::Time& time) {
    controller::ControllerBase::starting(time);

    variableInit_();
}

void MimicHumanoidController::stopping(const ros::Time& time) {
    ControllerBase::stopping(time);
}

void MimicHumanoidController::update(const ros::Time& time, const ros::Duration& period) {
    controller::ControllerBase::update(time, period);
    Mimic(time);
}

void MimicHumanoidController::Mimic(const ros::Time& time) {
    if (controlcfg_.decimation == 0) {
        std::cerr << "----error----  robotCfg_.humanoidcontrolcfg_.decimation" << std::endl;
        return;
    }
    if (loop_count % controlcfg_.decimation == 0) {
        // update motion references
        computeMotionCurrentRef(time);
        
        // update robot obs
        computeObservation();
        
        // update encoders
        computeEncoder(0);
        computeEncoder(1);
        
        // update robot actions
        computeActions();

        scalar_t actionMin = -controlcfg_.clipActions;
        scalar_t actionMax = controlcfg_.clipActions;
        std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                       [actionMin, actionMax](scalar_t x) { return std::max(actionMin, std::min(actionMax, x)); });
        for (int i = 0; i < actions_.size(); ++i) {
            lastActions_(i) = actions_[i];
        }
        motion_iter_++;
    }

    // set action
    vector_t action_cliped(controlcfg_.actionsSize);
    size_t action_index = 0;
    const scalar_t soft_torque_limit = 0.95;
    for (int i = 0; i < num_of_actuation(); i++) {
        scalar_t actionMin = joint_pos()[i] + (controlcfg_.damping_vec[action_index] * joint_vel()[i] -
                                               controlcfg_.user_torque_limit[action_index] * soft_torque_limit) /
                                                  controlcfg_.stiffness_vec[action_index];
        scalar_t actionMax = joint_pos()[i] + (controlcfg_.damping_vec[action_index] * joint_vel()[i] +
                                               controlcfg_.user_torque_limit[action_index] * soft_torque_limit) /
                                                  controlcfg_.stiffness_vec[action_index];
        action_cliped[action_index] =
            std::max(actionMin / controlcfg_.action_scale_vec[action_index],
                    std::min(actionMax / controlcfg_.action_scale_vec[action_index], (scalar_t)actions_[action_index]));
        scalar_t pos_des = action_cliped[action_index] * controlcfg_.action_scale_vec[action_index];
        SingleJointCtrl(i, pos_des, 0, controlcfg_.stiffness_vec[action_index], controlcfg_.damping_vec[action_index], 0, 0);
        action_index++;
    }
}

void MimicHumanoidController::computeMotionCurrentRef(const ros::Time& time) { 

}

void MimicHumanoidController::computeObservation() {
    Eigen::Quaterniond q_wi;
    q_wi = imu_data().quat;
    vector3_t gravityVector(0, 0, -1);
    vector3_t projectedGravity(q_wi.toRotationMatrix().transpose() * gravityVector);
    vector3_t baseAngVel(imu_data().angular_vel_local);

    vector_t dof_joint_pos, dof_joint_vel;
    dof_joint_pos.resize(controlcfg_.actionsSize);
    dof_joint_vel.resize(controlcfg_.actionsSize);
    int count = 0;
    for (size_t i = 0; i < num_of_actuation(); ++i) {
        dof_joint_pos(count) = joint_pos()[i];
        dof_joint_vel(count++) = joint_vel()[i];
    }

    vector_t actions(lastActions_);
    Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> obs(controlcfg_.observationSize);

    obs.segment(0, 3) = baseAngVel * obscale_.angVel;
    obs.segment(3, 3) = projectedGravity;
    obs.segment(6, controlcfg_.actionsSize) = dof_joint_pos * obscale_.dofPos;
    obs.segment(6 + controlcfg_.actionsSize, controlcfg_.actionsSize) = dof_joint_vel * obscale_.dofVel;
    obs.segment(6 + 2 * controlcfg_.actionsSize, controlcfg_.actionsSize) = actions;

    motion_phase_ = (double) motion_iter_ / motion_frames_;
    if (motion_phase_ >=1.0) {
        motion_phase_ = 1.0;
    }

    

    obs(6 + 3 * controlcfg_.actionsSize) = motion_phase_;
    // obs.segment(6 + 3 * controlcfg_.actionsSize, controlcfg_.motionRefSize) = motion_cur_ref_;

    /***************************************/

    for (size_t i = 0; i < controlcfg_.observationSize; i++) {
        observations_[i] = static_cast<tensor_element_t>(obs(i));
    }

    if (isfirstRecObs_) {
        proprioHistoryBuffer_.resize(controlcfg_.obsHistoryLength * controlcfg_.observationSize);
        for (int i = 0; i < controlcfg_.obsHistoryLength; ++i) {
            proprioHistoryBuffer_.segment(controlcfg_.observationSize * i, controlcfg_.observationSize) =
                obs.head(controlcfg_.observationSize).cast<tensor_element_t>();
        }
        isfirstRecObs_ = false;
    }
    encoder_input_ = proprioHistoryBuffer_;
    for (int i = 0; i < controlcfg_.observationSize * (controlcfg_.obsHistoryLength - 1); ++i) {
        proprioHistoryBuffer_[i + controlcfg_.observationSize] = encoder_input_[i];
    }
    proprioHistoryBuffer_.head(controlcfg_.observationSize) = obs.head(controlcfg_.observationSize).cast<tensor_element_t>();

    scalar_t obsMin = -controlcfg_.clipObs;
    scalar_t obsMax = controlcfg_.clipObs;

    std::transform(observations_.begin(), observations_.end(), observations_.begin(), [obsMin, obsMax](scalar_t x) { return std::max(obsMin, std::min(obsMax, x)); });
}

void MimicHumanoidController::computeActions() {
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> inputValues;
    std::vector<tensor_element_t> combined_obs = observations_;

    combined_obs.push_back(command_filtered_(0));
    combined_obs.push_back(command_filtered_(1));
    combined_obs.push_back(command_filtered_(2));

    for (int i = 0; i < encoder_output_.size(); ++i) {
        for (const auto& item : encoder_output_[i]) {
            combined_obs.push_back(item);
        }
    }

    inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, combined_obs.data(), combined_obs.size(),
                                                                     policyInputShapes_[0].data(), policyInputShapes_[0].size()));

    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues =
        policySessionPtr_->Run(runOptions, policyInputNames_.data(), inputValues.data(), 1, policyOutputNames_.data(), 1);

    for (int i = 0; i < controlcfg_.actionsSize; i++) {
        actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
    }
}

void MimicHumanoidController::computeEncoder(const size_t index) {
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> inputValues;
    inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, proprioHistoryBuffer_.data(), proprioHistoryBuffer_.size(), encoder_input_shapes_[index][0].data(), encoder_input_shapes_[index][0].size()));

    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues = encoder_session_ptr_[index]->Run(runOptions, encoder_input_names_[index].data(),
                                                                            inputValues.data(), 1, encoder_output_names_[index].data(), 1);
    for (int i = 0; i < controlcfg_.encoder_output_size[index]; ++i) {
        encoder_output_[index][i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
        if (index == 0) {
            linear_velocity_estimated_[i] = encoder_output_[index][i];
        }
    }
}

void MimicHumanoidController::variableInit_() {
    controller::ControllerBase::loop_count = 0;

    std::fill(actions_.begin(), actions_.end(), 0.0);
    std::fill(action_filtered_.begin(), action_filtered_.end(), 0.0);
    std::fill(observations_.begin(), observations_.end(), 0.0);

    lastActions_.setZero();
    command_filtered_.setZero();
    // baseLinVel_.setZero();
    // basePosition_.setZero();
    motion_phase_ = 0;
    motion_iter_ = 0;
    isfirstRecObs_ = true;
}

void MimicHumanoidController::LoadRLParameters() {
    if (!loadRLCfg())
        ROS_INFO_STREAM("error in load RL Cfg");
    if (!loadModel())
        ROS_INFO_STREAM("error in load Model");
}

bool MimicHumanoidController::loadModel() {
    std::string policyModelPath;
    if (!nh_global.getParam("/policyFile", policyModelPath)) {
      ROS_ERROR("Failed to retrieve policy path from the parameter server!");
      return false;
    }
    ROS_INFO_STREAM("Policy model path: " << policyModelPath);

    std::vector<std::string> encoder_model_path(encoder_output_.size());
    if (!nh_global.getParam("/encoderFile1", encoder_model_path[0])) {
      ROS_ERROR("Failed to retrieve encoder path from the parameter server!");
      return false;
    }
    if (!nh_global.getParam("/encoderFile2", encoder_model_path[1])) {
      ROS_ERROR("Failed to retrieve encoder path from the parameter server!");
      return false;
    }
    for (size_t i = 0; i < encoder_model_path.size(); i++) {
      ROS_INFO_STREAM("Encoder model path: " << encoder_model_path[i]);
    }

    onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "MimicHumanoidControllerOnnx"));

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetInterOpNumThreads(1);

    Ort::AllocatorWithDefaultOptions allocator;

    policySessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyModelPath.c_str(), sessionOptions);
    policyInputNames_.clear();
    policyOutputNames_.clear();
    policyInputShapes_.clear();
    policyOutputShapes_.clear();
    for (int i = 0; i < policySessionPtr_->GetInputCount(); i++) {
        policyInputNames_.push_back(policySessionPtr_->GetInputName(i, allocator));
        policyInputShapes_.push_back(policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        std::cerr << policySessionPtr_->GetInputName(i, allocator) << std::endl;
        std::vector<int64_t> shape = policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        std::cerr << "Shape: [";
        for (size_t j = 0; j < shape.size(); ++j) {
            std::cout << shape[j];
            if (j != shape.size() - 1) {
                std::cerr << ", ";
            }
        }
        std::cout << "]" << std::endl;
    }
    for (int i = 0; i < policySessionPtr_->GetOutputCount(); i++) {
        policyOutputNames_.push_back(policySessionPtr_->GetOutputName(i, allocator));
        std::cerr << policySessionPtr_->GetOutputName(i, allocator) << std::endl;
        policyOutputShapes_.push_back(policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        std::vector<int64_t> shape = policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        std::cerr << "Shape: [";
        for (size_t j = 0; j < shape.size(); ++j) {
            std::cout << shape[j];
            if (j != shape.size() - 1) {
                std::cerr << ", ";
            }
        }
        std::cout << "]" << std::endl;
    }

    encoder_session_ptr_.resize(encoder_model_path.size());
    encoder_input_names_.resize(encoder_model_path.size());
    encoder_output_names_.resize(encoder_model_path.size());
    encoder_input_shapes_.resize(encoder_model_path.size());
    encoder_output_shapes_.resize(encoder_model_path.size());

    for (size_t i = 0; i < encoder_session_ptr_.size(); ++i) {
        ROS_INFO("load encoder from: %s", encoder_model_path[i].c_str());
        encoder_session_ptr_[i] = std::make_unique<Ort::Session>(*onnxEnvPrt_, encoder_model_path[i].c_str(), sessionOptions);
        encoder_input_names_[i].clear();
        encoder_output_names_[i].clear();
        encoder_input_shapes_[i].clear();
        encoder_output_shapes_[i].clear();
        for (int j = 0; j < encoder_session_ptr_[i]->GetInputCount(); ++j) {
            encoder_input_names_[i].push_back(encoder_session_ptr_[i]->GetInputName(j, allocator));
            encoder_input_shapes_[i].push_back(encoder_session_ptr_[i]->GetInputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
        }
        for (int j = 0; j < encoder_session_ptr_[i]->GetInputCount(); ++j) {
            encoder_output_names_[i].push_back(encoder_session_ptr_[i]->GetOutputName(j, allocator));
            encoder_output_shapes_[i].push_back(encoder_session_ptr_[i]->GetOutputTypeInfo(j).GetTensorTypeAndShapeInfo().GetShape());
        }
    }

    ROS_INFO("Load Onnx model from successfully !!!");
    return true;
}

bool MimicHumanoidController::loadRLCfg() {
    try {
        int error = 0;
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/motion_frames", motion_frames_));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/control/action_scale", controlcfg_.action_scale_vec));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/control/decimation", controlcfg_.decimation));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/control/user_torque_limit", controlcfg_.user_torque_limit));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/control/kp", controlcfg_.stiffness_vec));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/control/kd", controlcfg_.damping_vec));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/normalization/clip_scales/clip_observations", controlcfg_.clipObs));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/normalization/clip_scales/clip_actions", controlcfg_.clipActions));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/normalization/obs_scales/lin_vel", obscale_.linVel));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/normalization/obs_scales/ang_vel", obscale_.angVel));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/normalization/obs_scales/dof_pos", obscale_.dofPos));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/normalization/obs_scales/dof_vel", obscale_.dofVel));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/size/actions_size", controlcfg_.actionsSize));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/size/observations_size", controlcfg_.observationSize));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/size/obs_history_length", controlcfg_.obsHistoryLength));
        error += static_cast<int>(!nh_global.getParam("/HumanoidRobotCfg/size/encoder_output_size", controlcfg_.encoder_output_size));
        if (error) {
          ROS_ERROR("Load parameters from ROS parameter server error!!! %d",error);
        }
        actions_.resize(controlcfg_.actionsSize, 0.);
        action_filtered_.resize(controlcfg_.actionsSize, 0.);
        observations_.resize(controlcfg_.observationSize, 0.);

        encoder_output_.resize(controlcfg_.encoder_output_size.size());
        for (int i = 0; i < controlcfg_.encoder_output_size.size(); ++i) {
            encoder_output_[i].resize(controlcfg_.encoder_output_size[i]);
        }

        lastActions_.resize(controlcfg_.actionsSize);
        lastActions_.setZero();

        command_filtered_.setZero();
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Error in the HumanoidRobotCfg: %s", e.what());
        return false;
    }
}
}

PLUGINLIB_EXPORT_CLASS(controller::MimicHumanoidController, controller_interface::ControllerBase)
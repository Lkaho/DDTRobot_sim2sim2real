/*============================= RL ==============================*/

#include "fsm/FSMState_RL.h"
#include "common/timeMarker.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace
{
constexpr const char * kDefaultEnginePath =
  "/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/engine/policy.engine";

std::string trimString(const std::string & input)
{
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::string stripQuotes(const std::string & input)
{
  if (input.size() >= 2) {
    const char first = input.front();
    const char last = input.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
      return input.substr(1, input.size() - 2);
    }
  }
  return input;
}

std::filesystem::path getDefaultRuntimeConfigPath()
{
  return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
         "config" / "rl_runtime.yaml";
}

std::filesystem::path resolveRuntimeConfigPath()
{
  const char * env_path = std::getenv("TITA_RL_CONFIG");
  if (env_path != nullptr && env_path[0] != '\0') {
    return std::filesystem::path(env_path);
  }
  return getDefaultRuntimeConfigPath();
}

bool parseFloatConfigValue(const std::string & value, float & output)
{
  try {
    size_t parsed = 0;
    output = std::stof(value, &parsed);
    return trimString(value.substr(parsed)).empty();
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

FSMState_RL::FSMState_RL(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::RL, "rl"),
  output(new float[8]),
  output_last(new float[8])
{
  runtime_config_.engine_path = kDefaultEnginePath;
  _LoadRuntimeConfig();
  _ResetEngine();
}

FSMState_RL::~FSMState_RL()
{
  stop_update_ = true;
  threadRunning = false;
  if (forward_thread.joinable()) {
    forward_thread.join();
  }
}

void FSMState_RL::enter()
{ 
  const std::string previous_engine_path = runtime_config_.engine_path;
  _LoadRuntimeConfig();
  if (cuda_test_ == nullptr || runtime_config_.engine_path != previous_engine_path) {
    _ResetEngine();
  }

  _data->state_command->firstRun = true;

  for (int i = 0; i < 2; i++)
  {
    wheel_init_pos_abs_[i] = _data->low_state->q[4*i+3];
    desired_pos[4 * i + 3] = 0;
    desired_pos[4 * i] = _data->low_state->q[4*i];
    desired_pos[4 * i + 1] = _data->low_state->q[4*i+1];
    desired_pos[4 * i + 2] = _data->low_state->q[4*i+2];
  }

  // Transition-to-default was previously tested here; keep direct policy handoff for now.
  // for (int i = 0; i < 8; i++) {
  //   entry_joint_pos_[i] = _data->low_state->q[i];
  // }
  // entry_wheel_vel_[0] = _data->low_state->dq[3];
  // entry_wheel_vel_[1] = _data->low_state->dq[7];
  // transition_step_ = 0;

  // 初始化当前观测
  current_obs_.joint_vel[0] = _data->low_state->dq[0];
  current_obs_.joint_vel[1] = _data->low_state->dq[4];
  current_obs_.joint_vel[2] = _data->low_state->dq[1];
  current_obs_.joint_vel[3] = _data->low_state->dq[5];
  current_obs_.joint_vel[4] = _data->low_state->dq[2];
  current_obs_.joint_vel[5] = _data->low_state->dq[6];
  current_obs_.joint_vel[6] = _data->low_state->dq[3];
  current_obs_.joint_vel[7] = _data->low_state->dq[7];
  current_obs_.joint_pos[0] = _data->low_state->q[0];
  current_obs_.joint_pos[1] = _data->low_state->q[4];
  current_obs_.joint_pos[2] = _data->low_state->q[1];
  current_obs_.joint_pos[3] = _data->low_state->q[5];
  current_obs_.joint_pos[4] = _data->low_state->q[2];
  current_obs_.joint_pos[5] = _data->low_state->q[6];

  params_.action_scale = 0.5;
  params_.num_of_dofs = 8;
  params_.lin_vel_scale = 2.0;
  params_.ang_vel_scale = 0.25;
  params_.dof_pos_scale = 1.0;
  params_.dof_vel_scale = 0.05;
  params_.commands_scale[0] = params_.lin_vel_scale;
  params_.commands_scale[1] = 0.0;  // y命令训练时为0
  params_.commands_scale[2] = params_.ang_vel_scale;

  const float default_dof_pos_tmp[8] = {0.1, 0.8, -1.5, 0, 0.1, 0.8, -1.5, 0};
  for (int i = 0; i < 8; i++)
  {
    params_.default_dof_pos[i] = default_dof_pos_tmp[i];
  }
  params_.default_dof_pos_leg[0] = default_dof_pos_tmp[0];
  params_.default_dof_pos_leg[1] = default_dof_pos_tmp[4];
  params_.default_dof_pos_leg[2] = default_dof_pos_tmp[1];
  params_.default_dof_pos_leg[3] = default_dof_pos_tmp[5];
  params_.default_dof_pos_leg[4] = default_dof_pos_tmp[2];
  params_.default_dof_pos_leg[5] = default_dof_pos_tmp[6];

  x_vel_cmd_ = 0.;
  y_vel_cmd_ = 0.;
  pitch_cmd_ = 0.;

  // 清空历史缓冲区
  for (int i = 0; i < 30; i++) hist_ang_vel[i] = 0;
  for (int i = 0; i < 30; i++) hist_gravity[i] = 0;
  for (int i = 0; i < 30; i++) hist_commands[i] = 0;
  for (int i = 0; i < 60; i++) hist_joint_pos[i] = 0;
  for (int i = 0; i < 80; i++) hist_joint_vel[i] = 0;
  for (int i = 0; i < 80; i++) hist_last_action[i] = 0;

  for (int i = 0; i < 8; i++) output_last.get()[i] = 0;
  a_l.setZero();
  last_print_time_ = 0;

  // action初始化
  action[0] = _data->low_state->q[0];
  action[1] = _data->low_state->q[1];
  action[2] = _data->low_state->q[2];
  action[3] = 0.0f;
  action[4] = _data->low_state->q[4];
  action[5] = _data->low_state->q[5];
  action[6] = _data->low_state->q[6];
  action[7] = 0.0f;

  // 预热填充历史缓冲区
  // 注意：前10帧的last_action必须保持为0
  for (int i = 0; i < history_length; i++)
  {
    _GetObs();
    // 强制last_action为0（前10帧）
    for (int j = 0; j < 8; j++) {
      current_obs_.last_action[j] = 0.0f;
    }
    _UpdateHistory();
  }
  std::cout << "init finished predict" << std::endl;
  std::cout << "[RL] joint_pos order fixed to [joint_left_leg_1, joint_right_leg_1, joint_left_leg_2, "
            << "joint_right_leg_2, joint_left_leg_3, joint_right_leg_3]" << std::endl;
  std::cout << "[RL] joint_vel order fixed to [joint_left_leg_1, joint_right_leg_1, joint_left_leg_2, "
            << "joint_right_leg_2, joint_left_leg_3, joint_right_leg_3, joint_left_leg_4, "
            << "joint_right_leg_4]" << std::endl;
  std::cout << "[RL] action order fixed to [joint_left_leg_1, joint_left_leg_2, joint_left_leg_3, "
            << "joint_left_leg_4, joint_right_leg_1, joint_right_leg_2, joint_right_leg_3, "
            << "joint_right_leg_4]" << std::endl;
  std::cout << "[RL] transition-to-default disabled; using direct policy handoff." << std::endl;

  // 预热推理（此时output_last仍为0，last_action历史保持为0）
  float temp_input[310];
  for (int i = 0; i < 10; i++)
  {
    _GetObs();
    // 强制last_action为0（预热推理阶段）
    for (int j = 0; j < 8; j++) {
      current_obs_.last_action[j] = 0.0f;
    }
    _AssembleInput(temp_input);
    cuda_test_->do_inference(temp_input, output.get());
    // 注意：不更新output_last，保持为0
  }

  threadRunning = true;
  if (thread_first_)
  {
    forward_thread = std::thread(&FSMState_RL::_Run_Forward, this);
    thread_first_ = false;
  }
  stop_update_ = false;
}

void FSMState_RL::run()
{
  x_vel_cmd_ = _data->state_command->rc_data_->twist_linear[point::X];
  y_vel_cmd_ = 0;  // 禁用y方向，与训练一致
  pitch_cmd_ = _data->state_command->rc_data_->twist_angular[point::Z];
  
  _data->low_cmd->qd.setZero();
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd.setZero();

  const float kLegKp = runtime_config_.leg_kp;
  const float kLegKd = runtime_config_.leg_kd;
  const float kWheelKd = runtime_config_.wheel_kd;
  const float kWheelKp = runtime_config_.wheel_kp;

  _data->low_cmd->qd[0] = desired_pos[0];
  _data->low_cmd->qd[1] = desired_pos[1];
  _data->low_cmd->qd[2] = desired_pos[2];
  _data->low_cmd->kp[0] = kLegKp;
  _data->low_cmd->kp[1] = kLegKp;
  _data->low_cmd->kp[2] = kLegKp;
  _data->low_cmd->kd[0] = kLegKd;
  _data->low_cmd->kd[1] = kLegKd;
  _data->low_cmd->kd[2] = kLegKd;

  _data->low_cmd->qd[3] = _data->low_state->q[3];
  _data->low_cmd->qd_dot[3] = desired_pos[3];
  _data->low_cmd->kp[3] = kWheelKp;
  _data->low_cmd->kd[3] = kWheelKd;

  _data->low_cmd->qd[4] = desired_pos[4];
  _data->low_cmd->qd[5] = desired_pos[5];
  _data->low_cmd->qd[6] = desired_pos[6];
  _data->low_cmd->kp[4] = kLegKp;
  _data->low_cmd->kp[5] = kLegKp;
  _data->low_cmd->kp[6] = kLegKp;
  _data->low_cmd->kd[4] = kLegKd;
  _data->low_cmd->kd[5] = kLegKd;
  _data->low_cmd->kd[6] = kLegKd;

  _data->low_cmd->qd[7] =  _data->low_state->q[7];
  _data->low_cmd->qd_dot[7] = desired_pos[7];
  _data->low_cmd->kp[7] = kWheelKp;
  _data->low_cmd->kd[7] = kWheelKd;
}

void FSMState_RL::exit() 
{
  stop_update_ = true;
}

void FSMState_RL::_LoadRuntimeConfig()
{
  const std::filesystem::path config_path = resolveRuntimeConfigPath();
  runtime_config_.config_path = config_path.string();

  std::ifstream config_file(config_path);
  if (!config_file.is_open()) {
    std::cout << "[RL] runtime config not found, use defaults: " << runtime_config_.config_path
              << std::endl;
    return;
  }

  RLRuntimeConfig loaded_config = runtime_config_;
  loaded_config.config_path = config_path.string();

  std::string line;
  while (std::getline(config_file, line)) {
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }

    line = trimString(line);
    if (line.empty()) {
      continue;
    }

    size_t separator_pos = line.find(':');
    if (separator_pos == std::string::npos) {
      separator_pos = line.find('=');
    }
    if (separator_pos == std::string::npos) {
      continue;
    }

    const std::string key = trimString(line.substr(0, separator_pos));
    const std::string value = stripQuotes(trimString(line.substr(separator_pos + 1)));

    if (key == "engine_path") {
      loaded_config.engine_path = value;
      continue;
    }

    float parsed_value = 0.0f;
    if (!parseFloatConfigValue(value, parsed_value)) {
      std::cout << "[RL] invalid runtime config value, skip: " << key
                << " = " << value << std::endl;
      continue;
    }

    if (key == "leg_kp") {
      loaded_config.leg_kp = parsed_value;
    } else if (key == "leg_kd") {
      loaded_config.leg_kd = parsed_value;
    } else if (key == "wheel_kp") {
      loaded_config.wheel_kp = parsed_value;
    } else if (key == "wheel_kd") {
      loaded_config.wheel_kd = parsed_value;
    }
  }

  runtime_config_ = loaded_config;
  std::cout << "[RL] runtime config loaded: " << runtime_config_.config_path << std::endl;
  std::cout << "[RL] engine_path = " << runtime_config_.engine_path << std::endl;
  std::cout << "[RL] gains: leg(" << runtime_config_.leg_kp << ", " << runtime_config_.leg_kd
            << "), wheel(" << runtime_config_.wheel_kp << ", " << runtime_config_.wheel_kd
            << ")" << std::endl;
}

void FSMState_RL::_ResetEngine()
{
  cuda_test_ = std::make_shared<CudaTest>(runtime_config_.engine_path);
  std::cout << "cuda init :" << cuda_test_->get_cuda_init() << std::endl;
}

FSMStateName FSMState_RL::checkTransition()
{
  this->_nextStateName = this->_stateName;
  switch (_data->state_command->desire_data_->fsm_state_name)
  {
  case FSMStateName::RECOVERY_STAND:
    this->_nextStateName = FSMStateName::RECOVERY_STAND;
    break;
  case FSMStateName::RL:
    break;
  case FSMStateName::TRANSFORM_DOWN:
    this->_nextStateName = FSMStateName::TRANSFORM_DOWN;
    break;
  case FSMStateName::PASSIVE:
    this->_nextStateName = FSMStateName::PASSIVE;
    break;
  default:
    break;
  }
  return this->_nextStateName;
}

// 更新当前观测（31维）
void FSMState_RL::_GetObs()
{
  Mat3<double> _B2G_RotMat = this->_data->state_estimator->getResult().rBody;
  a_l = 0.97*this->_data->state_estimator->getResult().omegaBody + 0.03*a_l;
  Vec3<double> projected_gravity = _B2G_RotMat * Vec3<double>(0.0, 0.0, -1.0);

  // base_ang_vel (3)
  current_obs_.base_ang_vel[0] = a_l(0) * 0.25;
  current_obs_.base_ang_vel[1] = a_l(1) * 0.25;
  current_obs_.base_ang_vel[2] = a_l(2) * 0.25;

  // projected_gravity (3)
  current_obs_.projected_gravity[0] = projected_gravity(0);
  current_obs_.projected_gravity[1] = projected_gravity(1);
  current_obs_.projected_gravity[2] = projected_gravity(2);

  // velocity_commands (3)
  float rx = pitch_cmd_;
  float lx = x_vel_cmd_;
  float max = 1.0;
  float min = -1.0;
  float rot = rx * 3.14;
  float vel_x = lx * 2;
  double heading = 0.;
  double angle = (double)rot - heading;
  angle = fmod(angle, 2.0*M_PI);
  if(angle > M_PI) angle = angle - 2.0*M_PI;
  angle = angle * 0.5;
  angle = std::max(std::min((float)angle, max), min);
  angle = angle * 0.25;
  
  current_obs_.velocity_commands[0] = vel_x;
  current_obs_.velocity_commands[1] = 0;  // y方向始终为0
  current_obs_.velocity_commands[2] = angle;

  // joint_pos (6) follows IsaacLab policy order: L1, R1, L2, R2, L3, R3.
  float raw_joint_pos[6];
  raw_joint_pos[0] = _data->low_state->q[0];
  raw_joint_pos[1] = _data->low_state->q[4];
  raw_joint_pos[2] = _data->low_state->q[1];
  raw_joint_pos[3] = _data->low_state->q[5];
  raw_joint_pos[4] = _data->low_state->q[2];
  raw_joint_pos[5] = _data->low_state->q[6];
  
  for (int i = 0; i < 6; i++) {
    current_obs_.joint_pos[i] = (raw_joint_pos[i] - params_.default_dof_pos_leg[i]) * params_.dof_pos_scale;
  }

  // joint_vel (8) follows IsaacLab policy order: L1, R1, L2, R2, L3, R3, L4, R4.
  float raw_joint_vel[8];
  raw_joint_vel[0] = _data->low_state->dq[0];
  raw_joint_vel[1] = _data->low_state->dq[4];
  raw_joint_vel[2] = _data->low_state->dq[1];
  raw_joint_vel[3] = _data->low_state->dq[5];
  raw_joint_vel[4] = _data->low_state->dq[2];
  raw_joint_vel[5] = _data->low_state->dq[6];
  raw_joint_vel[6] = _data->low_state->dq[3];
  raw_joint_vel[7] = _data->low_state->dq[7];
  for (int i = 0; i < 8; i++) {
    current_obs_.joint_vel[i] = raw_joint_vel[i] * params_.dof_vel_scale;
  }

  // last_action (8) - 复制上一次的output_last
  for (int i = 0; i < 8; i++) {
    current_obs_.last_action[i] = output_last.get()[i];
  }
}

// 更新历史缓冲区（按观测项分组，各维护10帧历史）
void FSMState_RL::_UpdateHistory()
{
  // 左移历史：去掉最旧的，加入最新的
  
  // base_ang_vel: 3维 * 10帧
  for (int i = 0; i < 27; i++) hist_ang_vel[i] = hist_ang_vel[i+3];
  hist_ang_vel[27] = current_obs_.base_ang_vel[0];
  hist_ang_vel[28] = current_obs_.base_ang_vel[1];
  hist_ang_vel[29] = current_obs_.base_ang_vel[2];

  // projected_gravity: 3维 * 10帧
  for (int i = 0; i < 27; i++) hist_gravity[i] = hist_gravity[i+3];
  hist_gravity[27] = current_obs_.projected_gravity[0];
  hist_gravity[28] = current_obs_.projected_gravity[1];
  hist_gravity[29] = current_obs_.projected_gravity[2];

  // velocity_commands: 3维 * 10帧
  for (int i = 0; i < 27; i++) hist_commands[i] = hist_commands[i+3];
  hist_commands[27] = current_obs_.velocity_commands[0];
  hist_commands[28] = current_obs_.velocity_commands[1];
  hist_commands[29] = current_obs_.velocity_commands[2];

  // joint_pos: 6维 * 10帧
  for (int i = 0; i < 54; i++) hist_joint_pos[i] = hist_joint_pos[i+6];
  for (int i = 0; i < 6; i++) hist_joint_pos[54+i] = current_obs_.joint_pos[i];

  // joint_vel: 8维 * 10帧
  for (int i = 0; i < 72; i++) hist_joint_vel[i] = hist_joint_vel[i+8];
  for (int i = 0; i < 8; i++) hist_joint_vel[72+i] = current_obs_.joint_vel[i];

  // last_action: 8维 * 10帧
  for (int i = 0; i < 72; i++) hist_last_action[i] = hist_last_action[i+8];
  for (int i = 0; i < 8; i++) hist_last_action[72+i] = current_obs_.last_action[i];
}

// 组装310维网络输入（按观测项分组：[ang_vel历史][gravity历史][commands历史][joint_pos历史][joint_vel历史][last_action历史]）
void FSMState_RL::_AssembleInput(float* input_buffer)
{
  int idx = 0;
  
  // base_ang_vel历史 (30维) [0:30]
  for (int i = 0; i < 30; i++) input_buffer[idx++] = hist_ang_vel[i];
  
  // projected_gravity历史 (30维) [30:60]
  for (int i = 0; i < 30; i++) input_buffer[idx++] = hist_gravity[i];
  
  // velocity_commands历史 (30维) [60:90]
  for (int i = 0; i < 30; i++) input_buffer[idx++] = hist_commands[i];
  
  // joint_pos历史 (60维) [90:150]
  for (int i = 0; i < 60; i++) input_buffer[idx++] = hist_joint_pos[i];
  
  // joint_vel历史 (80维) [150:230]
  for (int i = 0; i < 80; i++) input_buffer[idx++] = hist_joint_vel[i];
  
  // last_action历史 (80维) [230:310]
  for (int i = 0; i < 80; i++) input_buffer[idx++] = hist_last_action[i];
}

void FSMState_RL::_Forward()
{
  float input_buffer[310];
  _Forward(input_buffer);
}

void FSMState_RL::_Forward(float* input_buffer)
{
  _GetObs();
  _UpdateHistory();
  _AssembleInput(input_buffer);
  
  cuda_test_->do_inference(input_buffer, output.get());

  for (int i = 0; i < 8; i++)
    output_last.get()[i] = output.get()[i];
}

void FSMState_RL::_Run_Forward()
{
  while (threadRunning)
  {
    long long _start_time = getSystemTime();

    if (!stop_update_)
    {
      float input_buffer[310];

      // _Forward 内部会调用 _GetObs() 读取并缩放关节状态
      _Forward(input_buffer);

      float policy_action[8];
      // joint_left_leg_1,2,3 -> position
      policy_action[0] = output.get()[0] * 0.25f + params_.default_dof_pos[0];
      policy_action[1] = output.get()[1] * 0.25f + params_.default_dof_pos[1];
      policy_action[2] = output.get()[2] * 0.25f + params_.default_dof_pos[2];
      // joint_left_leg_4 -> velocity, restored to direct policy output (no filter)
      policy_action[3] = output.get()[3] * 5.0f;

      // joint_right_leg_1,2,3 -> position
      policy_action[4] = output.get()[4] * 0.25f + params_.default_dof_pos[4];
      policy_action[5] = output.get()[5] * 0.25f + params_.default_dof_pos[5];
      policy_action[6] = output.get()[6] * 0.25f + params_.default_dof_pos[6];
      // joint_right_leg_4 -> velocity, restored to direct policy output (no filter)
      policy_action[7] = output.get()[7] * 5.0f;

      // Transition-to-default/ramp-to-policy is disabled for now.
      // const int total_transition_steps = align_steps_ + ramp_steps_;
      // if (transition_step_ < align_steps_) {
      //   const float align_ratio =
      //     static_cast<float>(transition_step_ + 1) / static_cast<float>(std::max(1, align_steps_));
      //   for (int i = 0; i < 8; i++) {
      //     if (i % 4 == 3) {
      //       const int wheel_id = i / 4;
      //       action[i] = lerpValue(entry_wheel_vel_[wheel_id], 0.0f, align_ratio);
      //     } else {
      //       action[i] = lerpValue(entry_joint_pos_[i], params_.default_dof_pos[i], align_ratio);
      //     }
      //   }
      // } else if (transition_step_ < total_transition_steps) {
      //   const float ramp_ratio =
      //     static_cast<float>(transition_step_ - align_steps_ + 1) / static_cast<float>(std::max(1, ramp_steps_));
      //   for (int i = 0; i < 8; i++) {
      //     const float default_target = (i % 4 == 3) ? 0.0f : params_.default_dof_pos[i];
      //     action[i] = lerpValue(default_target, policy_action[i], ramp_ratio);
      //   }
      // } else {
      //   for (int i = 0; i < 8; i++) {
      //     action[i] = policy_action[i];
      //   }
      // }
      // transition_step_++;
      for (int i = 0; i < 8; i++) {
        action[i] = policy_action[i];
      }
      
      // 打印最新观测值
      long long current_time = getSystemTime();
      if (current_time - last_print_time_ > 100000) {
        std::cout << "=== Current Observation (31 dims) ===" << std::endl;
        std::cout << "base_ang_vel (3): ";
        for (int i = 0; i < 3; i++) std::cout << current_obs_.base_ang_vel[i] << " ";
        std::cout << std::endl;
        std::cout << "projected_gravity (3): ";
        for (int i = 0; i < 3; i++) std::cout << current_obs_.projected_gravity[i] << " ";
        std::cout << std::endl;
        std::cout << "velocity_commands (3): ";
        for (int i = 0; i < 3; i++) std::cout << current_obs_.velocity_commands[i] << " ";
        std::cout << std::endl;
        std::cout << "joint_pos_rel (6): ";
        for (int i = 0; i < 6; i++) std::cout << current_obs_.joint_pos[i] << " ";
        std::cout << std::endl;
        std::cout << "joint_vel (8): ";
        for (int i = 0; i < 8; i++) std::cout << current_obs_.joint_vel[i] << " ";
        std::cout << std::endl;
        std::cout << "last_action (8): ";
        for (int i = 0; i < 8; i++) std::cout << current_obs_.last_action[i] << " ";
        std::cout << std::endl;
        std::cout << "Network output (8): ";
        for (int j = 0; j < 8; j++) std::cout << output.get()[j] << " ";
        std::cout << std::endl;
        std::cout << "Wheel target raw: "
                  << output.get()[3] * 5.0f << " | "
                  << output.get()[7] * 5.0f << std::endl;
        std::cout << "Policy handoff mode: direct" << std::endl;
        std::cout << "Action (8): ";
        for (int j = 0; j < 8; j++) std::cout << action[j] << " ";
        std::cout << std::endl;
        std::cout << "---" << std::endl;
        last_print_time_ = current_time;
      }

      for (int i = 0; i < 8; i++)
      {
        desired_pos[i] = action[i];
      }
    }

    absoluteWait(_start_time, (long long)(0.02 * 1000000));
  }
  threadRunning = false;
}

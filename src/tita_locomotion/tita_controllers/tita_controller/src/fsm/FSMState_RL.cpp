/*============================= RL ==============================*/

#include "fsm/FSMState_RL.h"
#include "common/timeMarker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace
{
constexpr const char * kDefaultEnginePath =
  "/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/engine/policy.engine";
constexpr int kActionDim = 8;
constexpr int kMaxObservationTermDim = 8;

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

bool parseIntConfigValue(const std::string & value, int & output)
{
  try {
    size_t parsed = 0;
    output = std::stoi(value, &parsed);
    return trimString(value.substr(parsed)).empty();
  } catch (const std::exception &) {
    return false;
  }
}

bool parseStringListConfigValue(const std::string & value, std::vector<std::string> & output)
{
  const std::string trimmed = trimString(value);
  if (trimmed.size() < 2 || trimmed.front() != '[' || trimmed.back() != ']') {
    return false;
  }

  output.clear();
  const std::string inner = trimString(trimmed.substr(1, trimmed.size() - 2));
  if (inner.empty()) {
    return true;
  }

  std::stringstream ss(inner);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = stripQuotes(trimString(item));
    if (!item.empty()) {
      output.push_back(item);
    }
  }
  return true;
}

int getObservationTermDim(const std::string & term_name)
{
  if (term_name == "base_ang_vel") {
    return 3;
  }
  if (term_name == "projected_gravity") {
    return 3;
  }
  if (term_name == "velocity_commands") {
    return 3;
  }
  if (term_name == "joint_pos") {
    return 6;
  }
  if (term_name == "joint_vel") {
    return 8;
  }
  if (term_name == "last_action") {
    return 8;
  }
  if (term_name == "base_lin_vel_xy") {
    return 2;
  }
  return 0;
}

std::string joinStrings(const std::vector<std::string> & items)
{
  std::ostringstream oss;
  for (size_t i = 0; i < items.size(); i++) {
    if (i != 0) {
      oss << ", ";
    }
    oss << items[i];
  }
  return oss.str();
}

std::string formatArrayString(const float * values, int dim, int precision = 6)
{
  std::ostringstream oss;
  oss << "[";
  oss << std::setprecision(precision);
  for (int i = 0; i < dim; i++) {
    if (i != 0) {
      oss << " ";
    }
    oss << values[i];
  }
  oss << "]";
  return oss.str();
}

double wrapToPi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}
}  // namespace

FSMState_RL::FSMState_RL(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::RL, "rl"),
  output(new float[kActionDim]),
  output_last(new float[kActionDim])
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
  } else {
    _ValidateRuntimeConfigAgainstEngine();
  }

  _data->state_command->firstRun = true;

  for (int i = 0; i < 2; i++)
  {
    wheel_init_pos_abs_[i] = _data->low_state->q[4 * i + 3];
    desired_pos[4 * i + 3] = 0;
    desired_pos[4 * i] = _data->low_state->q[4 * i];
    desired_pos[4 * i + 1] = _data->low_state->q[4 * i + 1];
    desired_pos[4 * i + 2] = _data->low_state->q[4 * i + 2];
  }

  params_.action_scale = 0.5;
  params_.num_of_dofs = 8;
  params_.lin_vel_scale = 2.0;
  params_.ang_vel_scale = 0.25;
  params_.dof_pos_scale = 1.0;
  params_.dof_vel_scale = 0.05;
  params_.commands_scale[0] = params_.lin_vel_scale;
  params_.commands_scale[1] = 0.0;
  params_.commands_scale[2] = params_.ang_vel_scale;

  const float default_dof_pos_tmp[kActionDim] = {0.1, 0.8, -1.5, 0, 0.1, 0.8, -1.5, 0};
  for (int i = 0; i < kActionDim; i++)
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
  raw_yaw_vel_cmd_ = 0.0f;
  heading_cmd_ = static_cast<float>(_data->state_estimator->getResult().rpy(rpy::YAW));
  last_external_heading_cmd_ =
    static_cast<float>(_data->state_command->desire_data_->pose_rpy(rpy::YAW));
  hold_yaw_cmd_ = 0.0f;
  applied_yaw_cmd_ = 0.0f;
  yaw_input_active_ = false;

  for (auto & term_buffer : observation_term_buffers_) {
    std::fill(term_buffer.history.begin(), term_buffer.history.end(), 0.0f);
  }

  for (int i = 0; i < kActionDim; i++) {
    output_last.get()[i] = 0.0f;
  }
  estimated_velocity_[0] = 0.0f;
  estimated_velocity_[1] = 0.0f;
  a_l.setZero();
  last_print_time_ = 0;
  debug_step_counter_ = 0;

  action[0] = _data->low_state->q[0];
  action[1] = _data->low_state->q[1];
  action[2] = _data->low_state->q[2];
  action[3] = 0.0f;
  action[4] = _data->low_state->q[4];
  action[5] = _data->low_state->q[5];
  action[6] = _data->low_state->q[6];
  action[7] = 0.0f;

  for (int i = 0; i < runtime_config_.history_length; i++)
  {
    _GetObs();
    for (int j = 0; j < kActionDim; j++) {
      current_obs_.last_action[j] = 0.0f;
    }
    _UpdateHistory();
  }
  std::cout << "init finished predict" << std::endl;
  std::cout << "[RL] initial heading target = " << heading_cmd_ << " rad" << std::endl;
  std::cout << "[RL] joint_pos order fixed to [joint_left_leg_1, joint_right_leg_1, joint_left_leg_2, "
            << "joint_right_leg_2, joint_left_leg_3, joint_right_leg_3]" << std::endl;
  std::cout << "[RL] joint_vel order fixed to [joint_left_leg_1, joint_right_leg_1, joint_left_leg_2, "
            << "joint_right_leg_2, joint_left_leg_3, joint_right_leg_3, joint_left_leg_4, "
            << "joint_right_leg_4]" << std::endl;
  std::cout << "[RL] policy action order fixed to [joint_left_leg_1, joint_left_leg_2, "
            << "joint_left_leg_3, joint_right_leg_1, joint_right_leg_2, joint_right_leg_3, "
            << "joint_left_leg_4, joint_right_leg_4]" << std::endl;
  std::cout << "[RL] hardware action order fixed to [joint_left_leg_1, joint_left_leg_2, "
            << "joint_left_leg_3, joint_left_leg_4, joint_right_leg_1, joint_right_leg_2, "
            << "joint_right_leg_3, joint_right_leg_4]" << std::endl;
  std::cout << "[RL] transition-to-default disabled; using direct policy handoff." << std::endl;

  for (int i = 0; i < runtime_config_.history_length; i++)
  {
    _GetObs();
    for (int j = 0; j < kActionDim; j++) {
      current_obs_.last_action[j] = 0.0f;
    }
    _AssembleInput(warmup_input_buffer_.data());
    cuda_test_->do_inference(warmup_input_buffer_.data(), output.get());
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
  y_vel_cmd_ = 0;
  raw_yaw_vel_cmd_ = static_cast<float>(_data->state_command->rc_data_->twist_angular[point::Z]);

  const float external_heading_cmd =
    static_cast<float>(_data->state_command->desire_data_->pose_rpy(rpy::YAW));
  if (std::abs(external_heading_cmd - last_external_heading_cmd_) > 1.0e-4f) {
    heading_cmd_ = external_heading_cmd;
  }
  last_external_heading_cmd_ = external_heading_cmd;

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

  _data->low_cmd->qd[7] = _data->low_state->q[7];
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
  RLRuntimeConfig loaded_config;
  loaded_config.config_path = config_path.string();
  loaded_config.engine_path = kDefaultEnginePath;

  std::ifstream config_file(config_path);
  if (!config_file.is_open()) {
    runtime_config_ = loaded_config;
    _ConfigureObservationHistory();
    std::cout << "[RL] runtime config not found, use defaults: " << runtime_config_.config_path
              << std::endl;
    return;
  }

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
    const std::string value = trimString(line.substr(separator_pos + 1));

    if (key == "engine_path") {
      loaded_config.engine_path = stripQuotes(value);
      continue;
    }
    if (key == "history_length") {
      int parsed_value = 0;
      if (!parseIntConfigValue(value, parsed_value)) {
        std::cout << "[RL] invalid runtime config value, skip: " << key
                  << " = " << value << std::endl;
        continue;
      }
      loaded_config.history_length = parsed_value;
      continue;
    }
    if (key == "observation_terms") {
      std::vector<std::string> parsed_terms;
      if (!parseStringListConfigValue(value, parsed_terms)) {
        std::cout << "[RL] invalid runtime config value, skip: " << key
                  << " = " << value << std::endl;
        continue;
      }
      loaded_config.observation_terms = parsed_terms;
      continue;
    }

    float parsed_value = 0.0f;
    if (!parseFloatConfigValue(stripQuotes(value), parsed_value)) {
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
    } else if (key == "heading_control_stiffness") {
      loaded_config.heading_control_stiffness = parsed_value;
    } else if (key == "heading_ang_vel_min") {
      loaded_config.heading_ang_vel_min = parsed_value;
    } else if (key == "heading_ang_vel_max") {
      loaded_config.heading_ang_vel_max = parsed_value;
    } else {
      std::cout << "[RL] unknown runtime config key, skip: " << key << std::endl;
    }
  }

  const RLRuntimeConfig default_config;
  if (loaded_config.history_length <= 0) {
    std::cout << "[RL] invalid history_length=" << loaded_config.history_length
              << ", fallback to " << default_config.history_length << std::endl;
    loaded_config.history_length = default_config.history_length;
  }

  std::vector<std::string> sanitized_terms;
  for (const auto & term_name : loaded_config.observation_terms) {
    if (getObservationTermDim(term_name) <= 0) {
      std::cout << "[RL] unsupported observation term, skip: " << term_name << std::endl;
      continue;
    }
    if (std::find(sanitized_terms.begin(), sanitized_terms.end(), term_name) != sanitized_terms.end()) {
      std::cout << "[RL] duplicate observation term, skip: " << term_name << std::endl;
      continue;
    }
    sanitized_terms.push_back(term_name);
  }
  if (sanitized_terms.empty()) {
    std::cout << "[RL] no valid observation_terms configured, fallback to defaults" << std::endl;
    sanitized_terms = default_config.observation_terms;
  }
  loaded_config.observation_terms = sanitized_terms;

  runtime_config_ = loaded_config;
  _ConfigureObservationHistory();

  std::cout << "[RL] runtime config loaded: " << runtime_config_.config_path << std::endl;
  std::cout << "[RL] engine_path = " << runtime_config_.engine_path << std::endl;
  std::cout << "[RL] gains: leg(" << runtime_config_.leg_kp << ", " << runtime_config_.leg_kd
            << "), wheel(" << runtime_config_.wheel_kp << ", " << runtime_config_.wheel_kd
            << ")" << std::endl;
  std::cout << "[RL] heading control: stiffness=" << runtime_config_.heading_control_stiffness
            << ", yaw_vel_range=[" << runtime_config_.heading_ang_vel_min << ", "
            << runtime_config_.heading_ang_vel_max << "]" << std::endl;
  std::cout << "[RL] history_length = " << runtime_config_.history_length << std::endl;
  std::cout << "[RL] observation_terms = [" << joinStrings(runtime_config_.observation_terms)
            << "]" << std::endl;
  std::cout << "[RL] configured obs dims: frame=" << _GetConfiguredSingleFrameObservationDim()
            << ", history=" << _GetConfiguredInputDim() << std::endl;
}

void FSMState_RL::_ConfigureObservationHistory()
{
  observation_term_buffers_.clear();
  observation_term_buffers_.reserve(runtime_config_.observation_terms.size());

  for (const auto & term_name : runtime_config_.observation_terms) {
    ObservationTermBuffer term_buffer;
    term_buffer.name = term_name;
    term_buffer.dim = getObservationTermDim(term_name);
    term_buffer.history.assign(
      static_cast<size_t>(term_buffer.dim * runtime_config_.history_length), 0.0f);
    observation_term_buffers_.push_back(std::move(term_buffer));
  }

  inference_input_buffer_.assign(_GetConfiguredInputDim(), 0.0f);
  warmup_input_buffer_.assign(_GetConfiguredInputDim(), 0.0f);
}

void FSMState_RL::_ValidateRuntimeConfigAgainstEngine() const
{
  if (cuda_test_ == nullptr || !cuda_test_->get_cuda_init()) {
    throw std::runtime_error(
      std::string("[RL] failed to initialize TensorRT engine: ") + runtime_config_.engine_path);
  }

  const size_t configured_input_dim = _GetConfiguredInputDim();
  const size_t engine_input_dim = cuda_test_->get_input_dim();
  if (configured_input_dim != engine_input_dim) {
    throw std::runtime_error(
      std::string("[RL] observation history dim mismatch, configured=") +
      std::to_string(configured_input_dim) +
      ", engine=" + std::to_string(engine_input_dim));
  }
}

void FSMState_RL::_ResetEngine()
{
  cuda_test_ = std::make_shared<CudaTest>(runtime_config_.engine_path);
  std::cout << "cuda init :" << cuda_test_->get_cuda_init() << std::endl;
  _ValidateRuntimeConfigAgainstEngine();
}

size_t FSMState_RL::_GetConfiguredInputDim() const
{
  size_t input_dim = 0;
  for (const auto & term_buffer : observation_term_buffers_) {
    input_dim += term_buffer.history.size();
  }
  return input_dim;
}

size_t FSMState_RL::_GetConfiguredSingleFrameObservationDim() const
{
  size_t frame_dim = 0;
  for (const auto & term_buffer : observation_term_buffers_) {
    frame_dim += static_cast<size_t>(term_buffer.dim);
  }
  return frame_dim;
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

void FSMState_RL::_GetObs()
{
  Mat3<double> _B2G_RotMat = this->_data->state_estimator->getResult().rBody;
  a_l = 0.97 * this->_data->state_estimator->getResult().omegaBody + 0.03 * a_l;
  Vec3<double> projected_gravity = _B2G_RotMat * Vec3<double>(0.0, 0.0, -1.0);

  current_obs_.base_ang_vel[0] = a_l(0) * params_.ang_vel_scale;
  current_obs_.base_ang_vel[1] = a_l(1) * params_.ang_vel_scale;
  current_obs_.base_ang_vel[2] = a_l(2) * params_.ang_vel_scale;

  current_obs_.projected_gravity[0] = projected_gravity(0);
  current_obs_.projected_gravity[1] = projected_gravity(1);
  current_obs_.projected_gravity[2] = projected_gravity(2);

  const float lx = x_vel_cmd_;
  const float vel_x = lx * 2.0f;
  const float heading = static_cast<float>(this->_data->state_estimator->getResult().rpy(rpy::YAW));
  yaw_input_active_ = std::abs(raw_yaw_vel_cmd_) > 1.0e-3f;
  if (yaw_input_active_) {
    heading_cmd_ = heading;
  }

  const double yaw_error = wrapToPi(static_cast<double>(heading_cmd_) - heading);
  hold_yaw_cmd_ = std::clamp(
    runtime_config_.heading_control_stiffness * static_cast<float>(yaw_error),
    runtime_config_.heading_ang_vel_min,
    runtime_config_.heading_ang_vel_max);
  applied_yaw_cmd_ = yaw_input_active_ ? raw_yaw_vel_cmd_ : hold_yaw_cmd_;

  current_obs_.velocity_commands[0] = vel_x;
  current_obs_.velocity_commands[1] = 0;
  current_obs_.velocity_commands[2] = applied_yaw_cmd_ * params_.ang_vel_scale;

  float raw_joint_pos[6];
  raw_joint_pos[0] = _data->low_state->q[0];
  raw_joint_pos[1] = _data->low_state->q[4];
  raw_joint_pos[2] = _data->low_state->q[1];
  raw_joint_pos[3] = _data->low_state->q[5];
  raw_joint_pos[4] = _data->low_state->q[2];
  raw_joint_pos[5] = _data->low_state->q[6];

  for (int i = 0; i < 6; i++) {
    current_obs_.joint_pos[i] =
      (raw_joint_pos[i] - params_.default_dof_pos_leg[i]) * params_.dof_pos_scale;
  }

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

  for (int i = 0; i < kActionDim; i++) {
    current_obs_.last_action[i] = output_last.get()[i];
  }

  current_obs_.base_lin_vel_xy[0] =
    static_cast<float>(this->_data->state_estimator->getResult().vBody(0)) * params_.lin_vel_scale;
  current_obs_.base_lin_vel_xy[1] =
    static_cast<float>(this->_data->state_estimator->getResult().vBody(1)) * params_.lin_vel_scale;
}

void FSMState_RL::_FillObservationTermValues(const std::string & term_name, float * values) const
{
  if (term_name == "base_ang_vel") {
    for (int i = 0; i < 3; i++) {
      values[i] = current_obs_.base_ang_vel[i];
    }
    return;
  }
  if (term_name == "projected_gravity") {
    for (int i = 0; i < 3; i++) {
      values[i] = current_obs_.projected_gravity[i];
    }
    return;
  }
  if (term_name == "velocity_commands") {
    for (int i = 0; i < 3; i++) {
      values[i] = current_obs_.velocity_commands[i];
    }
    return;
  }
  if (term_name == "joint_pos") {
    for (int i = 0; i < 6; i++) {
      values[i] = current_obs_.joint_pos[i];
    }
    return;
  }
  if (term_name == "joint_vel") {
    for (int i = 0; i < 8; i++) {
      values[i] = current_obs_.joint_vel[i];
    }
    return;
  }
  if (term_name == "last_action") {
    for (int i = 0; i < 8; i++) {
      values[i] = current_obs_.last_action[i];
    }
    return;
  }
  if (term_name == "base_lin_vel_xy") {
    values[0] = current_obs_.base_lin_vel_xy[0];
    values[1] = current_obs_.base_lin_vel_xy[1];
  }
}

void FSMState_RL::_UpdateHistory()
{
  std::array<float, kMaxObservationTermDim> term_values = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  for (auto & term_buffer : observation_term_buffers_) {
    term_values.fill(0.0f);
    _FillObservationTermValues(term_buffer.name, term_values.data());

    const size_t history_dim = term_buffer.history.size();
    const size_t term_dim = static_cast<size_t>(term_buffer.dim);
    for (size_t i = 0; i + term_dim < history_dim; i++) {
      term_buffer.history[i] = term_buffer.history[i + term_dim];
    }
    const size_t tail_offset = history_dim - term_dim;
    for (size_t i = 0; i < term_dim; i++) {
      term_buffer.history[tail_offset + i] = term_values[i];
    }
  }
}

void FSMState_RL::_AssembleInput(float * input_buffer)
{
  size_t idx = 0;
  for (const auto & term_buffer : observation_term_buffers_) {
    for (float value : term_buffer.history) {
      input_buffer[idx++] = value;
    }
  }
}

void FSMState_RL::_Forward()
{
  _Forward(inference_input_buffer_.data());
}

void FSMState_RL::_Forward(float * input_buffer)
{
  _GetObs();
  _UpdateHistory();
  _AssembleInput(input_buffer);

  cuda_test_->do_inference(input_buffer, output.get(), estimated_velocity_);

  for (int i = 0; i < kActionDim; i++) {
    output_last.get()[i] = output.get()[i];
  }
}

void FSMState_RL::_PrintConfiguredObservationTerms() const
{
  std::array<float, kMaxObservationTermDim> term_values = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  for (const auto & term_buffer : observation_term_buffers_) {
    term_values.fill(0.0f);
    _FillObservationTermValues(term_buffer.name, term_values.data());
    std::cout << "    " << term_buffer.name << " (hist=" << runtime_config_.history_length
              << "): " << formatArrayString(term_values.data(), term_buffer.dim) << std::endl;
  }
}

void FSMState_RL::_Run_Forward()
{
  while (threadRunning)
  {
    long long _start_time = getSystemTime();

    if (!stop_update_)
    {
      _Forward();

      float hardware_action[kActionDim];
      hardware_action[0] = output.get()[0] * 0.25f + params_.default_dof_pos[0];
      hardware_action[1] = output.get()[1] * 0.25f + params_.default_dof_pos[1];
      hardware_action[2] = output.get()[2] * 0.25f + params_.default_dof_pos[2];
      hardware_action[3] = output.get()[6] * 5.0f;
      hardware_action[4] = output.get()[3] * 0.25f + params_.default_dof_pos[4];
      hardware_action[5] = output.get()[4] * 0.25f + params_.default_dof_pos[5];
      hardware_action[6] = output.get()[5] * 0.25f + params_.default_dof_pos[6];
      hardware_action[7] = output.get()[7] * 5.0f;

      for (int i = 0; i < kActionDim; i++) {
        action[i] = hardware_action[i];
      }

      const long long current_time = getSystemTime();
      if (current_time - last_print_time_ > 100000) {
        const float base_velocity_cmd[3] = {x_vel_cmd_, 0.0f, applied_yaw_cmd_};
        const float raw_keyboard_cmd[3] = {
          x_vel_cmd_,
          0.0f,
          raw_yaw_vel_cmd_};
        const float policy_applied[kActionDim] = {
          output.get()[0] * 0.25f + params_.default_dof_pos[0],
          output.get()[1] * 0.25f + params_.default_dof_pos[1],
          output.get()[2] * 0.25f + params_.default_dof_pos[2],
          output.get()[3] * 0.25f + params_.default_dof_pos[4],
          output.get()[4] * 0.25f + params_.default_dof_pos[5],
          output.get()[5] * 0.25f + params_.default_dof_pos[6],
          output.get()[6] * 5.0f,
          output.get()[7] * 5.0f};
        const float policy_scale[kActionDim] = {0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 5.0f, 5.0f};
        const float policy_offset[kActionDim] = {
          params_.default_dof_pos[0],
          params_.default_dof_pos[1],
          params_.default_dof_pos[2],
          params_.default_dof_pos[4],
          params_.default_dof_pos[5],
          params_.default_dof_pos[6],
          0.0f,
          0.0f};
        const char * action_term_names[kActionDim] = {
          "joint_pos->joint_left_leg_1",
          "joint_pos->joint_left_leg_2",
          "joint_pos->joint_left_leg_3",
          "joint_pos->joint_right_leg_1",
          "joint_pos->joint_right_leg_2",
          "joint_pos->joint_right_leg_3",
          "joint_vel->joint_left_leg_4",
          "joint_vel->joint_right_leg_4"};

        std::cout << "\n[Step " << debug_step_counter_ << ", Time "
                  << std::fixed << std::setprecision(1) << (static_cast<double>(debug_step_counter_) * 0.02)
                  << "s]" << std::endl;
        std::cout << "  Current policy observation: total_dim=" << _GetConfiguredInputDim() << std::endl;
        std::cout << "  Policy terms (current frame view):" << std::endl;
        _PrintConfiguredObservationTerms();
        if (cuda_test_ != nullptr && cuda_test_->has_estimated_velocity_output()) {
          const double estimated_vx = static_cast<double>(estimated_velocity_[0]) / params_.lin_vel_scale;
          const double estimated_vy = static_cast<double>(estimated_velocity_[1]) / params_.lin_vel_scale;
          double true_vx = this->_data->state_estimator->getResult().vBody(0);
          double true_vy = this->_data->state_estimator->getResult().vBody(1);
          bool using_wheel_velocity_fallback = false;

          if (std::abs(true_vx) < 1e-6 && std::abs(true_vy) < 1e-6) {
            const double left_wheel_vel = static_cast<double>(this->_data->low_state->dq[3]);
            const double right_wheel_vel = static_cast<double>(this->_data->low_state->dq[7]);
            const double wheel_radius = static_cast<double>(this->_data->params->wheel_radius);
            true_vx = 0.5 * wheel_radius * (left_wheel_vel + right_wheel_vel);
            true_vy = 0.0;
            using_wheel_velocity_fallback = true;
          }

          const double err_vx = estimated_vx - true_vx;
          const double err_vy = estimated_vy - true_vy;
          const double err_l2 = std::sqrt(err_vx * err_vx + err_vy * err_vy);
          std::cout << "  Velocity estimator:" << std::endl;
          std::cout << "    estimated base_lin_vel_xy: [" << estimated_vx << " " << estimated_vy << "]" << std::endl;
          std::cout << "    true base_lin_vel_xy"
                    << (using_wheel_velocity_fallback ? " (wheel-derived)" : "")
                    << ":      [" << true_vx << " " << true_vy << "]" << std::endl;
          std::cout << "    error:                     [" << err_vx << " " << err_vy
                    << "] |l2|=" << std::setprecision(4) << err_l2 << std::endl;
        }

        std::cout << "  Commands:" << std::endl;
        std::cout << "    base_velocity: " << formatArrayString(base_velocity_cmd, 3, 6) << std::endl;
        std::cout << "  Keyboard heading-hold:" << std::endl;
        std::cout << "    raw keyboard cmd: " << formatArrayString(raw_keyboard_cmd, 3, 6) << std::endl;
        std::cout << "    desired heading:  " << std::setprecision(4) << heading_cmd_ << std::endl;
        std::cout << "    hold yaw cmd:     " << std::setprecision(4) << hold_yaw_cmd_ << std::endl;
        std::cout << "    yaw input active: " << (yaw_input_active_ ? "true" : "false") << std::endl;

        std::cout << "  Actions (synced):" << std::endl;
        std::cout << "    " << std::left << std::setw(5) << "Idx"
                  << " " << std::setw(35) << "Term->Joint"
                  << " " << std::setw(10) << "Raw"
                  << " " << std::setw(8) << "Scale"
                  << " " << std::setw(12) << "Applied"
                  << " " << std::setw(15) << "Offset/Note" << std::endl;
        std::cout << "    " << std::string(80, '-') << std::endl;
        for (int j = 0; j < kActionDim; j++) {
          std::ostringstream offset_note;
          if (j < 6) {
            offset_note << "offset=" << std::fixed << std::setprecision(4) << policy_offset[j];
          } else {
            offset_note << "offset=0.0000";
          }
          std::cout << "    [" << std::left << std::setw(3) << j << "] "
                    << std::setw(35) << action_term_names[j]
                    << " " << std::right << std::fixed << std::setprecision(4) << std::setw(8) << output.get()[j]
                    << " x" << std::left << std::setw(6) << std::setprecision(2) << policy_scale[j]
                    << "=" << std::right << std::setw(10) << std::setprecision(4) << policy_applied[j]
                    << "  " << std::left << std::setw(15) << offset_note.str() << std::endl;
        }
        std::cout << "  Policy handoff mode: direct" << std::endl;
        std::cout << "---" << std::endl;
        last_print_time_ = current_time;
      }

      for (int i = 0; i < kActionDim; i++)
      {
        desired_pos[i] = action[i];
      }
      debug_step_counter_++;
    }

    absoluteWait(_start_time, static_cast<long long>(0.02 * 1000000));
  }
  threadRunning = false;
}

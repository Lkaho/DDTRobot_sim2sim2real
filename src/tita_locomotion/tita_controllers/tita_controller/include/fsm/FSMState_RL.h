#ifndef FSMSTATE_RL_H
#define FSMSTATE_RL_H

#include <string>
#include <thread>
#include "FSMState.h"
#include "tensorrt_cuda/tensor_cuda_test.hpp"

struct ModelParams
{
    float damping;
    float stiffness;
    float action_scale;
    float hip_scale_reduction;
    float num_of_dofs;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float clip_obs;
    float clip_actions;
    float torque_limits[8];
    float d_gains[8];
    float p_gains[8];
    float commands_scale[3];
    float default_dof_pos[8];
    float default_dof_pos_leg[6];
};

// 单帧观测结构（31维）
struct ObsFrame
{
    float base_ang_vel[3];      // [0:3]
    float projected_gravity[3]; // [3:6]
    float velocity_commands[3]; // [6:9]
    float joint_pos[6];         // [9:15]
    float joint_vel[8];         // [15:23]
    float last_action[8];       // [23:31]
};

struct RLRuntimeConfig
{
    std::string config_path;
    std::string engine_path;
    float leg_kp = 60.0f;
    float leg_kd = 20.0f;
    float wheel_kp = 1.5f;
    float wheel_kd = 12.0f;
};

class FSMState_RL : public FSMState
{
public:
  FSMState_RL(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_RL();

  void enter();
  void run();
  FSMStateName checkTransition();
  void exit();

private:
  float wheel_init_pos_abs_[4];
  float x_vel_cmd_;
  float y_vel_cmd_;
  float pitch_cmd_;
  
  ModelParams params_;
  ObsFrame current_obs_;  // 当前帧观测（31维）
  
  // 历史缓冲区（按观测项分组存储）
  // base_ang_vel: 3 * 10 = 30
  float hist_ang_vel[30];
  // projected_gravity: 3 * 10 = 30
  float hist_gravity[30];
  // velocity_commands: 3 * 10 = 30
  float hist_commands[30];
  // joint_pos: 6 * 10 = 60
  float hist_joint_pos[60];
  // joint_vel: 8 * 10 = 80
  float hist_joint_vel[80];
  // last_action: 8 * 10 = 80
  float hist_last_action[80];
  // 总计: 30+30+30+60+80+80 = 310

  void _Forward();  // 使用内部buffer
  void _Forward(float* input_buffer);  // 使用外部buffer（用于打印）
  void _Run_Forward();
  void _GetObs();  // 更新 current_obs_
  void _AssembleInput(float* input_buffer);  // 组装310维输入
  void _UpdateHistory();  // 更新历史缓冲区
  void _LoadRuntimeConfig();
  void _ResetEngine();

  std::shared_ptr<CudaTest> cuda_test_;
  RLRuntimeConfig runtime_config_;
  std::thread forward_thread;
  bool threadRunning;
  float desired_pos[8] = {0, 0.75, -1.5, 0, 0, 0.75, -1.5, 0};
  float entry_joint_pos_[8] = {0.0f};
  float entry_wheel_vel_[2] = {0.0f, 0.0f};
  int transition_step_ = 0;
  int align_steps_ = 25;
  int ramp_steps_ = 25;

  std::shared_ptr<float[]> output;       // 8维网络输出
  std::shared_ptr<float[]> output_last;  // 8维上一动作

  int history_length = 10;
  Vec3<double> a_l;
  float action[8];
  bool stop_update_ = false;
  bool thread_first_ = true;
  long long last_print_time_ = 0;
};

#endif  // FSMSTATE_RL_H

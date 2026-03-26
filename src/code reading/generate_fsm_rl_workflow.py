#!/usr/bin/env python3
"""
生成 FSMState_RL 工作流图
"""

from graphviz import Digraph

def create_fsm_rl_workflow():
    dot = Digraph(comment='FSMState_RL Workflow', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5', fontname='Helvetica')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    dot.attr('edge', fontname='Helvetica', fontsize='8')

    # ==================== 构造函数 ====================
    with dot.subgraph(name='cluster_ctor') as c:
        c.attr(label='构造函数', style='rounded,filled', fillcolor='#E3F2FD', color='#1565C0')
        c.node('init_cuda', '初始化 CUDA\nCudaTest("xx.engine")', shape='box', style='filled,rounded', fillcolor='white')
        c.node('alloc_memory', '分配内存\ninput_0[33], input_1[330]\noutput[8], output_last[8]', shape='box', style='filled,rounded', fillcolor='white')

    # ==================== enter() ====================
    with dot.subgraph(name='cluster_enter') as c:
        c.attr(label='enter() - 进入状态', style='rounded,filled', fillcolor='#E8F5E9', color='#2E7D32')
        
        c.node('first_run', '设置 firstRun = true', shape='box', style='filled,rounded', fillcolor='white')
        c.node('init_pos', '记录轮子初始位置\nwheel_init_pos_abs_', shape='box', style='filled,rounded', fillcolor='white')
        c.node('init_obs', '初始化观测值\ndof_pos, dof_vel', shape='box', style='filled,rounded', fillcolor='white')
        c.node('set_params', '设置参数\naction_scale, lin_vel_scale\ndof_pos_scale, etc.', shape='box', style='filled,rounded', fillcolor='white')
        c.node('reset_cmd', '重置命令\nx_vel_cmd = 0\ny_vel_cmd = 0\npitch_cmd = 0', shape='box', style='filled,rounded', fillcolor='white')
        c.node('clear_buffer', '清空历史缓冲区\ninput_1 = 0\noutput_last = 0', shape='box', style='filled,rounded', fillcolor='white')
        c.node('warmup', '预热填充缓冲区\nfor i in range(history_length=10)\n  _GetObs() + shift buffer', shape='box', style='filled,rounded', fillcolor='#FFF3E0')
        c.node('warmup_forward', '预热前向传播\nfor i in range(10)\n  _Forward()', shape='box', style='filled,rounded', fillcolor='#FFF3E0')
        c.node('start_thread', '启动推理线程\n_Run_Forward() in new thread', shape='box', style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')

    # ==================== run() ====================
    with dot.subgraph(name='cluster_run') as c:
        c.attr(label='run() - 主循环 (每周期调用)', style='rounded,filled', fillcolor='#FFF3E0', color='#E65100')
        
        c.node('get_cmd', '获取遥控命令\nrc_data_->twist_linear[X/Y]\ntwist_angular[Z]', shape='box', style='filled,rounded', fillcolor='white')
        c.node('zero_cmd', '清零命令\nqd=0, qd_dot=0\nkp=0, kd=0\ntau_cmd=0', shape='box', style='filled,rounded', fillcolor='white')
        c.node('compute_torque', '计算力矩输出\n轮子: 12*pos + 1.5*vel\n关节: 60*pos_err + 2.0*vel', shape='box', style='filled,rounded', fillcolor='#E1F5FE', penwidth='2')

    # ==================== _Run_Forward() 线程 ====================
    with dot.subgraph(name='cluster_thread') as c:
        c.attr(label='_Run_Forward() - 后台推理线程 (100Hz)', style='rounded,filled', fillcolor='#F3E5F5', color='#6A1B9A')
        
        c.node('thread_start', '线程开始\nwhile(threadRunning)', shape='ellipse', style='filled', fillcolor='white')
        c.node('read_state', '读取关节状态\nq[0-3]→dof_pos[4-7]\ndq[0-3]→dof_vel[4-7]\nq[4-7]→dof_pos[0-3]\ndq[4-7]→dof_vel[0-3]', shape='box', style='filled,rounded', fillcolor='white')
        c.node('call_forward', '调用 _Forward()', shape='box', style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
        c.node('process_action', '处理网络输出\naction[j] = output[j] * action_scale + default_dof_pos', shape='box', style='filled,rounded', fillcolor='white')
        c.node('update_desired', '更新期望位置\ndesired_pos[i+4] = action[i]\ndesired_pos[i] = action[i+4]', shape='box', style='filled,rounded', fillcolor='white')
        c.node('sleep', '精确睡眠\nabsoluteWait(10ms)', shape='box', style='filled,rounded', fillcolor='white')

    # ==================== _Forward() ====================
    with dot.subgraph(name='cluster_forward') as c:
        c.attr(label='_Forward() - 单次推理', style='rounded,filled', fillcolor='#FFEBEE', color='#C62828')
        
        c.node('get_obs', '_GetObs()\n构建观测向量', shape='box', style='filled,rounded', fillcolor='white')
        c.node('cuda_infer', 'CUDA推理\ncuda_test_->do_inference()\ninput_0[33] + input_1[330] → output[8]', shape='box', style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
        c.node('shift_buffer', '滑动窗口更新\ninput_1 左移33\ninput_0 追加到末尾', shape='box', style='filled,rounded', fillcolor='white')
        c.node('save_output', '保存输出\noutput_last = output', shape='box', style='filled,rounded', fillcolor='white')

    # ==================== _GetObs() ====================
    with dot.subgraph(name='cluster_getobs') as c:
        c.attr(label='_GetObs() - 构建观测 (33维)', style='rounded,filled', fillcolor='#E0F7FA', color='#00838F')
        
        c.node('get_rotmat', '获取旋转矩阵\n_B2G_RotMat, _G2B_RotMat\n(from state_estimator)', shape='box', style='filled,rounded', fillcolor='white')
        c.node('filter_angvel', '角速度滤波\nangvel = 0.97*omegaBody + 0.03*a_l', shape='box', style='filled,rounded', fillcolor='white')
        c.node('calc_gravity', '计算重力投影\nprojected_gravity = _B2G_RotMat * (0,0,-1)', shape='box', style='filled,rounded', fillcolor='white')
        c.node('calc_forward', '计算前向向量\nprojected_forward = _G2B_RotMat * (1,0,0)', shape='box', style='filled,rounded', fillcolor='white')
        
        # 观测值分组
        c.node('obs_angvel', 'obs[0-2]:\nangvel * 0.25', shape='box', style='filled,rounded', fillcolor='#E1F5FE')
        c.node('obs_gravity', 'obs[3-5]:\nprojected_gravity', shape='box', style='filled,rounded', fillcolor='#E1F5FE')
        c.node('obs_cmd', 'obs[6-8]:\nvel_x, vel_y, angle\n(commands_scale)', shape='box', style='filled,rounded', fillcolor='#E1F5FE')
        c.node('obs_pos', 'obs[9-16]:\n(dof_pos - default) * scale', shape='box', style='filled,rounded', fillcolor='#E1F5FE')
        c.node('obs_vel', 'obs[17-24]:\ndof_vel * 0.05', shape='box', style='filled,rounded', fillcolor='#E1F5FE')
        c.node('obs_last', 'obs[25-32]:\noutput_last (上一动作)', shape='box', style='filled,rounded', fillcolor='#E1F5FE')

    # ==================== checkTransition() ====================
    with dot.subgraph(name='cluster_transition') as c:
        c.attr(label='checkTransition() - 状态切换', style='rounded,filled', fillcolor='#FBE9E7', color='#BF360C')
        
        c.node('check_fsm', '检查 desire_data_->fsm_state_name', shape='diamond', style='filled', fillcolor='white')
        c.node('to_recovery', '→ RECOVERY_STAND', shape='box', style='filled,rounded', fillcolor='white')
        c.node('to_rl', '→ RL (保持)', shape='box', style='filled,rounded', fillcolor='#E8F5E9')
        c.node('to_transform', '→ TRANSFORM_DOWN', shape='box', style='filled,rounded', fillcolor='white')
        c.node('to_passive', '→ PASSIVE', shape='box', style='filled,rounded', fillcolor='white')

    # ==================== exit() ====================
    with dot.subgraph(name='cluster_exit') as c:
        c.attr(label='exit() - 退出状态', style='rounded,filled', fillcolor='#EFEBE9', color='#4E342E')
        c.node('stop_thread', '停止更新\nstop_update_ = true', shape='box', style='filled,rounded', fillcolor='white')

    # ==================== 数据流连接 ====================
    # 构造流程
    dot.edge('init_cuda', 'alloc_memory', style='invis')
    
    # enter流程
    dot.edge('alloc_memory', 'first_run', style='dashed', color='#757575')
    dot.edge('first_run', 'init_pos', style='invis')
    dot.edge('init_pos', 'init_obs', style='invis')
    dot.edge('init_obs', 'set_params', style='invis')
    dot.edge('set_params', 'reset_cmd', style='invis')
    dot.edge('reset_cmd', 'clear_buffer', style='invis')
    dot.edge('clear_buffer', 'warmup', style='invis')
    dot.edge('warmup', 'warmup_forward', style='invis')
    dot.edge('warmup_forward', 'start_thread', style='invis')
    
    # run流程
    dot.edge('start_thread', 'get_cmd', style='dashed', color='#757575', label='周期调用')
    dot.edge('get_cmd', 'zero_cmd', style='invis')
    dot.edge('zero_cmd', 'compute_torque', style='invis')
    
    # 线程流程
    dot.edge('start_thread', 'thread_start', style='dashed', color='#6A1B9A', label='启动')
    dot.edge('thread_start', 'read_state', style='invis')
    dot.edge('read_state', 'call_forward', style='invis')
    dot.edge('call_forward', 'process_action', style='invis')
    dot.edge('process_action', 'update_desired', style='invis')
    dot.edge('update_desired', 'sleep', style='invis')
    dot.edge('sleep', 'thread_start', style='dashed', color='#6A1B9A', label='循环')
    
    # _Forward流程
    dot.edge('call_forward', 'get_obs', style='dashed', color='#C62828', label='调用')
    dot.edge('get_obs', 'cuda_infer', style='invis')
    dot.edge('cuda_infer', 'shift_buffer', style='invis')
    dot.edge('shift_buffer', 'save_output', style='invis')
    dot.edge('save_output', 'process_action', style='dashed', color='#6A1B9A', constraint='false')
    
    # _GetObs流程
    dot.edge('get_obs', 'get_rotmat', style='dashed', color='#00838F')
    dot.edge('get_rotmat', 'filter_angvel', style='invis')
    dot.edge('filter_angvel', 'calc_gravity', style='invis')
    dot.edge('calc_gravity', 'calc_forward', style='invis')
    
    # 观测值构建
    dot.edge('calc_forward', 'obs_angvel', style='invis')
    dot.edge('obs_angvel', 'obs_gravity', style='invis')
    dot.edge('obs_gravity', 'obs_cmd', style='invis')
    dot.edge('obs_cmd', 'obs_pos', style='invis')
    dot.edge('obs_pos', 'obs_vel', style='invis')
    dot.edge('obs_vel', 'obs_last', style='invis')
    dot.edge('obs_last', 'cuda_infer', style='dashed', color='#C62828', constraint='false')
    
    # 状态转换
    dot.edge('get_cmd', 'check_fsm', style='dashed', color='#BF360C', label='检查')
    dot.edge('check_fsm', 'to_recovery', label='RECOVERY_STAND', color='#BF360C')
    dot.edge('check_fsm', 'to_rl', label='RL', color='#2E7D32')
    dot.edge('check_fsm', 'to_transform', label='TRANSFORM_DOWN', color='#BF360C')
    dot.edge('check_fsm', 'to_passive', label='PASSIVE', color='#BF360C')
    
    # exit
    dot.edge('to_recovery', 'stop_thread', style='dashed', color='#4E342E')
    dot.edge('to_transform', 'stop_thread', style='dashed', color='#4E342E')
    dot.edge('to_passive', 'stop_thread', style='dashed', color='#4E342E')

    # 保存
    dot.render('fsm_rl_workflow', cleanup=True)
    print(f"✓ FSMState_RL 工作流图已生成: fsm_rl_workflow.png")
    
    return dot


def create_rl_data_flow():
    """RL 数据流图"""
    dot = Digraph(comment='RL Data Flow', format='png')
    dot.attr(rankdir='LR', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    
    # 输入
    dot.node('imu', 'IMU数据\n(gyro, accel)', shape='box', style='filled,rounded', fillcolor='#E3F2FD')
    dot.node('joint', '关节状态\n(q, dq)', shape='box', style='filled,rounded', fillcolor='#E3F2FD')
    dot.node('cmd', '遥控命令\n(x_vel, y_vel, yaw)', shape='box', style='filled,rounded', fillcolor='#FFF3E0')
    
    # 处理
    dot.node('estimator', '状态估计器\nStateEstimator\n(旋转矩阵, 角速度)', shape='box', style='filled,rounded', fillcolor='#E8F5E9')
    dot.node('getobs', '_GetObs()\n构建33维观测', shape='box', style='filled,rounded', fillcolor='#E0F7FA')
    
    # 神经网络
    dot.node('buffer', '历史缓冲区\ninput_1[330]\n(10帧历史)', shape='cylinder', style='filled', fillcolor='#F3E5F5')
    dot.node('nn', '神经网络\nTensorRT CUDA\nPolicy Network', shape='box', style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
    
    # 输出
    dot.node('action', '动作输出\noutput[8]\n(关节位置偏移)', shape='box', style='filled,rounded', fillcolor='#FFEBEE')
    dot.node('pd', 'PD控制\n(位置环+阻尼)', shape='box', style='filled,rounded', fillcolor='#FFF8E1')
    dot.node('torque', '力矩输出\ntau_cmd[8]', shape='box', style='filled,rounded', fillcolor='#C8E6C9', penwidth='2')
    
    # 连接
    dot.edge('imu', 'estimator', color='#1976D2')
    dot.edge('joint', 'getobs', color='#1976D2')
    dot.edge('estimator', 'getobs', color='#1976D2')
    dot.edge('cmd', 'getobs', color='#F57C00')
    
    dot.edge('getobs', 'nn', label='input_0[33]', color='#C62828')
    dot.edge('buffer', 'nn', label='input_1[330]', color='#C62828')
    dot.edge('nn', 'buffer', label='更新历史', style='dashed', color='#6A1B9A')
    
    dot.edge('nn', 'action', color='#C62828')
    dot.edge('action', 'pd', color='#F9A825')
    dot.edge('joint', 'pd', label='反馈', style='dashed', color='#1976D2')
    dot.edge('pd', 'torque', color='#388E3C', penwidth='2')
    
    dot.render('fsm_rl_dataflow', cleanup=True)
    print(f"✓ RL数据流图已生成: fsm_rl_dataflow.png")
    
    return dot


def create_observation_breakdown():
    """观测值分解图"""
    dot = Digraph(comment='Observation Breakdown', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    
    dot.node('root', 'Observations[33]', shape='box', style='filled,rounded', fillcolor='#1565C0', fontcolor='white', penwidth='2')
    
    # 分组
    dot.node('angvel', '[0-2]\n角速度\nangvel * 0.25', shape='box', style='filled,rounded', fillcolor='#E3F2FD')
    dot.node('gravity', '[3-5]\n重力投影\nprojected_gravity', shape='box', style='filled,rounded', fillcolor='#E8F5E9')
    dot.node('commands', '[6-8]\n命令\nvel_x, vel_y, angle', shape='box', style='filled,rounded', fillcolor='#FFF3E0')
    dot.node('dof_pos', '[9-16]\n关节位置\n(dof_pos - default) * scale', shape='box', style='filled,rounded', fillcolor='#F3E5F5')
    dot.node('dof_vel', '[17-24]\n关节速度\ndof_vel * 0.05', shape='box', style='filled,rounded', fillcolor='#E0F7FA')
    dot.node('last_action', '[25-32]\n上一动作\noutput_last', shape='box', style='filled,rounded', fillcolor='#FFEBEE')
    
    dot.edge('root', 'angvel', color='#757575')
    dot.edge('root', 'gravity', color='#757575')
    dot.edge('root', 'commands', color='#757575')
    dot.edge('root', 'dof_pos', color='#757575')
    dot.edge('root', 'dof_vel', color='#757575')
    dot.edge('root', 'last_action', color='#757575')
    
    dot.render('fsm_rl_observations', cleanup=True)
    print(f"✓ 观测值分解图已生成: fsm_rl_observations.png")
    
    return dot


if __name__ == '__main__':
    print("=" * 50)
    print("生成 FSMState_RL 工作流图")
    print("=" * 50)
    
    create_fsm_rl_workflow()
    create_rl_data_flow()
    create_observation_breakdown()
    
    print("=" * 50)
    print("所有图表已生成完成!")
    print("=" * 50)

#!/usr/bin/env python3
"""
生成 FSMState_RL 详细流程图
"""

from graphviz import Digraph

def create_detailed_flow():
    """详细流程图"""
    dot = Digraph(comment='FSMState_RL Detailed Flow', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    dot.attr('edge', fontname='Helvetica', fontsize='8')
    
    # ==================== 构造函数 ====================
    with dot.subgraph(name='cluster_ctor') as c:
        c.attr(label='构造函数 FSMState_RL()', style='rounded,filled', 
               fillcolor='#E3F2FD', color='#1565C0', fontname='Helvetica-Bold')
        
        c.node('ctor_cuda', '初始化 CUDA\nCudaTest("xx.engine")', 
               shape='box', style='filled,rounded', fillcolor='white')
        c.node('ctor_mem', '分配内存\ninput_0[31], input_1[310]\noutput[8], output_last[8]', 
               shape='box', style='filled,rounded', fillcolor='white')
    
    # ==================== enter() ====================
    with dot.subgraph(name='cluster_enter') as c:
        c.attr(label='enter() - 进入状态 (初始化)', style='rounded,filled',
               fillcolor='#E8F5E9', color='#2E7D32', fontname='Helvetica-Bold')
        
        c.node('enter_1', '1. 记录轮子初始位置\nwheel_init_pos_abs_[0]=q[3]\nwheel_init_pos_abs_[1]=q[7]', 
               shape='box', style='filled,rounded', fillcolor='white')
        c.node('enter_2', '2. 设置默认参数\naction_scale=0.5, lin_vel_scale=2.0\ndof_pos_scale=1.0, dof_vel_scale=0.05', 
               shape='box', style='filled,rounded', fillcolor='white')
        c.node('enter_3', '3. 清空历史缓冲区\ninput_1[310]=0, output_last[8]=0', 
               shape='box', style='filled,rounded', fillcolor='white')
        c.node('enter_4', '4. 预热填充 (10帧)\n_GetObs() + 滑动窗口', 
               shape='box', style='filled,rounded', fillcolor='#FFF3E0', penwidth='2')
        c.node('enter_5', '5. 预热推理 (10次)\n_Forward()', 
               shape='box', style='filled,rounded', fillcolor='#FFF3E0')
        c.node('enter_6', '6. 启动后台线程\nthread(_Run_Forward)', 
               shape='box', style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
    
    # ==================== run() 主循环 ====================
    with dot.subgraph(name='cluster_run') as c:
        c.attr(label='run() - 主控制循环 (500Hz)', style='rounded,filled',
               fillcolor='#FFF3E0', color='#E65100', fontname='Helvetica-Bold')
        
        c.node('run_1', '1. 读取遥控命令\nx_vel_cmd, y_vel_cmd, pitch_cmd', 
               shape='box', style='filled,rounded', fillcolor='white')
        c.node('run_2', '2. 清零命令缓冲区\nqd=0, qd_dot=0, kp=0, kd=0, tau_cmd=0', 
               shape='box', style='filled,rounded', fillcolor='white')
        c.node('run_3', '3. PD控制力矩计算\n轮子: 12*pos + 1.5*vel\n腿部: 60*pos_err + 2.0*vel', 
               shape='box', style='filled,rounded', fillcolor='#E1F5FE', penwidth='2')
        
        c.node('run_note', '注: desired_pos 由\n后台线程更新', shape='note', 
               style='filled', fillcolor='#FFF9C4', fontsize='8')
    
    # ==================== 后台线程 ====================
    with dot.subgraph(name='cluster_thread') as c:
        c.attr(label='_Run_Forward() - 后台推理线程 (100Hz)', style='rounded,filled',
               fillcolor='#F3E5F5', color='#6A1B9A', fontname='Helvetica-Bold')
        
        c.node('thread_loop', 'while(threadRunning)', shape='ellipse', 
               style='filled', fillcolor='white')
        c.node('thread_1', '1. 读取关节状态\nobs_.dof_pos[i] = q[i]\nobs_.dof_vel[i] = dq[i]\n轮子位置置零', 
               shape='box', style='filled,rounded', fillcolor='white')
        c.node('thread_2', '2. 调用 _Forward()\n神经网络推理', 
               shape='box', style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
        c.node('thread_3', '3. 处理网络输出\naction[j] = output[j]*scale + default', 
               shape='box', style='filled,rounded', fillcolor='white')
        c.node('thread_4', '4. 更新期望位置\ndesired_pos[i] = action[i]', 
               shape='box', style='filled,rounded', fillcolor='#C8E6C9', penwidth='2')
        c.node('thread_5', '5. 精确睡眠 10ms\nabsoluteWait(10000us)', 
               shape='box', style='filled,rounded', fillcolor='white')
    
    # ==================== _Forward ====================
    with dot.subgraph(name='cluster_forward') as c:
        c.attr(label='_Forward() - 单次推理', style='rounded,filled',
               fillcolor='#FFEBEE', color='#C62828', fontname='Helvetica-Bold')
        
        c.node('fwd_1', '_GetObs()\n构建31维观测', shape='box', 
               style='filled,rounded', fillcolor='white')
        c.node('fwd_2', 'CUDA推理\ndo_inference()', shape='box', 
               style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
        c.node('fwd_3', '滑动窗口更新\ninput_1[310]', shape='box', 
               style='filled,rounded', fillcolor='white')
        c.node('fwd_4', '保存输出\noutput_last[8] = output[8]', shape='box', 
               style='filled,rounded', fillcolor='white')
    
    # ==================== _GetObs ====================
    with dot.subgraph(name='cluster_getobs') as c:
        c.attr(label='_GetObs() - 构建观测 (31维)', style='rounded,filled',
               fillcolor='#E0F7FA', color='#00838F', fontname='Helvetica-Bold')
        
        c.node('obs_angvel', '角速度(3维)\nangvel * 0.25', shape='box', 
               style='filled,rounded', fillcolor='#E1F5FE')
        c.node('obs_gravity', '重力投影(3维)', shape='box', 
               style='filled,rounded', fillcolor='#E1F5FE')
        c.node('obs_cmd', '命令(3维)\nvel_x, vel_y, angle', shape='box', 
               style='filled,rounded', fillcolor='#E1F5FE')
        c.node('obs_pos', '关节位置(6维)\n❌ 跳过索引 3,7', shape='box', 
               style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
        c.node('obs_vel', '关节速度(8维)', shape='box', 
               style='filled,rounded', fillcolor='#C8E6C9')
        c.node('obs_last', '上一动作(8维)', shape='box', 
               style='filled,rounded', fillcolor='#C8E6C9')
    
    # ==================== 连接 ====================
    # 构造流程
    dot.edge('ctor_cuda', 'ctor_mem', style='invis')
    
    # enter流程
    dot.edge('ctor_mem', 'enter_1', style='dashed', color='#757575')
    dot.edge('enter_1', 'enter_2', style='invis')
    dot.edge('enter_2', 'enter_3', style='invis')
    dot.edge('enter_3', 'enter_4', style='invis')
    dot.edge('enter_4', 'enter_5', style='invis')
    dot.edge('enter_5', 'enter_6', style='invis')
    
    # run流程
    dot.edge('enter_6', 'run_1', style='dashed', color='#757575', label='周期调用')
    dot.edge('run_1', 'run_2', style='invis')
    dot.edge('run_2', 'run_3', style='invis')
    dot.edge('run_3', 'run_note', style='dashed', color='#F57C00', constraint='false')
    
    # 线程流程
    dot.edge('enter_6', 'thread_loop', style='dashed', color='#6A1B9A', label='启动线程')
    dot.edge('thread_loop', 'thread_1', style='invis')
    dot.edge('thread_1', 'thread_2', style='invis')
    dot.edge('thread_2', 'thread_3', style='invis')
    dot.edge('thread_3', 'thread_4', style='invis')
    dot.edge('thread_4', 'thread_5', style='invis')
    dot.edge('thread_5', 'thread_loop', style='dashed', color='#6A1B9A', label='循环')
    
    # _Forward流程
    dot.edge('thread_2', 'fwd_1', style='dashed', color='#C62828', label='调用')
    dot.edge('fwd_1', 'fwd_2', style='invis')
    dot.edge('fwd_2', 'fwd_3', style='invis')
    dot.edge('fwd_3', 'fwd_4', style='invis')
    
    # _GetObs构建
    dot.edge('fwd_1', 'obs_angvel', style='dashed', color='#00838F')
    dot.edge('obs_angvel', 'obs_gravity', style='invis')
    dot.edge('obs_gravity', 'obs_cmd', style='invis')
    dot.edge('obs_cmd', 'obs_pos', style='invis')
    dot.edge('obs_pos', 'obs_vel', style='invis')
    dot.edge('obs_vel', 'obs_last', style='invis')
    dot.edge('obs_last', 'fwd_2', style='dashed', color='#C62828', constraint='false')
    
    # 数据流向
    dot.edge('fwd_4', 'thread_3', style='dashed', color='#6A1B9A', constraint='false')
    dot.edge('thread_4', 'run_3', style='dashed', color='#E65100', 
             label='更新desired_pos', constraint='false')
    
    dot.render('fsm_rl_detailed_flow', cleanup=True)
    print(f"✓ 详细流程图已生成: fsm_rl_detailed_flow.png")
    
    return dot


def create_architecture_diagram():
    """架构图"""
    dot = Digraph(comment='Architecture', format='png')
    dot.attr(rankdir='LR', bgcolor='white', pad='0.5')
    
    # 输入
    with dot.subgraph(name='cluster_input') as c:
        c.attr(label='输入', style='rounded,filled', fillcolor='#E3F2FD')
        c.node('in_cmd', '遥控命令\n(x_vel, y_vel, yaw)')
        c.node('in_imu', 'IMU\n(gyro, accel)')
        c.node('in_joint', '关节状态\n(q[8], dq[8])')
    
    # 核心处理
    with dot.subgraph(name='cluster_core') as c:
        c.attr(label='FSMState_RL 核心', style='rounded,filled', 
               fillcolor='#F3E5F5', color='#6A1B9A')
        
        c.node('getobs', '_GetObs()\n31维观测', shape='box', 
               style='filled,rounded', fillcolor='white')
        c.node('buffer', 'History Buffer\ninput_1[310]', shape='cylinder', 
               style='filled', fillcolor='#E1F5FE')
        c.node('nn', 'TensorRT\nPolicy Network', shape='box', 
               style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
        c.node('pd', 'PD Controller', shape='box', 
               style='filled,rounded', fillcolor='#E8F5E9')
    
    # 输出
    with dot.subgraph(name='cluster_output') as c:
        c.attr(label='输出', style='rounded,filled', fillcolor='#FFEBEE')
        c.node('out_tau', '力矩\ntau_cmd[8]')
        c.node('out_state', '状态反馈\n(desired_pos)')
    
    # 线程
    dot.node('main_thread', '主线程\nrun() 500Hz', shape='ellipse', 
             style='filled', fillcolor='#FFF3E0')
    dot.node('bg_thread', '后台线程\n_Run_Forward() 100Hz', shape='ellipse', 
             style='filled', fillcolor='#E0F7FA')
    
    # 连接
    dot.edge('in_cmd', 'getobs', color='#F57C00')
    dot.edge('in_imu', 'getobs', color='#1976D2')
    dot.edge('in_joint', 'getobs', color='#1976D2')
    
    dot.edge('getobs', 'nn', label='input_0[31]', color='#C62828')
    dot.edge('buffer', 'nn', label='input_1[310]', color='#C62828')
    dot.edge('nn', 'buffer', label='更新', style='dashed', color='#6A1B9A')
    
    dot.edge('nn', 'out_state', label='output[8]', color='#388E3C')
    dot.edge('out_state', 'pd', color='#388E3C')
    dot.edge('pd', 'out_tau', color='#C62828', penwidth='2')
    
    dot.edge('main_thread', 'pd', style='dashed', color='#E65100')
    dot.edge('bg_thread', 'nn', style='dashed', color='#6A1B9A')
    
    dot.render('fsm_rl_architecture', cleanup=True)
    print(f"✓ 架构图已生成: fsm_rl_architecture.png")
    
    return dot


if __name__ == '__main__':
    print("=" * 50)
    print("生成 FSMState_RL 详细逻辑图")
    print("=" * 50)
    
    create_detailed_flow()
    create_architecture_diagram()
    
    print("=" * 50)
    print("所有图表已生成完成!")
    print("=" * 50)

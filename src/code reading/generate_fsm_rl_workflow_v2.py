#!/usr/bin/env python3
"""
生成 FSMState_RL 更新后的工作流图 (27维观测，去掉轮子位置)
"""

from graphviz import Digraph

def create_observation_breakdown_v2():
    """观测值分解图 (更新版)"""
    dot = Digraph(comment='Observation Breakdown v2', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    
    # 总维度
    dot.node('root', 'Observations[27]\n(原33维，去掉2个轮子位置/速度/动作)', 
             shape='box', style='filled,rounded', fillcolor='#1565C0', fontcolor='white', penwidth='2')
    
    # 分组 - 6组，每组6维
    dot.node('angvel', '[0-2] 角速度\nangvel * 0.25\n(3维)', shape='box', style='filled,rounded', fillcolor='#E3F2FD')
    dot.node('gravity', '[3-5] 重力投影\nprojected_gravity\n(3维)', shape='box', style='filled,rounded', fillcolor='#E8F5E9')
    dot.node('commands', '[6-8] 命令\nvel_x, vel_y, angle\n(3维)', shape='box', style='filled,rounded', fillcolor='#FFF3E0')
    dot.node('dof_pos', '[9-14] 关节位置(6维)\n索引: 0,1,2,4,5,6\n(skip 3,7轮子)', shape='box', style='filled,rounded', fillcolor='#F3E5F5')
    dot.node('dof_vel', '[15-20] 关节速度(6维)\n索引: 0,1,2,4,5,6\n(skip 3,7轮子)', shape='box', style='filled,rounded', fillcolor='#E0F7FA')
    dot.node('last_action', '[21-26] 上一动作(6维)\n索引: 0,1,2,4,5,6\n(skip 3,7轮子)', shape='box', style='filled,rounded', fillcolor='#FFEBEE')
    
    # 跳过的轮子
    dot.node('skip', '跳过的轮子关节\n索引 3, 7\n(共4维: 位置+速度+动作*2)', shape='box', style='filled,rounded', fillcolor='#FFCDD2', color='#C62828')
    
    dot.edge('root', 'angvel', color='#757575')
    dot.edge('root', 'gravity', color='#757575')
    dot.edge('root', 'commands', color='#757575')
    dot.edge('root', 'dof_pos', color='#757575')
    dot.edge('root', 'dof_vel', color='#757575')
    dot.edge('root', 'last_action', color='#757575')
    
    # 注释说明
    dot.edge('root', 'skip', style='dashed', color='#C62828', label='训练时去掉', fontcolor='#C62828')
    
    dot.render('fsm_rl_observations_v2', cleanup=True)
    print(f"✓ 更新后的观测值分解图已生成: fsm_rl_observations_v2.png")
    
    return dot


def create_rl_data_flow_v2():
    """RL 数据流图 (更新版)"""
    dot = Digraph(comment='RL Data Flow v2', format='png')
    dot.attr(rankdir='LR', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    
    # 输入
    dot.node('imu', 'IMU数据\n(gyro, accel)', shape='box', style='filled,rounded', fillcolor='#E3F2FD')
    dot.node('joint', '关节状态\n(q[8], dq[8])', shape='box', style='filled,rounded', fillcolor='#E3F2FD')
    dot.node('cmd', '遥控命令\n(x_vel, y_vel, yaw)', shape='box', style='filled,rounded', fillcolor='#FFF3E0')
    
    # 处理
    dot.node('estimator', '状态估计器\nStateEstimator\n(旋转矩阵, 角速度)', shape='box', style='filled,rounded', fillcolor='#E8F5E9')
    dot.node('getobs', '_GetObs()\n构建27维观测\n(skip轮子索引3,7)', shape='box', style='filled,rounded', fillcolor='#E0F7FA', penwidth='2')
    
    # 神经网络
    dot.node('buffer', '历史缓冲区\ninput_1[270]\n(27维 * 10帧)', shape='cylinder', style='filled', fillcolor='#F3E5F5')
    dot.node('nn', '神经网络\nTensorRT CUDA\nPolicy Network\n输入27维', shape='box', style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
    
    # 输出
    dot.node('action', '动作输出\noutput[8]\n(8维，包含轮子)', shape='box', style='filled,rounded', fillcolor='#FFEBEE')
    dot.node('pd', 'PD控制\n(位置环+阻尼)', shape='box', style='filled,rounded', fillcolor='#FFF8E1')
    dot.node('torque', '力矩输出\ntau_cmd[8]', shape='box', style='filled,rounded', fillcolor='#C8E6C9', penwidth='2')
    
    # 关键变化说明
    dot.node('note', '关键变化:\n观测只使用6维腿部关节\n但网络仍输出8维动作\n(包含轮子控制)', 
             shape='note', style='filled', fillcolor='#FFF9C4', fontsize='8')
    
    # 连接
    dot.edge('imu', 'estimator', color='#1976D2')
    dot.edge('joint', 'getobs', color='#1976D2')
    dot.edge('estimator', 'getobs', color='#1976D2')
    dot.edge('cmd', 'getobs', color='#F57C00')
    
    dot.edge('getobs', 'nn', label='input_0[27]', color='#C62828', penwidth='2')
    dot.edge('buffer', 'nn', label='input_1[270]', color='#C62828')
    dot.edge('nn', 'buffer', label='更新历史', style='dashed', color='#6A1B9A')
    
    dot.edge('nn', 'action', color='#C62828')
    dot.edge('action', 'pd', color='#F9A825')
    dot.edge('joint', 'pd', label='反馈', style='dashed', color='#1976D2')
    dot.edge('pd', 'torque', color='#388E3C', penwidth='2')
    
    dot.edge('nn', 'note', style='invis')
    
    dot.render('fsm_rl_dataflow_v2', cleanup=True)
    print(f"✓ 更新后的数据流图已生成: fsm_rl_dataflow_v2.png")
    
    return dot


def create_code_changes_summary():
    """代码修改摘要图"""
    dot = Digraph(comment='Code Changes Summary', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    
    # 标题
    dot.attr(label='FSMState_RL 代码修改摘要\n观测维度: 33 → 27 (去掉轮子关节 3,7)', fontname='Helvetica-Bold', fontsize='14')
    
    # 修改前
    with dot.subgraph(name='cluster_before') as c:
        c.attr(label='修改前 (33维)', style='rounded,filled', fillcolor='#FFEBEE', color='#C62828')
        c.node('before1', 'input_0[33]', shape='box', style='filled', fillcolor='white')
        c.node('before2', 'input_1[330] (33*10)', shape='box', style='filled', fillcolor='white')
        c.node('before3', 'input_1_temp[297]', shape='box', style='filled', fillcolor='white')
        c.node('before4', '关节位置: 8维', shape='box', style='filled', fillcolor='white')
        c.node('before5', '关节速度: 8维', shape='box', style='filled', fillcolor='white')
        c.node('before6', '上一动作: 8维', shape='box', style='filled', fillcolor='white')
    
    # 修改后
    with dot.subgraph(name='cluster_after') as c:
        c.attr(label='修改后 (27维)', style='rounded,filled', fillcolor='#E8F5E9', color='#2E7D32')
        c.node('after1', 'input_0[27]', shape='box', style='filled', fillcolor='#C8E6C9')
        c.node('after2', 'input_1[270] (27*10)', shape='box', style='filled', fillcolor='#C8E6C9')
        c.node('after3', 'input_1_temp[243]', shape='box', style='filled', fillcolor='#C8E6C9')
        c.node('after4', '关节位置: 6维\n(skip 3,7)', shape='box', style='filled', fillcolor='#C8E6C9')
        c.node('after5', '关节速度: 6维\n(skip 3,7)', shape='box', style='filled', fillcolor='#C8E6C9')
        c.node('after6', '上一动作: 6维\n(skip 3,7)', shape='box', style='filled', fillcolor='#C8E6C9')
    
    # 不变的
    with dot.subgraph(name='cluster_same') as c:
        c.attr(label='保持不变', style='rounded,filled', fillcolor='#E3F2FD', color='#1565C0')
        c.node('same1', 'output[8]\n网络仍输出8维', shape='box', style='filled', fillcolor='white')
        c.node('same2', 'output_last[8]', shape='box', style='filled', fillcolor='white')
        c.node('same3', 'obs_.dof_pos[8]', shape='box', style='filled', fillcolor='white')
        c.node('same4', 'obs_.dof_vel[8]', shape='box', style='filled', fillcolor='white')
    
    # 注释
    dot.node('note1', '修改位置:\n1. 构造函数: 内存分配\n2. enter(): 缓冲区清空\n3. _GetObs(): 跳过索引3,7\n4. _Forward(): 滑动窗口', 
             shape='note', style='filled', fillcolor='#FFF9C4', fontsize='8')
    
    dot.render('fsm_rl_changes_summary', cleanup=True)
    print(f"✓ 代码修改摘要图已生成: fsm_rl_changes_summary.png")
    
    return dot


if __name__ == '__main__':
    print("=" * 50)
    print("生成 FSMState_RL 更新后的工作流图")
    print("(观测维度: 33 → 27，去掉轮子关节)")
    print("=" * 50)
    
    create_observation_breakdown_v2()
    create_rl_data_flow_v2()
    create_code_changes_summary()
    
    print("=" * 50)
    print("所有更新后的图表已生成完成!")
    print("=" * 50)

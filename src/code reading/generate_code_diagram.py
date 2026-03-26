#!/usr/bin/env python3
"""
生成 TITA 机器人项目的代码架构图
"""

from graphviz import Digraph
import os

def create_code_diagram():
    # 创建有向图
    dot = Digraph(comment='TITA Robot Code Architecture', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='10')
    dot.attr('edge', fontname='Helvetica', fontsize='9')

    # ==================== 颜色定义 ====================
    colors = {
        'command': '#E1F5FE',      # 浅蓝 - 命令层
        'controller': '#F3E5F5',    # 浅紫 - 控制器
        'device': '#E8F5E9',        # 浅绿 - 设备层
        'interaction': '#FFF3E0',   # 浅橙 - 交互层
        'simulation': '#ECEFF1',    # 浅灰 - 仿真层
        'utils': '#FCE4EC',         # 浅粉 - 工具
        'fsm': '#E0F2F1',           # 浅青 - 状态机
        'border': '#37474F'         # 边框色
    }

    # ==================== tita_command 子图 ====================
    with dot.subgraph(name='cluster_command') as c:
        c.attr(label='tita_command [命令层]', style='rounded,filled', 
               fillcolor=colors['command'], color=colors['border'], fontname='Helvetica-Bold')
        
        c.node('CM', 'command_manager\n命令管理器', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('AC', 'active_command\n主动命令', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('PC', 'passive_command\n被动命令', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('SDK', 'sdk_command\nSDK命令', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('TC_cmd', 'teleop_command\n远程操作', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])

    # ==================== tita_locomotion - Controller 子图 ====================
    with dot.subgraph(name='cluster_controller') as c:
        c.attr(label='tita_controller [主控制器]', style='rounded,filled',
               fillcolor=colors['controller'], color=colors['border'], fontname='Helvetica-Bold')
        
        c.node('TCN', 'TitaController\n主控制器节点', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'], peripheries='2')
        c.node('OE', 'OrientationEstimator\n姿态估计器', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('PC_pub', 'PlanCommands\n规划命令发布', shape='ellipse', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('RS_pub', 'RobotStates\n机器人状态发布', shape='ellipse', style='filled,rounded',
               fillcolor='white', color=colors['border'])

    # ==================== FSM 子图 ====================
    with dot.subgraph(name='cluster_fsm') as c:
        c.attr(label='FSM [有限状态机]', style='rounded,filled',
               fillcolor=colors['fsm'], color=colors['border'], fontname='Helvetica-Bold')
        
        c.node('FSM', 'FSM\n状态机控制器', shape='diamond', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('PASSIVE', 'PASSIVE\n被动状态', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('RL', 'RL\n强化学习模式', shape='box', style='filled,rounded',
               fillcolor='#FFECB3', color=colors['border'], penwidth='2')
        c.node('RECOVERY', 'RECOVERY_STAND\n恢复站立', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('JOINTPD', 'JOINT_PD\n关节PD控制', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('TRANSFORM', 'TRANSFORM_DOWN\n降下', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])

    # ==================== Devices 子图 ====================
    with dot.subgraph(name='cluster_devices') as c:
        c.attr(label='devices [设备层]', style='rounded,filled',
               fillcolor=colors['device'], color=colors['border'], fontname='Helvetica-Bold')
        
        c.node('TR', 'tita_robot\n硬件接口', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('CR', 'can_receiver\nCAN接收', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('CS', 'can_sender\nCAN发送', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('HB', 'hw_broadcaster\n硬件广播', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('HWB', 'hardware_bridge\n硬件桥接', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])

    # ==================== Interaction 子图 ====================
    with dot.subgraph(name='cluster_interaction') as c:
        c.attr(label='interaction [交互层]', style='rounded,filled',
               fillcolor=colors['interaction'], color=colors['border'], fontname='Helvetica-Bold')
        
        c.node('JC', 'joy_controller\n手柄控制', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('KC', 'keyboard_controller\n键盘控制', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])

    # ==================== Simulation 子图 ====================
    with dot.subgraph(name='cluster_simulation') as c:
        c.attr(label='tita_webots_ros2 [仿真层]', style='rounded,filled',
               fillcolor=colors['simulation'], color=colors['border'], fontname='Helvetica-Bold')
        
        c.node('WB', 'webots_bridge\nWebots桥接', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('GB', 'gazebo_bridge\nGazebo桥接', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])

    # ==================== tita_bringup 子图 ====================
    with dot.subgraph(name='cluster_bringup') as c:
        c.attr(label='tita_bringup [工具与启动]', style='rounded,filled',
               fillcolor=colors['utils'], color=colors['border'], fontname='Helvetica-Bold')
        
        c.node('UTILS', 'tita_utils\n工具库', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])
        c.node('TOPICS', 'topic_names\nTopic定义', shape='box', style='filled,rounded',
               fillcolor='white', color=colors['border'])

    # ==================== 边定义 ====================
    # 命令流
    dot.edge('TC_cmd', 'CM', label='/command/teleop/command', color='#1976D2', fontcolor='#1976D2')
    dot.edge('AC', 'CM', label='/command/active/command', color='#1976D2', fontcolor='#1976D2')
    dot.edge('PC', 'CM', label='/command/passive/command', color='#1976D2', fontcolor='#1976D2')
    dot.edge('SDK', 'CM', label='/command/user/command', color='#1976D2', fontcolor='#1976D2')
    
    # Manager 到 Controller
    dot.edge('CM', 'TCN', label='/command/manager/\ncmd_twist, cmd_pose, cmd_key', color='#D32F2F', fontcolor='#D32F2F', penwidth='2')
    
    # 交互到 Controller
    dot.edge('JC', 'TCN', label='joy', color='#F57C00', fontcolor='#F57C00')
    dot.edge('KC', 'TCN', label='key', color='#F57C00', fontcolor='#F57C00')
    
    # Controller 内部
    dot.edge('TCN', 'FSM', label='control', color='#388E3C', fontcolor='#388E3C')
    dot.edge('TCN', 'OE', label='estimate', color='#388E3C', fontcolor='#388E3C')
    dot.edge('TCN', 'PC_pub', label='publish', color='#7B1FA2', fontcolor='#7B1FA2')
    dot.edge('TCN', 'RS_pub', label='publish', color='#7B1FA2', fontcolor='#7B1FA2')
    
    # FSM 状态转换
    dot.edge('FSM', 'PASSIVE', style='dashed', color='#607D8B')
    dot.edge('FSM', 'RL', style='dashed', color='#607D8B')
    dot.edge('FSM', 'RECOVERY', style='dashed', color='#607D8B')
    dot.edge('FSM', 'JOINTPD', style='dashed', color='#607D8B')
    dot.edge('FSM', 'TRANSFORM', style='dashed', color='#607D8B')
    
    # Controller 到 Hardware
    dot.edge('TCN', 'HWB', label='ros2_control', color='#00796B', fontcolor='#00796B', penwidth='2')
    dot.edge('HWB', 'TR', label='socket_can', color='#00796B', fontcolor='#00796B')
    dot.edge('TR', 'CR', label='CAN', color='#5D4037', fontcolor='#5D4037')
    dot.edge('TR', 'CS', label='CAN', color='#5D4037', fontcolor='#5D4037')
    dot.edge('TR', 'HB', label='broadcast', color='#5D4037', fontcolor='#5D4037')
    
    # 仿真
    dot.edge('TCN', 'WB', label='ros2_control', style='dashed', color='#78909C', fontcolor='#78909C')
    dot.edge('TCN', 'GB', label='ros2_control', style='dashed', color='#78909C', fontcolor='#78909C')
    
    # 工具
    dot.edge('UTILS', 'CM', style='dotted', color='#C2185B', constraint='false')
    dot.edge('UTILS', 'TCN', style='dotted', color='#C2185B', constraint='false')
    dot.edge('TOPICS', 'CM', style='dotted', color='#C2185B', constraint='false')
    dot.edge('TOPICS', 'TCN', style='dotted', color='#C2185B', constraint='false')

    # 保存
    dot.render('tita_code_architecture', cleanup=True)
    print(f"✓ 代码架构图已生成: tita_code_architecture.png")
    
    return dot


def create_fsm_state_diagram():
    """生成 FSM 状态机图"""
    dot = Digraph(comment='FSM State Machine', format='png')
    dot.attr(rankdir='LR', bgcolor='white', pad='0.5')
    
    # 状态节点
    dot.node('START', '*', shape='circle', style='filled', fillcolor='black', width='0.3', height='0.3', fontcolor='white')
    dot.node('PASSIVE', 'PASSIVE\n被动/待机', shape='box', style='rounded,filled', fillcolor='#E3F2FD')
    dot.node('RECOVERY', 'RECOVERY_STAND\n恢复站立', shape='box', style='rounded,filled', fillcolor='#E8F5E9')
    dot.node('RL', 'RL\n强化学习模式', shape='box', style='rounded,filled', fillcolor='#FFF3E0', penwidth='3')
    dot.node('JOINTPD', 'JOINT_PD\n关节控制', shape='box', style='rounded,filled', fillcolor='#F3E5F5')
    dot.node('TRANSFORM', 'TRANSFORM_DOWN\n降下', shape='box', style='rounded,filled', fillcolor='#FFEBEE')
    
    # 边
    dot.edge('START', 'PASSIVE')
    dot.edge('PASSIVE', 'RECOVERY', label='启动/恢复')
    dot.edge('PASSIVE', 'JOINTPD', label='关节测试')
    dot.edge('RECOVERY', 'RL', label='站立完成', penwidth='2', color='#FF6F00')
    dot.edge('RECOVERY', 'TRANSFORM', label='关闭')
    dot.edge('RL', 'TRANSFORM', label='降下/关机')
    dot.edge('RL', 'JOINTPD', label='关节模式')
    dot.edge('RL', 'PASSIVE', label='紧急停止', style='dashed', color='#D32F2F')
    dot.edge('JOINTPD', 'PASSIVE', label='停止')
    dot.edge('TRANSFORM', 'PASSIVE', label='完成')
    
    dot.render('tita_fsm_diagram', cleanup=True)
    print(f"✓ FSM状态图已生成: tita_fsm_diagram.png")
    
    return dot


def create_class_diagram():
    """生成核心类图"""
    dot = Digraph(comment='Core Class Diagram', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    
    # TitaController
    dot.node('TitaController', '''{TitaController|+ joints_ : vector\<Joint\>\l+ FSMController_ : shared_ptr\<FSM\>\l+ controlData_ : shared_ptr\<ControlFSMData\>\l|+ on_init()\l+ on_configure()\l+ on_activate()\l+ update()\l+ cmd_vel_cb()\l+ joy_cb()\l+ setup_controller()\l+ lqr_loop_thread()\l}''', 
    shape='record', style='filled', fillcolor='#E3F2FD')
    
    # FSM
    dot.node('FSM', '''{FSM|– _currentState : FSMState*\l– _nextState : FSMState*\l– _stateList : FSMStateList\l|+ run()\l+ getNextState()\l+ checkSafty()\l}''',
    shape='record', style='filled', fillcolor='#FFF3E0')
    
    # FSMState (abstract)
    dot.node('FSMState', '''{FSMState|+ _stateName : FSMStateName\l+ _stateNameStr : string\l|+ enter()\l+ run()\l+ exit()\l+ checkTransition()\l}''',
    shape='record', style='filled,dashed', fillcolor='#ECEFF1')
    
    # Concrete states
    states = ['FSMState_RL', 'FSMState_Passive', 'FSMState_RecoveryStand', 
              'FSMState_JointPD', 'FSMState_TransformDown']
    for state in states:
        dot.node(state, state.replace('FSMState_', ''), shape='record', style='filled', fillcolor='white')
    
    # CommandManagerNode
    dot.node('CMN', '''{CommandManagerNode|– active_cmd_subscription_\l– passive_cmd_subscription_\l– body_twist_publisher_\l|+ active_callback()\l+ passive_callback()\l+ joy_callback()\l}''',
    shape='record', style='filled', fillcolor='#E8F5E9')
    
    # ControlFSMData
    dot.node('CFD', '''{ControlFSMData|+ low_state\l+ low_cmd\l+ state_estimator\l+ state_command\l+ wheel_legged_data\l+ params\l}''',
    shape='record', style='filled', fillcolor='#F3E5F5')
    
    # 关系
    dot.edge('TitaController', 'FSM', label='uses', arrowhead='open', color='#1976D2')
    dot.edge('TitaController', 'CFD', label='owns', arrowhead='diamond', color='#D32F2F')
    dot.edge('FSM', 'FSMState', label='manages', arrowhead='open', color='#1976D2')
    
    for state in states:
        dot.edge('FSMState', state, label='inherits', arrowhead='empty', style='dashed', color='#757575')
    
    dot.render('tita_class_diagram', cleanup=True)
    print(f"✓ 类图已生成: tita_class_diagram.png")
    
    return dot


def create_data_flow_diagram():
    """生成数据流图"""
    dot = Digraph(comment='Data Flow', format='png')
    dot.attr(rankdir='LR', bgcolor='white', pad='0.5')
    
    # 节点样式
    dot.attr('node', shape='box', style='rounded,filled', fontname='Helvetica', fontsize='9')
    
    # 输入
    dot.node('Joy', '手柄输入', fillcolor='#E3F2FD')
    dot.node('SDK', 'SDK输入', fillcolor='#E3F2FD')
    dot.node('Passive', '被动控制', fillcolor='#E3F2FD')
    
    # 处理
    dot.node('CM', 'CommandManager', fillcolor='#FFF3E0', penwidth='2')
    dot.node('TC', 'TitaController', fillcolor='#F3E5F5', penwidth='2')
    dot.node('FSM', 'FSM状态机', fillcolor='#E8F5E9')
    
    # 输出
    dot.node('HW', 'Hardware\n硬件', fillcolor='#FFEBEE')
    dot.node('Topic', 'RobotStates\nPlanCommands', fillcolor='#E0F2F1', shape='ellipse')
    
    # 数据流向
    dot.edge('Joy', 'CM', label='Joy msg')
    dot.edge('SDK', 'CM', label='LocomotionCmd')
    dot.edge('Passive', 'CM', label='限制/安全模式')
    
    dot.edge('CM', 'TC', label='Twist/Pose/String')
    
    dot.edge('TC', 'FSM', label='state/command')
    dot.edge('FSM', 'TC', label='torque_cmd')
    
    dot.edge('TC', 'HW', label='joint cmd')
    dot.edge('HW', 'TC', label='IMU/joint_states', style='dashed')
    
    dot.edge('TC', 'Topic', label='publish', style='dashed', color='#00796B')
    
    dot.render('tita_data_flow', cleanup=True)
    print(f"✓ 数据流图已生成: tita_data_flow.png")
    
    return dot


if __name__ == '__main__':
    print("=" * 50)
    print("生成 TITA 机器人项目代码图")
    print("=" * 50)
    
    create_code_diagram()
    create_fsm_state_diagram()
    create_class_diagram()
    create_data_flow_diagram()
    
    print("=" * 50)
    print("所有图表已生成完成!")
    print("=" * 50)

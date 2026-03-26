#!/usr/bin/env python3
"""
生成 FSMState_RL 正确的工作流图
关节位置: 6维(去掉轮子) | 关节速度: 8维 | 上一动作: 8维
"""

from graphviz import Digraph

def create_observation_correct():
    """正确的观测值分解图"""
    dot = Digraph(comment='Observation Correct', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5')
    dot.attr('node', fontname='Helvetica', fontsize='9')
    
    # 总维度
    dot.node('root', 'Observations[31]', 
             shape='box', style='filled,rounded', fillcolor='#1565C0', fontcolor='white', penwidth='2')
    
    # 各组分
    dot.node('angvel', '[0-2]\n角速度 (3维)\nangvel * 0.25', shape='box', style='filled,rounded', fillcolor='#E3F2FD')
    dot.node('gravity', '[3-5]\n重力投影 (3维)\nprojected_gravity', shape='box', style='filled,rounded', fillcolor='#E8F5E9')
    dot.node('commands', '[6-8]\n命令 (3维)\nvel_x, vel_y, angle', shape='box', style='filled,rounded', fillcolor='#FFF3E0')
    
    # 关键变化：关节位置6维
    dot.node('dof_pos', '[9-14]\n关节位置 (6维)\n索引: 0,1,2,4,5,6\n❌ 跳过轮子: 3,7', 
             shape='box', style='filled,rounded', fillcolor='#FFCDD2', penwidth='2')
    
    # 保持8维
    dot.node('dof_vel', '[15-22]\n关节速度 (8维)\n0,1,2,3,4,5,6,7', shape='box', style='filled,rounded', fillcolor='#C8E6C9')
    dot.node('last_action', '[23-30]\n上一动作 (8维)\n0,1,2,3,4,5,6,7', shape='box', style='filled,rounded', fillcolor='#C8E6C9')
    
    # 跳过的轮子注释
    dot.node('skip_note', '轮子关节索引:\n3 = joint_left_leg_4\n7 = joint_right_leg_4\n(来自 RobotParameters.h)', 
             shape='note', style='filled', fillcolor='#FFF9C4', fontsize='8')
    
    dot.edge('root', 'angvel', color='#757575')
    dot.edge('root', 'gravity', color='#757575')
    dot.edge('root', 'commands', color='#757575')
    dot.edge('root', 'dof_pos', color='#C62828', penwidth='2')
    dot.edge('root', 'dof_vel', color='#757575')
    dot.edge('root', 'last_action', color='#757575')
    
    dot.edge('dof_pos', 'skip_note', style='dashed', color='#C62828', constraint='false')
    
    dot.render('fsm_rl_observations_correct', cleanup=True)
    print(f"✓ 正确的观测值分解图已生成: fsm_rl_observations_correct.png")
    
    return dot


def create_summary_table():
    """修改对照表"""
    dot = Digraph(comment='Summary Table', format='png')
    dot.attr(rankdir='TB', bgcolor='white', pad='0.5')
    
    # 使用表格形式
    dot.attr(label='FSMState_RL 观测维度修改对照表', fontname='Helvetica-Bold', fontsize='14')
    
    # 表头
    dot.node('header', '观测项 | 原维度 | 修改后 | 说明', shape='none', fontsize='10', fontname='Helvetica-Bold')
    
    # 各行
    dot.node('row1', '角速度     | 3  | 3  | 不变', shape='none', fontsize='9')
    dot.node('row2', '重力投影   | 3  | 3  | 不变', shape='none', fontsize='9')
    dot.node('row3', '命令       | 3  | 3  | 不变', shape='none', fontsize='9')
    dot.node('row4', '关节位置   | 8  | 6  | ❌ 去掉轮子索引 3,7', shape='none', fontsize='9', fontcolor='#C62828')
    dot.node('row5', '关节速度   | 8  | 8  | ✅ 保持8维', shape='none', fontsize='9', fontcolor='#2E7D32')
    dot.node('row6', '上一动作   | 8  | 8  | ✅ 保持8维', shape='none', fontsize='9', fontcolor='#2E7D32')
    dot.node('row7', '─────────────────────────────', shape='none', fontsize='9')
    dot.node('row8', '总计       | 33 | 31 | input_0[31], input_1[310]', shape='none', fontsize='9', fontname='Helvetica-Bold')
    
    # 连接
    for i in range(1, 9):
        dot.edge('header', f'row{i}', style='invis')
        if i < 8:
            dot.edge(f'row{i}', f'row{i+1}', style='invis')
    
    dot.render('fsm_rl_summary_table', cleanup=True)
    print(f"✓ 修改对照表已生成: fsm_rl_summary_table.png")
    
    return dot


def create_joint_index_map():
    """关节索引映射图"""
    dot = Digraph(comment='Joint Index Map', format='png')
    dot.attr(rankdir='LR', bgcolor='white', pad='0.5')
    
    dot.attr(label='关节索引映射 (来自 RobotParameters.h)', fontname='Helvetica-Bold', fontsize='12')
    
    # 8个关节
    for i in range(8):
        if i == 3 or i == 7:
            # 轮子关节 - 红色
            dot.node(f'j{i}', f'索引 {i}\n轮子关节\njoint_xxx_leg_4', 
                    shape='box', style='filled,rounded', fillcolor='#FFCDD2', color='#C62828', penwidth='2')
        else:
            # 腿部关节 - 绿色
            dot.node(f'j{i}', f'索引 {i}\n腿部关节', 
                    shape='box', style='filled,rounded', fillcolor='#C8E6C9')
    
    # 连接
    for i in range(7):
        dot.edge(f'j{i}', f'j{i+1}', style='invis')
    
    # 图例
    dot.node('legend1', '轮子关节\n(去掉位置观测)', shape='box', style='filled,rounded', fillcolor='#FFCDD2')
    dot.node('legend2', '腿部关节\n(保留)', shape='box', style='filled,rounded', fillcolor='#C8E6C9')
    
    dot.edge('j7', 'legend1', style='invis')
    
    dot.render('fsm_rl_joint_index_map', cleanup=True)
    print(f"✓ 关节索引映射图已生成: fsm_rl_joint_index_map.png")
    
    return dot


if __name__ == '__main__':
    print("=" * 50)
    print("生成 FSMState_RL 正确的代码图")
    print("=" * 50)
    
    create_observation_correct()
    create_summary_table()
    create_joint_index_map()
    
    print("=" * 50)
    print("所有图表已生成完成!")
    print("=" * 50)

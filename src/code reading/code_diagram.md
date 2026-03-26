# TITA 机器人项目代码架构图

## 项目总览

这是一个 ROS2 机器人控制系统，用于 TITA 轮腿机器人的运动控制和命令管理。

```mermaid
graph TB
    subgraph "tita_command [命令层]"
        CM[command_manager<br/>命令管理器]
        AC[active_command<br/>主动命令]
        PC[passive_command<br/>被动命令]
        SDK[sdk_command<br/>SDK命令]
        TC[teleop_command<br/>远程操作命令]
    end

    subgraph "tita_locomotion [运动控制层]"
        subgraph "Controller [主控制器]"
            TCN[tita_controller_node<br/>主控制器节点]
            FSM[FSM 状态机]
            OE[OrientationEstimator<br/>姿态估计器]
        end
        
        subgraph "Devices [设备层]"
            CR[can_receiver<br/>CAN接收]
            CS[can_sender<br/>CAN发送]
            TR[tita_robot<br/>机器人硬件接口]
            HB[hw_broadcaster<br/>硬件广播]
        end
        
        subgraph "Interaction [交互层]"
            JC[joy_controller<br/>手柄控制]
            KC[keyboard_controller<br/>键盘控制]
        end
        
        subgraph "Simulation [仿真层]"
            WB[webots_bridge<br/>Webots桥接]
            GB[gazebo_bridge<br/>Gazebo桥接]
        end
    end

    subgraph "tita_bringup [启动与工具]"
        LU[tita_utils<br/>工具库]
        Launch[launch files<br/>启动文件]
    end

    %% 命令流
    TC -->|joy| CM
    AC -->|LocomotionCmd| CM
    PC -->|LocomotionCmd| CM
    SDK -->|LocomotionCmd| CM
    
    CM -->|cmd_twist/cmd_pose/cmd_key| TCN
    JC -->|joy| TCN
    
    %% 控制流
    TCN --> FSM
    FSM --> OE
    TCN <-->|硬件接口| TR
    TR <-->|CAN| CR
    TR <-->|CAN| CS
    TR --> HB
    
    %% 仿真
    TCN <-->|ros2_control| WB
    TCN <-->|ros2_control| GB
    
    %% 工具
    LU -.->|topics| CM
    LU -.->|topics| TCN
```

## FSM 状态机详解

```mermaid
stateDiagram-v2
    [*] --> Passive
    Passive --> RecoveryStand : 启动/恢复
    Passive --> JointPD : 关节控制
    
    RecoveryStand --> RL : 站立完成 → 强化学习模式
    RecoveryStand --> TransformDown : 关闭
    
    RL --> TransformDown : 降下
    RL --> JointPD : 关节控制
    RL --> Passive : 紧急停止
    
    TransformDown --> Passive : 完成
    JointPD --> Passive
    
    note right of RL
        主要运动模式:
        - 轮式移动
        - 平衡控制
        - LQR优化
    end note
```

## 类图

### 主控制器类

```mermaid
classDiagram
    class TitaController {
        +vector~Joint~ joints_
        +shared_ptr~FSM~ FSMController_
        +shared_ptr~ControlFSMData~ controlData_
        +on_init()
        +on_configure()
        +on_activate()
        +update()
        +cmd_vel_cb()
        +joy_cb()
        +setup_controller()
        +lqr_loop_thread()
    }
    
    class FSM {
        -FSMState* _currentState
        -FSMState* _nextState
        -FSMStateList _stateList
        +run()
        +getNextState()
        +checkSafty()
    }
    
    class FSMState {
        <<abstract>>
        +FSMStateName _stateName
        +string _stateNameStr
        +enter()
        +run()
        +exit()
        +checkTransition()
    }
    
    class FSMState_RL {
        +run()
        +checkTransition()
    }
    
    class FSMState_Passive {
        +run()
    }
    
    class FSMState_RecoveryStand {
        +run()
    }
    
    class ControlFSMData {
        +shared_ptr~LowState~ low_state
        +shared_ptr~LowCommand~ low_cmd
        +shared_ptr~StateEstimator~ state_estimator
        +shared_ptr~StateCommand~ state_command
        +shared_ptr~WheelLeggedData~ wheel_legged_data
        +shared_ptr~Parameters~ params
    }
    
    TitaController --> FSM
    TitaController --> ControlFSMData
    FSM --> FSMState
    FSMState <|-- FSMState_RL
    FSMState <|-- FSMState_Passive
    FSMState <|-- FSMState_RecoveryStand
    FSMState <|-- FSMState_JointPD
    FSMState <|-- FSMState_TransformDown
```

### 命令管理类

```mermaid
classDiagram
    class CommandManagerNode {
        +active_callback()
        +passive_callback()
        +joy_callback()
        -Publisher body_pose_publisher_
        -Publisher body_twist_publisher_
        -Publisher body_fsm_publisher_
    }
    
    class ActiveCommandNode {
        +state_machine()
        +handle_user_btn()
    }
    
    class PassiveCommandNode {
        +velocity_control()
    }
    
    class SDKCommandNode {
        +user_function()
        +state_machine()
    }
    
    class TeleopCommandNode {
        +elrs_emwave_device()
        +button_function()
    }
    
    CommandManagerNode --> ActiveCommandNode : 订阅
    CommandManagerNode --> PassiveCommandNode : 订阅
    CommandManagerNode --> SDKCommandNode : 订阅
    CommandManagerNode --> TeleopCommandNode : 订阅
```

## 设备通信架构

```mermaid
graph LR
    subgraph "Hardware Interface"
        HR[tita_robot]
        CR[can_receiver]
        CS[can_sender]
        CU[can_utils]
    end
    
    subgraph "ROS2 Control"
        HWB[hardware_bridge]
        HWBC[hw_broadcaster]
        CC[commands_can]
    end
    
    subgraph "Controllers"
        TC[tita_controller]
    end
    
    TC <-->|LoanedInterface| HWB
    HWB <-->|socket_can| HR
    HR <-->|CAN Frame| CR
    HR <-->|CAN Frame| CS
    HWBC -->|joint_states| TC
    CC <-->|command| HR
```

## 数据流向

```mermaid
sequenceDiagram
    participant User
    participant Teleop as teleop_command
    participant CM as command_manager
    participant TC as tita_controller
    participant FSM as FSM状态机
    participant HW as Hardware
    
    User->>Teleop: 手柄/键盘输入
    Teleop->>CM: /command/teleop/command
    
    alt SDK控制
        User->>SDK: API调用
        SDK->>CM: /command/user/command
    end
    
    alt 被动控制
        PC->>CM: /command/passive/command<br/>速度限制/安全模式
    end
    
    CM->>TC: /command/manager/cmd_twist<br/>/command/manager/cmd_pose<br/>/command/manager/cmd_key
    
    TC->>FSM: 状态更新
    FSM->>FSM: run() / checkTransition()
    
    alt RL模式
        FSM->>FSM: LQR优化
        FSM->>FSM: WBC全身控制
    end
    
    TC->>HW: 关节指令 (q, dq, tau, kp, kd)
    HW->>TC: 传感器反馈 (IMU, 关节状态)
    TC->>CM: /locomotion/body/fsm_mode
```

## 关键 Topic 通信图

```mermaid
graph LR
    subgraph "Commands [输入命令]"
        T1[/command/teleop/command\n手柄命令/]
        T2[/command/active/command\n主动命令/]
        T3[/command/passive/command\n被动命令/]
        T4[/command/user/command\n用户命令/]
    end
    
    subgraph "Manager [命令管理]"
        M1[/command/manager/cmd_twist\n速度命令/]
        M2[/command/manager/cmd_pose\n位姿命令/]
        M3[/command/manager/cmd_key\nFSM切换命令/]
    end
    
    subgraph "Controller [控制器]"
        C1[/imu_sensor_broadcaster/imu\nIMU数据/]
        C2[/joint_states\n关节状态/]
        C3[/locomotion/body/fsm_mode\n当前状态/]
        C4[~/plan_commands\n规划命令/]
        C5[~/robot_states\n机器人状态/]
    end
    
    T1 --> M1
    T2 --> M1
    T3 --> M1
    T4 --> M1
    
    M1 --> TC
    M2 --> TC
    M3 --> TC
    
    TC --> C3
    TC --> C4
    TC --> C5
```

## 模块目录结构

```
tita_rl_sim2sim2real/src/
├── tita_locomotion/                 # 运动控制模块
│   ├── tita_controllers/
│   │   └── tita_controller/         # 主控制器
│   │       ├── include/
│   │       │   ├── tita_controller/
│   │       │   ├── fsm/            # 状态机
│   │       │   ├── estimator/      # 估计器
│   │       │   └── planner/        # 规划器
│   │       └── src/
│   ├── devices/                     # 硬件设备
│   │   ├── commands_can/           # CAN命令
│   │   ├── hardware_bridge/        # 硬件桥接
│   │   ├── hw_broadcaster/         # 硬件广播
│   │   └── tita_robot/             # 机器人硬件接口
│   ├── interaction/                 # 交互
│   │   ├── joy_controller/         # 手柄控制
│   │   └── keyboard_controller/    # 键盘控制
│   ├── libraries/                   # 库
│   │   └── qpoases/                # QP求解器
│   └── tita_webots_ros2/           # 仿真支持
│
├── tita_command/                    # 命令模块
│   ├── command_manager/            # 命令管理器
│   ├── active_command/             # 主动命令
│   ├── passive_command/            # 被动命令
│   ├── sdk_command/                # SDK命令
│   └── teleop_command/             # 远程操作
│
└── tita_bringup/                    # 启动配置
    ├── include/tita_utils/         # 工具库
    ├── launch/                     # 启动文件
    └── config/                     # 配置文件
```

## 核心参数配置

```mermaid
graph TB
    subgraph "Controller Parameters"
        P1[update_rate<br/>更新频率]
        P2[wheel_radius<br/>轮半径]
        P3[task.twist_linear_x<br/>线速度限制]
        P4[task.twist_angular_z<br/>角速度限制]
        P5[task.pose_position_z<br/>高度限制]
        P6[joint_pd<br/>关节PD参数]
        P7[single_joint_pd<br/>单关节PD]
    end
    
    subgraph "Estimator Parameters"
        E1[imu_process_noise<br/>IMU过程噪声]
        E2[imu_sensor_noise<br/>IMU传感器噪声]
    end
    
    subgraph "FSM Parameters"
        F1[tau_ff<br/>前馈力矩]
        F2[RL Policy<br/>强化学习策略]
    end
```

---

*生成时间: 2026-03-24*
*项目: TITA Robot ROS2 Control System*

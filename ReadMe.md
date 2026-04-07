# TITA RL Sim2Sim2Real

TITA 轮腿机器人 `IsaacLab -> TensorRT -> Webots/ROS2` 的部署与验证仓库。  
当前这份说明主要覆盖 `sim2sim` 流程，也就是：

1. 在 IsaacLab 中训练策略并导出 `onnx/engine`
2. 在本仓中加载 TensorRT `engine`
3. 在 Webots 中完成策略部署、键盘交互和楼梯测试

持续更新中，欢迎提 Issue 交流。

## 1. 仓库结构

- `src/tita_locomotion`: 机器人 ROS 2 控制、Webots bridge、键盘控制
- `src/tita_locomotion/tita_controllers/tita_controller`: RL 控制器与 TensorRT 推理
- `src/tita_locomotion/tita_webots_ros2/webots_bridge`: Webots 仿真桥接
- `src/tita_locomotion/locomotion_bringup`: 仿真/实机启动入口
- `sim2sim2real.repos`: 依赖仓列表

更底层的话题、控制器说明可参考 [README.md](/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/README.md)。

## 2. 环境准备

建议使用 `ROS 2 Humble + Webots + TensorRT` 环境。  
如果当前 shell 里启用了 `conda`，先退出：

```bash
conda deactivate
```

### 2.1 获取代码

```bash
mkdir tita_rl_sim2sim2real
git clone https://github.com/DDTRobot/tita_rl_sim2sim2real.git
```

### 2.2 依赖说明

如果你在宿主机上自行配环境，至少需要：

- ROS 2 Humble
- `ros2_control`
- Webots
- TensorRT
- CUDA

## 3. 准备策略文件

RL 控制器运行时配置位于：

- [rl_runtime.yaml](/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/tita_controllers/tita_controller/config/rl_runtime.yaml)
- [rl_runtime_estVel.yaml](/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/tita_controllers/tita_controller/config/rl_runtime_estVel.yaml)

两者的区别是：

- `rl_runtime.yaml`:
  直接把 `base_lin_vel_xy` 放进 policy observation。
  这一路速度来自机器人本体的真实速度输入；在当前 Webots sim2sim 流程里，对应的是仿真侧接入的 base velocity。
- `rl_runtime_estVel.yaml`:
  用 history-based velocity estimator 路线。
  policy 不直接依赖外部接入的真实底盘速度，而是通过 history + MLP 估计速度再参与控制。

重点检查 `engine_path` 是否指向你实际要部署的 TensorRT engine，例如：

```yaml
engine_path: /home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/engine/policy_stair.engine
```

注意：

- 如果你在 Docker 中运行，这里必须填写 Docker 容器内可见的绝对路径
- 当前 `sim_bringup.launch.py` 也支持通过 `rl_config:=/abs/path/to/xxx.yaml` 覆盖默认配置

## 4. 编译

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to locomotion_bringup webots_bridge template_ros2_controller tita_controller joy_controller keyboard_controller
source install/setup.bash
```

## 5. Webots Sim2Sim 流程

### 5.1 启动仿真

默认使用 Webots：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch locomotion_bringup sim_bringup.launch.py
```

如果要显式指定 RL 配置文件：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch locomotion_bringup sim_bringup.launch.py \
  rl_config:=/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/tita_controllers/tita_controller/config/rl_runtime.yaml
```

这个 launch 会启动：

- Webots bridge
- `joint_state_broadcaster`
- `imu_sensor_broadcaster`
- `tita_controller`

入口文件见 [sim_bringup.launch.py](/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/locomotion_bringup/launch/sim_bringup.launch.py)。

### 5.2 启动键盘控制

新开一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_controller keyboard_controller_node --ros-args -r __ns:=/tita
```

键盘节点源码见 [keyboard_controller.cpp](/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/interaction/keyboard_controller/src/keyboard_controller.cpp)。

### 5.3 推荐操作顺序

推荐按下面顺序测试：

1. 启动 `sim_bringup`
2. 启动 `keyboard_controller`
3. 按 `1` 进入 `transform_up`
4. 确认机器人站稳
5. 按 `2` 切换到 `rl`
6. 用键盘给速度或 heading 命令，观察平地和楼梯行为

## 6. 键盘说明

当前版本常用按键如下：

- `0`: `idle`
- `1`: `transform_up`
- `2`: `rl`
- `3`: `transform_down`
- `w/x`: 前进/后退速度
- `a/d`: 原始 yaw rate 输入
- `s`: 清零速度命令
- `q/e`: 调整 RL 的 heading target
- `c`: 清零 heading target
- `+/-`: 调整速度倍率
- `*` / `/`: 调整姿态倍率
- 方向键上/下: 调整机身高度

补充说明：

- 键盘节点启动时，`desired heading` 会初始化为机器人 spawn 时的 heading
- 在 RL 模式下，当前逻辑与 IsaacLab `play.py` 对齐：
  - 有 `a/d` yaw 输入时，直接使用原始 yaw rate
  - 没有 yaw 输入时，自动进入 heading-hold

## 7. Mesh 显示问题

如果 Webots 中机器人 mesh 不显示，可以执行：

```bash
sudo mkdir -p /usr/share/robot_description
sudo cp -r src/tita_locomotion/tita_description/tita /usr/share/robot_description/
```

如果还有 tower 资源需求，也可以一起复制：

```bash
sudo cp -r src/tita_locomotion/tita_description/tower /usr/share/robot_description/
```

## 8. 常用调试点

### 8.1 检查 IMU

```bash
ros2 topic echo /tita/imu_sensor_broadcaster/imu
```

### 8.2 检查关节状态

```bash
ros2 topic echo /tita/joint_states
```

### 8.3 检查键盘命令

```bash
ros2 topic echo /tita/command/manager/cmd_twist
ros2 topic echo /tita/command/manager/cmd_pose
ros2 topic echo /tita/command/manager/cmd_key
```

### 8.4 检查当前 RL 配置是否生效

`sim_bringup.launch.py` 会把 `rl_config` 写入环境变量 `TITA_RL_CONFIG`。  
如果不传，控制器默认读取：

- [rl_runtime.yaml](/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/tita_controllers/tita_controller/config/rl_runtime.yaml)

## 9. 已对齐的部署要点

当前仓内 sim2sim 部署已经做过这些对齐：

- `history_length` 和 `observation_terms` 由 YAML 配置驱动
- `base_lin_vel_xy` 已加入默认 policy obs
- `joint_pos` / `joint_vel` 观测顺序已按训练配置对齐
- `last_action` 按 policy action 顺序回填
- policy action 到硬件 action 的重排已修正
- RL yaw 命令逻辑已对齐 IsaacLab `play.py`
- Webots bridge 已增加与训练一致口径的扭矩限幅

## 10. 常见问题

### 10.1 切到 RL 后机器人发散或直接飞

优先检查：

- `engine_path` 是否对应正确的训练模型
- runtime obs 定义是否和训练配置一致
- `rl_config` 是否加载到了预期文件

### 10.2 Webots 里动作比 IsaacSim 更猛

这是常见的 sim-to-sim gap。  
当前至少需要关注：

- 接触模型差异
- 楼梯几何差异
- 扭矩限幅与执行器动态差异

## 11. 实机说明

本 README 先聚焦 `sim2sim`。  
如果你要继续走 `sim2real`，建议再配合：

- [hw_bringup.launch.py](/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/locomotion_bringup/launch/hw_bringup.launch.py)
- [src/tita_locomotion/README.md](/home/raise/Workspaces/tita_rl_sim2sim2real/src/tita_locomotion/README.md)

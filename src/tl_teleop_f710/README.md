# 天链机械臂 F710 手柄遥操作功能包

## 概述

`tl_teleop_f710` 是[天链机器人](https://www.tl-robot.com/) ROS2 工作空间的一个功能包，
通过 **Logitech F710 游戏手柄**远程控制天链机械臂运动。

基于笛卡尔空间伺服（`/tl_driver/set_servol_pos` 话题）实现直观的末端位置控制，
操作者无需理解关节空间，推摇杆机械臂就往对应方向运动。

支持真机控制和 Gazebo 仿真两种模式。

## 适用型号

各型号有独立的配置文件和启动文件：

| 型号 | 轴数 | 真机配置 | 仿真配置 | 真机启动 | 仿真启动 |
|------|------|----------|----------|----------|----------|
| TCB605 | 6 轴 | `tcb605.yaml` | `tcb605_sim.yaml` | `tcb605.launch.py` | `tcb605_gazebo.launch.py` |
| TCB610 | 6 轴 | `tcb605.yaml` | `tcb605_sim.yaml` | `tcb605.launch.py` | `tcb605_gazebo.launch.py` |
| TCB705 | 7 轴 | `tcb710.yaml` | `tcb710_sim.yaml` | `tcb710.launch.py` | `tcb710_gazebo.launch.py` |
| TCB710 | 7 轴 | `tcb710.yaml` | `tcb710_sim.yaml` | `tcb710.launch.py` | `tcb710_gazebo.launch.py` |

## 环境要求

- **操作系统**：Ubuntu 22.04
- **ROS2 发行版**：Humble Hawksbill
- **前置依赖**：tl_driver 功能包（需先启动 tl_driver 节点）
- **硬件**：Logitech F710 游戏手柄

## 安装依赖

```bash
# 自动安装所有 ROS 依赖（包括 joy 包）
cd ~/tl_robot_ros2_py
rosdep install --from-paths src --ignore-src -r -y

# 或者手动安装
# sudo apt install ros-humble-joy
```

## 安装 udev 规则（免 root 权限）

```bash
sudo cp src/tl_teleop_f710/udev/99-logitech-f710.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后将 F710 手柄接收器插入 USB 口，手柄开关拨到 **"D"（DirectInput 模式）**。

## 编译

```bash
cd ~/tl_robot_ros2_py
colcon build --packages-select tl_teleop_f710
source install/setup.bash
```

## 使用

### 真机模式

终端 1 — 启动机械臂驱动（以 TCB605 为例）：

```bash
ros2 launch tl_driver tl_tcb605_driver.launch.py
```

终端 2 — 启动手柄遥操作（按臂型选择）：

```bash
# TCB605 / TCB610（6 轴）
ros2 launch tl_teleop_f710 tl_teleop_tcb605.launch.py

# TCB705 / TCB710（7 轴）
ros2 launch tl_teleop_f710 tl_teleop_tcb710.launch.py
```

真机模式下节点会自动完成以下初始化：
1. 等待 tl_driver 服务就绪
2. 设置运行模式为远程模式
3. 设置 ServoJ 运动速度
4. 开启关节跟踪模式（ServoJ）
5. 输出 ✅ 提示，遥操作就绪
6. 需要按一下手柄上的“START”键，开始手柄遥控机械臂

如果手柄在非默认路径，可指定设备：

```bash
ros2 launch tl_teleop_f710 tl_teleop_tcb605.launch.py joy_dev:=/dev/input/js1
```

### Gazebo 仿真模式（无需连接真机）

```bash
# TCB605 仿真（需先安装 Pinocchio：pip3 install pinocchio）
ros2 launch tl_teleop_f710 tl_teleop_tcb605_gazebo.launch.py

# TCB710 仿真
ros2 launch tl_teleop_f710 tl_teleop_tcb710_gazebo.launch.py
```

仿真模式下 IK 由桥接节点内部使用 Pinocchio 库本地求解，无需 MoveIt2。

## 操作说明

| F710 输入 | 功能 | 说明 |
|-----------|------|------|
| **左摇杆** | X/Y 平移 | 末端在基座标系下沿 X/Y 方向移动 |
| **右摇杆上下** | Z 平移 | 末端沿 Z 方向（上下）移动 |
| **右摇杆左右** | 偏航 | 末端绕 Z 轴旋转 |
| **LB + 右摇杆左右** | 翻滚 | 末端绕 X 轴旋转 |
| **RB + 右摇杆左右** | 俯仰 | 末端绕 Y 轴旋转 |
| **十字键上** | 加速 | 增大运动速度 |
| **十字键下** | 减速 | 减小运动速度 |
| **A 键** | 回零 | 回到初始位姿 |
| **B 键** | 停止 | 停止当前运动 |
| **START 键** | 开始 | 开始手柄控制 |

## 参数配置

以tcb605机械臂为例，真机和仿真使用独立的配置文件：

| 模式 | 配置文件 | 特点 |
|------|----------|------|
| 真机 | `config/tl_teleop_tcb605.yaml` | 含 ServoJ 初始化参数，灵敏度较高 |
| 仿真 | `config/tl_teleop_tcb605_sim.yaml` | 跳过 ServoJ 初始化，灵敏度偏低更平滑 |

### 参数说明

| 参数 | 真机默认值 | 仿真默认值 | 说明 |
|------|-----------|-----------|------|
| `control_rate` | 20.0 | 20.0 | 控制循环频率 (Hz) |
| `simulation_mode` | false | true | true 时跳过 ServoJ 初始化 |
| `speed_default` | 50.0 | 50.0 | 默认运动速度 (0-100) |
| `speed_min` | 5.0 | 5.0 | 最小速度 |
| `speed_max` | 100.0 | 100.0 | 最大速度 |
| `speed_step` | 5.0 | 5.0 | 十字键每按一次速度变化量 |
| `pos_sensitivity` | 50.0 | 30.0 | 位置灵敏度 (mm/s，速度=100 时) |
| `rot_sensitivity` | 1.0 | 0.8 | 姿态灵敏度 (rad/s，速度=100 时) |
| `step_size` | 2.0 | 1.0 | servol 插值步长 (mm) |
| `deadzone` | 0.15 | 0.15 | 摇杆死区 |
| `initial_pose` | [230,0,359,3.14,0,0] | [230,0,359,3.14,0,0] | 回零后的初始位姿 |
| `servo_speed` | 25.0 | — | ServoJ 运动速度（仅真机） |

速度范围 0-100，十字键上下调节，步长 5。可通过增大 `pos_sensitivity` 来整体提高运动速度。

### 速度调优

末端运动速度估算公式：

```
速度 ≈ 摇杆值 × pos_sensitivity × (speed_value / 100)
```

- 调大 `pos_sensitivity` → 整体变快
- 调小 `pos_sensitivity` → 整体变慢

## 坐标系

- 所有平移运动基于**基座标系（Base）**
- 姿态旋转使用欧拉角 (RPY)，单位 rad
- 位置单位 mm

## 真机架构

```
F710 手柄 → joy_node → /joy → tl_teleop_f710_node
                                   ↓
                         /tl_driver/set_servol_pos
                                   ↓
                              tl_driver 节点
                    （_tl_host.so SWIG IK + servoj 执行）
                                   ↓
                              机械臂实际运动
```

## 仿真架构

```
F710 手柄 → joy_node → /joy → tl_teleop_f710_node
                                        ↓
                              /tl_driver/set_servol_pos 话题
                                        ↓
                            tl_teleop_f710_sim_bridge 节点
                            （Pinocchio 本地 IK 求解）
                                        ↓
                     Gazebo（ros2_control position controller）
                                        ↓
                                   仿真机械臂运动
```

## 注意事项

1. 真机使用前请确保机械臂已上电并处于远程模式（节点会自动设置）
2. 回零前请确认周围无障碍物
3. 速度范围 0-100，建议从 50 开始适应后再调高
4. 仿真模式需安装 Python 库：`pip3 install pinocchio`
5. 长距离移动时建议使用较大 `step_size` 以提高响应
6. 真机与仿真参数独立配置在各自的 YAML 文件中，互不干扰

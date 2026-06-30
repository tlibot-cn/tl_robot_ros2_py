<div align="center">

# 天链机器人 tl_teleop_f710 使用说明书

</div>

## 目录
* 1.[tl_teleop_f710 功能包说明](#1-tl_teleop_f710-功能包说明)
* 2.[tl_teleop_f710 功能包使用](#2-tl_teleop_f710-功能包使用)
* 3.[tl_teleop_f710 功能包架构说明](#3-tl_teleop_f710-功能包架构说明)
* 4.[tl_teleop_f710 功能包话题与服务说明](#4-tl_teleop_f710-功能包话题与服务说明)

## 1 tl_teleop_f710 功能包说明

`tl_teleop_f710` 功能包实现了通过 **Logitech F710 游戏手柄**远程控制天链机械臂运动。基于笛卡尔空间伺服（`/tl_driver/set_servol_pos` 话题）实现直观的末端位置控制，操作者无需理解关节空间，推摇杆机械臂就往对应方向运动。支持真机控制和 Gazebo 仿真两种模式。

通过以下四部分内容的介绍可以帮助大家：
* 1.了解该功能包的使用。
* 2.熟悉功能包中的文件构成及作用。
* 3.熟悉功能包相关的话题，方便开发和使用。

### 1.1 功能特性

- **F710 游戏手柄遥操作**：通过 Logitech F710 无线手柄（DirectInput 模式）控制机械臂末端在笛卡尔空间 6 自由度运动
- **笛卡尔空间伺服**：直接下发末端目标位姿（x, y, z, rx, ry, rz），由 tl_driver 内部 IK 或仿真 Pinocchio IK 求解关节角
- **真机/仿真双模式**：真机模式连接实体机械臂通过 ServoJ 关节跟踪驱动；仿真模式通过 Pinocchio 本地 IK + Gazebo position controller 虚拟驱动
- **回零 FK**：按下 A 键回零时，通过 FK 将配置的 `home_joints` 关节角度（度）转为笛卡尔位姿，不同臂型零位自动适配
- **6/7 轴自适应**：FK 和 IK 模型根据加载的 URDF 自动确定关节数量，无需手动指定
- **多种臂型兼容**：通过 `arm_type` 参数可切换任意天链机械臂型号，无需修改代码或创建新文件
- **速度实时调节**：十字键上下实时调整运动速度（0-100），支持 LB/RB 切换姿态控制模式（偏航/翻滚/俯仰）
- **摇杆死区滤波**：默认 0.15 死区阈值，手柄回中微小抖动被过滤
- **仿真 Pinocchio IK**：仿真模式下使用 Pinocchio 库本地求解逆运动学（阻尼伪逆法），无需依赖 MoveIt2
- **START 键安全机制**：需要按下手柄 START 键后才开始发送运动指令，防止误触

### 1.2 系统依赖关系

tl_teleop_f710 运行时依赖 **tl_driver** 功能包提供以下服务与话题：

| 依赖项 | 类型 | 说明 |
|--------|------|------|
| `/tl_driver/set_current_mode` | 服务 | 切换到远程模式（模式 2） |
| `/tl_driver/set_speed` | 服务 | 设置运动速度 |
| `/tl_driver/open_servoj` | 服务 | 开启 ServoJ 模式（含动力学参数 vmax/amax/jmax） |
| `/tl_driver/close_servoj` | 服务 | 关闭 ServoJ 模式 |
| `/tl_driver/coord_transform` | 服务 | 正运动学求解（关节 → 笛卡尔），用于真机回零 |
| `/tl_driver/set_servol_pos` | 话题 | 【发布】发送目标笛卡尔位姿（ServolMove） |

> **注意**：真机模式下依赖上述 tl_driver 接口；仿真模式下依赖 Gazebo + Pinocchio，不依赖 tl_driver。

因此真机使用 tl_teleop_f710 前应先启动 tl_driver。

### 1.3 适用型号

通用启动文件按轴数（6 轴 / 7 轴）分类，兼容以下所有天链机械臂型号：

| 轴数 | 支持型号 | 通用配置文件 | 通用启动文件 |
|------|---------|-------------|-------------|
| **6 轴** | TCB605、TCB605F、TCB605L、TCB605LV、TCB605V、TCB610 | `tl_teleop_f710_6axis.yaml` / `_sim.yaml` | `tl_teleop_f710_6axis.launch.py` / `_gazebo.launch.py` |
| **7 轴** | TCB610V、TCB705、TCB705F、TCB705L、TCB705LV、TCB705V、TCB710、TCB710V | `tl_teleop_f710_7axis.yaml` / `_sim.yaml` | `tl_teleop_f710_7axis.launch.py` / `_gazebo.launch.py` |

> 真机启动无需指定型号参数，YAML 中 `arm_type` 仅用于标识；
> 仿真启动需通过 `arm_type:=` 参数指定具体型号（见[启动方式](#22-启动方式)）。

## 2 tl_teleop_f710 功能包使用

### 2.1 安装依赖

```bash
# 自动安装所有 ROS 依赖（包括 joy 包）
cd ~/tl_robot_ros2_py
rosdep install --from-paths src --ignore-src -r -y

# 或者手动安装
# sudo apt install ros-humble-joy
```

仿真模式需安装 Pinocchio：

```bash
pip3 install pinocchio
```

### 2.2 安装 udev 规则（免 root 权限）

```bash
sudo cp src/tl_teleop_f710/udev/99-logitech-f710.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后将 F710 手柄接收器插入 USB 口，手柄开关拨到 **"D"（DirectInput 模式）**。

### 2.3 编译

```bash
cd ~/tl_robot_ros2_py
colcon build --packages-select tl_teleop_f710
source install/setup.bash
```

### 2.4 启动方式

#### 真机模式

**第一步**：启动机械臂驱动（以 TCB605 为例）：

```bash
ros2 launch tl_driver tl_tcb605_driver.launch.py
```

> 其他臂型的启动命令参见 [tl_driver 说明文档](../tl_driver/README.md)。

**第二步**：启动手柄遥操作（按轴数选择）：

```bash
# 6 轴机械臂通用（自动加载 config/tl_teleop_f710_6axis.yaml）
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis.launch.py

# 7 轴机械臂通用（自动加载 config/tl_teleop_f710_7axis.yaml）
ros2 launch tl_teleop_f710 tl_teleop_f710_7axis.launch.py
```

启动成功后，节点将自动依次执行：
1. 等待 tl_driver 服务就绪
2. 设置运行模式为远程模式（模式 2）
3. 设置 ServoJ 运动速度
4. 开启关节跟踪模式（ServoJ）
5. 通过 FK 将 `home_joints` 转为笛卡尔初始位姿
6. 输出 ✅ 提示，遥操作就绪

> 启动后需按一下手柄上的 **START 键**，才开始发送运动指令。

如果手柄在非默认路径，可指定设备：

```bash
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis.launch.py joy_dev:=/dev/input/js1
```

#### Gazebo 仿真模式（无需连接真机）

```bash
# 6 轴仿真（arm_type 指定具体型号）
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=tcb605
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=tcb610
ros2 launch tl_teleop_f710 tl_teleop_f710_6axis_gazebo.launch.py arm_type:=tcb605f

# 7 轴仿真
ros2 launch tl_teleop_f710 tl_teleop_f710_7axis_gazebo.launch.py arm_type:=tcb710
ros2 launch tl_teleop_f710 tl_teleop_f710_7axis_gazebo.launch.py arm_type:=tcb705
```

**支持的 `arm_type` 值**（对应 `tl_description/urdf/` 下的 URDF 模型文件）：

| 轴数 | arm_type 可选值 |
|------|----------------|
| 6 轴 | `tcb605`、`tcb605f`、`tcb605l`、`tcb605lv`、`tcb605v`、`tcb610` |
| 7 轴 | `tcb610v`、`tcb705`、`tcb705f`、`tcb705l`、`tcb705lv`、`tcb705v`、`tcb710`、`tcb710v` |

`arm_type` 参数决定了 Gazebo 中加载的 URDF 模型、Pinocchio 运动学模型以及 position controller 的关节数量。

### 2.5 操作说明

| F710 输入 | 功能 | 说明 |
|-----------|------|------|
| **左摇杆** | X/Y 平移 | 末端在基座标系下沿 X/Y 方向移动 |
| **右摇杆上下** | Z 平移 | 末端沿 Z 方向（上下）移动 |
| **右摇杆左右** | 偏航 | 末端绕 Z 轴旋转 |
| **LB + 右摇杆左右** | 翻滚 | 末端绕 X 轴旋转 |
| **RB + 右摇杆左右** | 俯仰 | 末端绕 Y 轴旋转 |
| **十字键上** | 加速 | 增大运动速度 |
| **十字键下** | 减速 | 减小运动速度 |
| **A 键** | 回零 | 回到初始位姿（通过 FK 将 `home_joints` 转为笛卡尔位姿） |
| **B 键** | 停止 | 停止当前运动 |
| **START 键** | 开始 | 开始手柄控制 |

### 2.6 配置参数说明

配置文件位于 `config/` 下，按轴数和模式分为四套：

| 配置 | 文件 | 特点 |
|------|------|------|
| 6轴真机 | `config/tl_teleop_f710_6axis.yaml` | 含 ServoJ 参数，`control_rate=100`，灵敏度适中 |
| 6轴仿真 | `config/tl_teleop_f710_6axis_sim.yaml` | `simulation_mode=true`，跳 ServoJ，灵敏度更高、步长更小 |
| 7轴真机 | `config/tl_teleop_f710_7axis.yaml` | 含 ServoJ 参数，`control_rate=100`，灵敏度适中 |
| 7轴仿真 | `config/tl_teleop_f710_7axis_sim.yaml` | `simulation_mode=true`，跳 ServoJ，灵敏度更高、步长更小 |

以 6 轴真机配置为例：

```yaml
/**:
  ros__parameters:
    control_rate: 100.0
    arm_type: tcb605
    speed_default: 50.0
    speed_min: 5.0
    speed_max: 100.0
    speed_step: 5.0
    pos_sensitivity: 80.0
    rot_sensitivity: 1.0
    step_size: 5.0
    servo_speed: 25.0
    servo_vmax: 80.0
    servo_amax: 3000.0
    servo_jmax: 50000.0
    deadzone: 0.15
    home_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    axis_left_x: 1
    axis_left_y: 0
    axis_right_x: 2
    axis_right_y: 3
    axis_dpad_x: 4
    axis_dpad_y: 5
    btn_a: 1
    btn_b: 2
    btn_lb: 4
    btn_rb: 5
    btn_start: 9
```

参数详解：

| 参数 | 6轴真机 | 6轴仿真 | 7轴真机 | 7轴仿真 | 说明 |
|------|---------|---------|---------|---------|------|
| `control_rate` | 100.0 | 100.0 | 100.0 | 100.0 | 控制循环频率 (Hz) |
| `simulation_mode` | false | true | false | true | true 时跳过 ServoJ 初始化 |
| `arm_type` | tcb605 | tcb605 | tcb710 | tcb710 | 机械臂型号标识 |
| `speed_default` | 50.0 | 50.0 | 50.0 | 50.0 | 默认运动速度 (0-100) |
| `speed_min` | 5.0 | 5.0 | 5.0 | 5.0 | 最小速度 |
| `speed_max` | 100.0 | 100.0 | 100.0 | 100.0 | 最大速度 |
| `speed_step` | 5.0 | 5.0 | 5.0 | 5.0 | 十字键每按一次速度变化量 |
| `pos_sensitivity` | 80.0 | 200.0 | 80.0 | 200.0 | 位置灵敏度 (mm/s) |
| `rot_sensitivity` | 1.0 | 2.0 | 1.0 | 2.0 | 姿态灵敏度 (rad/s) |
| `step_size` | 5.0 | 1.0 | 5.0 | 1.0 | servol 插值步长 (mm) |
| `deadzone` | 0.15 | 0.15 | 0.15 | 0.15 | 摇杆死区 |
| `home_joints` | 6 个零值 | 6 个零值 | 7 个零值 | 7 个零值 | 回零关节角度（度），通过 FK 转为笛卡尔位姿 |
| `servo_speed` | 25.0 | — | 25.0 | — | ServoJ 运动速度（仅真机） |
| `servo_vmax` | 80.0 | — | 80.0 | — | ServoJ 各轴最大速度 (°/s) |
| `servo_amax` | 3000.0 | — | 3000.0 | — | ServoJ 各轴最大加速度 (°/s²) |
| `servo_jmax` | 50000.0 | — | 50000.0 | — | ServoJ 各轴最大加加速度 (°/s³) |

> 速度范围 0-100，十字键上下调节，步长 5。末端速度估算公式：`速度 ≈ 摇杆值 × pos_sensitivity × (speed_value / 100)`。

## 3 tl_teleop_f710 功能包架构说明

### 3.1 功能包文件总览

```
tl_teleop_f710/
├── package.xml                        # ROS2 包元信息（build_type: ament_python）
├── setup.py                           # Python 包构建脚本
├── setup.cfg                          # ament_python 标准配置文件
├── resource/tl_teleop_f710            # ament 包索引标记文件
├── README.md                          # 本说明文档
├── config/                            # 参数配置文件（4 套）
│   ├── tl_teleop_f710_6axis.yaml      # 6 轴真机配置
│   ├── tl_teleop_f710_6axis_sim.yaml  # 6 轴仿真配置
│   ├── tl_teleop_f710_7axis.yaml      # 7 轴真机配置
│   └── tl_teleop_f710_7axis_sim.yaml  # 7 轴仿真配置
├── launch/                            # ROS2 启动文件（4 个）
│   ├── tl_teleop_f710_6axis.launch.py       # 6 轴真机启动
│   ├── tl_teleop_f710_6axis_gazebo.launch.py # 6 轴仿真启动
│   ├── tl_teleop_f710_7axis.launch.py       # 7 轴真机启动
│   └── tl_teleop_f710_7axis_gazebo.launch.py # 7 轴仿真启动
├── tl_teleop_f710/                   # Python 包源码目录
│   ├── __init__.py                    # 包标记文件
│   ├── tl_teleop_f710_node.py         # F710 遥操作主节点（F710TeleopNode 类）
│   └── tl_teleop_f710_sim_bridge.py   # 仿真桥接节点（ServolSimBridge 类）
├── udev/                              # udev 规则
│   └── 99-logitech-f710.rules         # F710 免 root 权限规则
└── test/                              # 测试
    └── __init__.py
```

### 3.2 代码架构说明

tl_teleop_f710 包含 **两个 ROS2 节点**，分别负责真机/仿真不同的职责。

#### tl_teleop_f710_node（F710 手柄遥操作主节点）

基于 `rclpy.node.Node`（类 `F710TeleopNode`）实现，采用 **单线程 + 定时器驱动** 架构：

| 组件 | 频率/机制 | 职责 |
|------|-----------|------|
| `/joy` 话题回调 | 事件驱动 | 缓存最新 F710 摇杆数据（`_joy_callback`） |
| 主控制循环（`_control_loop`） | `control_rate` Hz | 读取摇杆数据 → 计算增量 → 发布 servol 指令 |
| ServoJ 初始化定时器 | 1Hz | 异步状态机：等待服务 → 设置模式 → 设置速度 → 开启 ServoJ |
| FK 初始化（仿真模式） | 启动时一次 | 加载 Pinocchio 模型，将 `home_joints` 转为笛卡尔初始位姿 |

**核心类 `F710TeleopNode` 主要成员：**

| 类别 | 成员 | 说明 |
|------|------|------|
| 话题订阅 | `joy_sub_` | 订阅 `/joy`（`sensor_msgs/Joy`）接收手柄数据 |
| 话题发布 | `servol_pub_` | 发布 `/tl_driver/set_servol_pos`（`ServolMove`）发送笛卡尔目标 |
| 服务客户端 | `_set_mode_client` 等 5 个 | 调用 tl_driver 的模式切换、速度设置、ServoJ 启停、正运动学服务 |
| 控制定时器 | `control_timer_` | `control_rate` Hz 驱动 `_control_loop` |

#### tl_teleop_f710_sim_bridge（Servol → Gazebo 仿真桥接节点）

基于 `rclpy.node.Node`（类 `ServolSimBridge`）实现，采用 **多线程 + Pinocchio IK** 架构：

| 组件 | 频率/机制 | 职责 |
|------|-----------|------|
| `/tl_driver/set_servol_pos` 回调 | 事件驱动 | IK 求解 → 发送到 Gazebo position controller |
| `/joint_states` 回调 | 事件驱动 | 缓存 Gazebo 最新关节状态 |
| IK 求解 | 每帧 | 阻尼伪逆法，`max_iter=200`，`eps=1e-4` |

**仿真数据流：**

```
F710 手柄 → joy_node → /joy → tl_teleop_f710_node
                                        ↓
                              /tl_driver/set_servol_pos
                                        ↓
                            tl_teleop_f710_sim_bridge
                           （Pinocchio 本地 IK 求解）
                                        ↓
                     Gazebo position controller
                                        ↓
                                 仿真机械臂运动
```

## 4 tl_teleop_f710 功能包话题与服务说明

### 4.1 本节点发布/订阅的话题

| 方向 | 话题名 | 类型 | 发布者 | 说明 |
|------|--------|------|--------|------|
| 订阅 | `/joy` | `sensor_msgs/Joy` | `joy_node` | F710 手柄原始数据 |
| 发布 | `/tl_driver/set_servol_pos` | `tl_ros2_interface/ServolMove` | `tl_teleop_f710_node` | 目标笛卡尔位姿 + 插值步长 |
| 订阅 | `/tl_driver/set_servol_pos` | `tl_ros2_interface/ServolMove` | `tl_teleop_f710_sim_bridge` | （仿真）接收目标位姿做 IK |
| 订阅 | `/joint_states` | `sensor_msgs/JointState` | `tl_teleop_f710_sim_bridge` | （仿真）缓存当前关节角 |
| 发布 | `/tcb_group_position_controller/commands` | `std_msgs/Float64MultiArray` | `tl_teleop_f710_sim_bridge` | （仿真）发送关节目标到 Gazebo |

### 4.2 依赖的 tl_driver 接口

| 接口名 | 类型 | 使用方 | 说明 |
|--------|------|--------|------|
| `/tl_driver/set_current_mode` | 服务 | 真机节点 | ServoJ 初始化：设置远程模式（模式 2） |
| `/tl_driver/set_speed` | 服务 | 真机节点 | ServoJ 初始化：设置运动速度 |
| `/tl_driver/open_servoj` | 服务 | 真机节点 | ServoJ 初始化：开启关节跟踪模式 |
| `/tl_driver/close_servoj` | 服务 | 真机节点 | 节点退出时关闭 ServoJ |
| `/tl_driver/coord_transform` | 服务 | 真机节点 | 回零时 FK 计算（关节 → 笛卡尔） |

## 注意事项

1. 真机使用前请确保机械臂已上电并处于远程模式（节点会自动设置）
2. 回零前请确认周围无障碍物
3. 速度范围 0-100，建议从 50 开始适应后再调高
4. 仿真模式需安装 Python 库：`pip3 install pinocchio`
5. 长距离移动时建议使用较大 `step_size` 以提高响应
6. 真机与仿真参数独立配置在各自的 YAML 文件中，互不干扰
7. 仿真模式通过 `arm_type` 参数自动匹配对应 URDF 模型和关节数量（6/7 轴）

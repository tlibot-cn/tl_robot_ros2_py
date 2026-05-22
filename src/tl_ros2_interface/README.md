# TL ROS2 Interface 使用说明书

版本：V1.1.0

<div align="center">

TL 系列机械臂 ROS2 接口说明

</div>

## 目录
* 1 [功能包说明](#功能包说明)
* 2 [使用说明](#使用说明)
* 3 [文件总览](#文件总览)
* 4 [消息（msg）说明](#消息msg说明)
* 5 [服务（srv）说明](#服务srv说明)

## 功能包说明
tl_ros2_interface 功能包为 TL 系列机械臂在 ROS2 框架下提供消息（msg）和服务（srv）接口定义，供上层驱动或应用调用。该包本身没有可执行程序，主要作用为定义协议数据结构和服务接口。

## 使用说明
1. 将该功能包加入到你的 ROS2 工作区（colcon 工作区）src 下。
2. 在编译前确认 package.xml 与 CMakeLists.txt 中的依赖（例如 std_msgs、geometry_msgs 等）已满足。
3. 使用 colcon build 编译整个工作区：

```
colcon build --symlink-install
```

4. 编译完成后，其他功能包可通过包含该包的消息/服务来与机械臂交互。

## 文件总览
```
tl_ros2_interface/
├── CMakeLists.txt        # 编译规则
├── package.xml           # 依赖声明
├── msg/                  # 消息定义
│   ├── ArmStatus.msg
│   ├── CartesianPose.msg
│   ├── JobFileName.msg
│   ├── JobInsertMove.msg
│   ├── ModbusMasterParam.msg
│   ├── ModbusRTUParam.msg
│   ├── ModbusTCPParam.msg
│   ├── MoveCommand.msg
│   ├── ObjectInfo.msg
│   ├── RobotDHParam.msg
│   ├── RobotJointParam.msg
│   └── ToolParam.msg
├── srv/                  # 服务定义
│   ├── CoordTransform.srv
│   ├── GetAllJobFileName.srv
│   ├── GetCoordNum.srv
│   ├── GetCurrentCoord.srv
│   ├── GetDHParam.srv
│   ├── GetDigitalInputOutput.srv
│   ├── GetGlobalPos.srv
│   ├── GetJointSoftwareVersion.srv
│   ├── GetJointTemperature.srv
│   ├── GetJointVoltage.srv
│   ├── GetMotorCurrent.srv
│   ├── GetPosReachable.srv
│   ├── GetPosTransform.srv
│   ├── GetRobotJointParam.srv
│   ├── GetRobotState.srv
│   ├── GetSpeed.srv
│   ├── JobRun.srv
│   ├── Jogging.srv
│   ├── LogDownload.srv
│   ├── ModbusRead.srv
│   ├── ModbusWrite.srv
│   ├── OpenServoJ.srv
│   ├── QueueMotionMoveJ.srv
│   ├── QueueMotionSetStatus.srv
│   ├── RestoreDefaultDHParam.srv
│   ├── SetAxisZeroPos.srv
│   ├── SetControllerIP.srv
│   ├── SetCoordNum.srv
│   ├── SetCurrentCoord.srv
│   ├── SetCurrentMode.srv
│   ├── SetDHParam.srv
│   ├── SetDigitalOutput.srv
│   ├── SetDragMode.srv
│   ├── SetGlobalPos.srv
│   ├── SetRobotJointParam.srv
│   ├── SetSpeed.srv
│   ├── SetToolParam.srv
│   ├── SetUserCoord.srv
│   ├── ToolHandCalib.srv
│   ├── TrackPlayback.srv
│   ├── TrackSave.srv
│   └── ...
```

## 消息（msg）说明
以下列出 msg 文件定义及字段说明。

### ArmStatus.msg
```
# 机械臂运行状态
builtin_interfaces/Time stamp
string run_state
```
__msg成员__
- stamp: 时间戳
- run_state: 运行状态描述字符串

### CartesianPose.msg
```
# 直角坐标系位姿
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 rpy
float64 arm_angle
```
__msg成员__
- header: 标准消息头
- position: x,y,z 位置（米）
- rpy: 姿态欧拉角（弧度）
- arm_angle: 机械臂额外角度（弧度）

### JobFileName.msg
```
# 作业文件名
string[] file_name
```
__msg成员__
- file_name: 作业文件名数组

### JobInsertMove.msg
```
# 插入运动指令
int32 line
MoveCommand cmd
```
__msg成员__
- line: 插入的行号
- cmd: MoveCommand 类型的运动指令

### ModbusMasterParam.msg
```
# Modbus Master参数
string type
bool start_addr
tl_ros2_interface/ModbusTCPParam tcp
tl_ros2_interface/ModbusRTUParam rtu
```
__msg成员__
- type: "TCP" 或 "RTU"
- start_addr: 是否从起始地址开始
- tcp: Modbus TCP 参数
- rtu: Modbus RTU 参数

### ModbusRTUParam.msg
```
# Modbus RTU参数
int32 slave_id
int32 port
int32 baudrate
int32 data_bit
int32 stop_bit
string check_bit
```
__msg成员__
- slave_id: 从站 ID
- port: 串口号
- baudrate: 波特率
- data_bit: 数据位
- stop_bit: 停止位
- check_bit: 校验方式

### ModbusTCPParam.msg
```
# Modbus TCP参数
string ip
int32 port
```
__msg成员__
- ip: 目标 IP
- port: 目标端口

### MoveCommand.msg
```
# 运动指令
float64[] target_pos_value
string target_pos_name
int32 target_pos_type
int32 coord
float64 velocity
float64 velocity_sync
float64 acc
float64 dec
int32 pl
int32 time
int32 tool_num
int32 user_num
int32 posidtype
int32 configuration
int32 spin
bool para_sync
```
__msg成员__
- target_pos_value: 目标位置数值数组（关节或笛卡尔）
- target_pos_name: 目标位置名称
- target_pos_type: 位置类型标识
- coord: 坐标系编号
- velocity/velocity_sync/acc/dec: 速度与加减速参数
- pl/time: 轨迹/时间相关参数
- tool_num/user_num: 工具/用户坐标编号
- posidtype/configuration/spin/para_sync: 其他标志位或配置

### ObjectInfo.msg
```
# 目标检测结果
string type
geometry_msgs/PointStamped pos
```
__msg成员__
- type: 目标类型
- pos: 检测到的目标位置（带时间戳）

### RobotDHParam.msg
```
# 机器人DH参数
float64 l1
float64 l2
float64 l3
float64 l4
float64 l5
float64 l6
float64 l7
float64 l8
float64 l9
float64 l10
float64 l11
float64 l12
float64 l13
float64 l14
float64 l15
float64 l16
float64 l17
float64 l18
float64 l19
float64 l20

float64 couple_coe_1_2
float64 couple_coe_2_3
float64 couple_coe_3_2
float64 couple_coe_3_4
float64 couple_coe_4_5
float64 couple_coe_4_6
float64 couple_coe_5_6

float64 dynamic_limit_max
float64 dynamic_limit_min

float64 pitch
float64 sliding_lead_value
float64 uplift_lead_value
float64 spray_distance

float64 three_axis_direction
float64 five_axis_direction

float64	two_axis_convertion_ratio
float64 three_axis_convertion_ratio
float64 amplification_ratio

float64 convertion_ratio_x
float64 convertion_ratio_y
float64 convertion_ratio_z

float64 convertion_ratio_j1
float64 convertion_ratio_j2
float64 convertion_ratio_j3

int32 upside_down

float64 pc
float64[] sp
float64[] tl
```
__msg成员__
- l1..l20: DH 或结构相关长度参数
- couple_coe_*: 联动系数
- dynamic_limit_*: 动态限制
- pitch/sliding/uplift/spray: 专用参数
- direction/convertion_ratio/...: 传感/换算参数
- upside_down: 反向标志
- pc/sp/tl: 其他参数数组

### RobotJointParam.msg
```
# 机械臂关节参数
float64 reduction_ratio
int32 encoder_resolution
float64 pos_sw_limit
float64 neg_sw_limit
float64 rated_rot_speed
float64 rated_derot_speed
float64 max_rot_speed
float64 max_derot_speed
float64 rated_vel
float64 rated_devel
float64 max_acc
float64 max_dec
int32 direction
```
__msg成员__
- reduction_ratio: 减速比
- encoder_resolution: 编码器分辨率
- pos_sw_limit/neg_sw_limit: 位置限位
- rated_*/max_*: 额定/最大速度与加速度
- direction: 旋向

### ToolParam.msg
```
# 工具参数
float64 x
float64 y
float64 z
float64 a
float64 b
float64 c
float64 payload_mass
float64 payload_inertia
float64 payload_mass_center_x
float64 payload_mass_center_y
float64 payload_mass_center_z
```
__msg成员__
- x,y,z,a,b,c: 工具相对基座的位姿（单位米/弧度）
- payload_*: 负载质量、惯量、质心位置

## 服务（srv）说明
下面列出 srv 文件的请求/响应字段及简要说明。

### CoordTransform.srv
```
int32 origin_coord
int32 target_coord
int32 form
float64[] origin_pos
float64[] reference_pos
---
bool success
string message
float64[] target_pos
```
- origin_coord/target_coord: 源/目标坐标系编号
- form: 转换模式
- origin_pos/reference_pos: 输入位置与参考位置
- 返回: success/message/target_pos

### GetAllJobFileName.srv
```
---
bool success
string message
tl_ros2_interface/JobFileName[] robots_file
```
- 返回所有作业文件名列表

### GetCoordNum.srv
```
---
bool success
string message
int32 tool_num
int32 user_num
```
- 返回工具与用户坐标数量

### GetCurrentCoord.srv
```
---
bool success
string message
int32 coord
```
- 返回当前坐标系编号

### GetDHParam.srv
```
---
bool success
string message
tl_ros2_interface/RobotDHParam param
```
- 返回机器人 DH 参数

### GetDigitalInputOutput.srv
```
---
bool success
string message
int32[] input
int32[] output
```
- 返回数字 IO 状态

### GetGlobalPos.srv
```
string pos_name
---
bool success
string message
float64[] pos
```
- 输入 pos_name，返回对应全局位置点

### GetJointSoftwareVersion.srv
```
int32 axis_num
---
bool success
string message
```
- 查询指定轴的软件版本（返回 message）

### GetJointTemperature.srv
```
---
bool success
string message
float64[] temperatures
```
- 返回各关节温度数组

### GetJointVoltage.srv
```
---
bool success
string message
float64[] joint_voltage
float64[] positioner_voltage
```
- 返回关节与定位器电压

### GetMotorCurrent.srv
```
---
bool success
string message
float64[] motor_current
```
- 返回电机电流数组

### GetPosReachable.srv
```
float64[] pos
string move_type
---
bool success
string message
```
- 输入目标位姿，返回是否可达

### GetPosTransform.srv
```
float64[] input
---
bool success
string message
float64[] output
```
- 输入位姿，返回转换后的位姿

### GetRobotJointParam.srv
```
int32 id
---
bool success
string message
tl_ros2_interface/RobotJointParam param
```
- 查询并返回指定关节参数

### GetRobotState.srv
```
int32 channel
bool stop
int32 mode
int32 interval
bool io_state
int32 position
bool detail_motion_pos
int32 pos_sum
string[] io_port
string[] optional
---
bool success
string message
```
- 获取机械臂运行相关状态，包含 IO 与运动详情

### GetSpeed.srv
```
---
bool success
string message
float64 speed
```
- 返回当前全局速度设置

### JobRun.srv
```
string job_name
---
bool success
string message
```
- 请求运行指定作业文件

### Jogging.srv
```
int32 axis
bool direction
---
bool success
string message
```
- 点动控制指定轴及方向

### LogDownload.srv
```
int32 count
string directory_path
---
bool success
string message
```
- 请求下载日志到目录

### ModbusWrite.srv
```
int32 master_id
int32 addr
int32[] data
tl_ros2_interface/ModbusMasterParam master_param
---
bool success
string message
```
- 向 Modbus 从站写入数据

### ModbusRead.srv
```
int32 master_id
int32 addr
int32 quantity
tl_ros2_interface/ModbusMasterParam master_param
---
bool success
string message
int32[] data
```
- 从 Modbus 读取数据

### OpenServoJ.srv
```
float64[] vmax
float64[] amax
float64[] jmax
---
bool success
string message
```
- 设置并打开关节伺服的最大速度/加速度

### QueueMotionMoveJ.srv
```
bool is_continue
tl_ros2_interface/MoveCommand cmd
---
bool success
string message
```
- 将 MoveJ 命令加入队列或继续队列执行

### QueueMotionSetStatus.srv
```
bool status
---
bool success
string message
```
- 设置队列运动模式开关

### RestoreDefaultDHParam.srv
```
int32 robot_num
---
bool success
string message
```
- 恢复默认 DH 参数

### SetAxisZeroPos.srv
```
int32 axis
---
bool success
string message
```
- 设置轴零位

### SetControllerIP.srv
```
string name
string addr
string gateway
string dns
---
bool success
string message
```
- 配置控制器网络参数

### SetCoordNum.srv
```
int32 coord
---
bool success
string message
```
- 设置当前坐标系

### SetCurrentCoord.srv
```
int32 mode
---
bool success
string message
```
- 设置当前工作模式/坐标相关模式

### SetDHParam.srv
```
tl_ros2_interface/RobotDHParam param
---
bool success
string message
```
- 设置机器人 DH 参数

### SetDigitalOutput.srv
```
int32 port
int32 value
---
bool success
string message
```
- 设置数字输出口值

### SetDragMode.srv
```
int32 mode
---
bool success
string message
```
- 设置拖拽（示教）模式

### SetGlobalPos.srv
```
string pos_name
float64[] pos_info
---
bool success
string message
```
- 设置/保存全局位置点

### SetRobotJointParam.srv
```
int32 id
tl_ros2_interface/RobotJointParam param
---
bool success
string message
```
- 设置指定关节参数

### SetSpeed.srv
```
float64 speed
---
bool success
string message
```
- 设置全局速度

### SetToolParam.srv
```
int32 tool_num
tl_ros2_interface/ToolParam param
---
bool success
string message
```
- 设置工具参数（TCP、负载）

### SetUserCoord.srv
```
int32 user_num
tl_ros2_interface/CartesianPose pos
---
bool success
string message
```
- 设置用户坐标系的笛卡尔位姿

### TrackPlayback.srv
```
int32 vel
---
bool success
string message
```
- 回放轨迹

### TrackSave.srv
```
string traj_name
---
bool success
string message
```
- 保存轨迹

## 说明与后续
- 若需更详细的字段含义或示例用法，可提交 issue 或说明需要的具体消息/服务，文档将补充字段示例与使用场景。

## 联系
如有问题，请在仓库 issue 提出或联系包维护者。

# TL ROS2 Interface 使用说明书

版本：V1.0.0

## 目录
* 1 [功能包说明](#功能包说明)
* 2 [使用说明](#使用说明)
* 3 [文件总览](#文件总览)
* 4 [消息（msg）列表](#消息msg列表)
* 5 [服务（srv）列表](#服务srv列表)

## 功能包说明
tl_ros2_interface 功能包为 TL 系列机械臂在 ROS2 框架下提供消息（msg）和服务（srv）接口定义，供上层驱动或应用调用。该包本身没有可执行程序，主要作用为定义协议数据结构和服务接口。

## 使用说明
1. 将该功能包加入到你的 ROS2 工作区（colcon 工作区）src 下。
2. 在编译前确认 package.xml 与 CMakeLists.txt 中的依赖（例如 geometry_msgs 等）已满足。
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

## 消息（msg）列表
详见 msg/ 目录中各 .msg 文件。主要消息包含机械臂状态、笛卡尔位姿、工具与 DH 参数、作业文件名与插入移动等结构，用于在 ROS2 话题中传递机械臂信息。

## 服务（srv）列表
详见 srv/ 目录中各 .srv 文件。服务涵盖坐标变换、DH 参数获取/设置、IO 读写、作业管理、速度/模式设置、工具标定等接口。

## 进一步说明
- 如需消息字段的详细说明，请打开对应的 .msg 文件查看注释与字段定义。
- 如需服务的输入输出说明，请打开对应的 .srv 文件查看具体定义。

## 联系
如有问题，请在仓库 issue 提出或联系包维护者。
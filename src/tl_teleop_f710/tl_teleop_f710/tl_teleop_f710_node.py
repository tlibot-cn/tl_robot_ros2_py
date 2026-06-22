#!/usr/bin/env python3
"""天链机械臂 Logitech F710 手柄遥操作节点。

通过 F710 游戏手柄远程控制天链机械臂，基于笛卡尔空间伺服（servol）
实现直观的末端位置控制。操作者推摇杆，机械臂就往对应方向运动。

工作流程：
  F710 → joy_node → /joy 话题 → 本节点 → /tl_driver/set_servol_pos → tl_driver → 机械臂

控制模式（笛卡尔空间，基座标系）：
  - 左摇杆  → X/Y 平移
  - 右摇杆  → Z 平移 + 偏航（默认）/ 翻滚（LB）/ 俯仰（RB）
  - 十字键  → 速度倍率调节
  - A 键    → 回零
  - B 键    → 停止

依赖：
  - ros-humble-joy（apt 安装）
  - tl_ros2_interface（ServolMove 消息）
  - tl_driver（set_servol_pos 话题）
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from tl_ros2_interface.msg import ServolMove
from tl_ros2_interface.srv import SetCurrentMode, SetSpeed, OpenServoJ


class F710TeleopNode(Node):
    """F710 手柄遥操作 ROS2 节点。"""

    def __init__(self):
        super().__init__('tl_teleop_f710_node')

        # ==================== 声明参数 ====================
        self._declare_parameters()

        # ==================== 读取参数 ====================
        self._load_parameters()

        # ==================== 内部状态 ====================
        self._latest_joy = None          # 最新摇杆数据
        self._joy_lock = False           # 简单锁标志
        self._last_dpad_time = 0.0       # 十字键上次触发时间
        self._last_a_press = 0.0         # A 键上次按下时间
        self._last_b_press = 0.0         # B 键上次按下时间
        self.speed_value_ = self.speed_default_  # 当前运动速度 0-100
        self.target_pose_ = list(self.initial_pose_)    # [x, y, z, rx, ry, rz]

        # ==================== 订阅 ====================
        self.joy_sub_ = self.create_subscription(
            Joy, '/joy', self._joy_callback, 10)

        # ==================== 发布 ====================
        self.servol_pub_ = self.create_publisher(
            ServolMove, '/tl_driver/set_servol_pos', 10)

        # ==================== 服务客户端（ServoJ 初始化）====================
        self._set_mode_client = self.create_client(
            SetCurrentMode, '/tl_driver/set_current_mode')
        self._set_speed_client = self.create_client(
            SetSpeed, '/tl_driver/set_speed')
        self._open_servoj_client = self.create_client(
            OpenServoJ, '/tl_driver/open_servoj')
        self._close_servoj_client = self.create_client(
            Trigger, '/tl_driver/close_servoj')
        self._init_state = 0        # ServoJ 初始化状态机：0=等待服务 1~3=进行中 4=完成
        self._init_future = None    # 当前异步服务调用的 future

        # ==================== ServoJ 初始化定时器 ====================
        self.create_timer(1.0, self._init_servoj)
        self.get_logger().info('ServoJ 初始化定时器已启动（1 秒后检查服务）')

        # ==================== 控制定时器 ====================
        period = 1.0 / self.control_rate_
        self.control_timer_ = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'F710 遥操作节点已启动 ({self.control_rate_}Hz, '
            f'灵敏度 {self.pos_sensitivity_}mm/s)')

    # ==================== 参数系统 ====================

    def _declare_parameters(self):
        """声明所有 ROS2 参数（与 YAML 配置文件对应）。"""
        # 控制频率
        self.declare_parameter('control_rate', 20.0)
        # 运动速度（0-100，对应机械臂实际速度范围）
        self.declare_parameter('speed_default', 50.0)
        self.declare_parameter('speed_min', 5.0)
        self.declare_parameter('speed_max', 100.0)
        self.declare_parameter('speed_step', 5.0)
        # ServoJ 初始化参数
        self.declare_parameter('servo_speed', 25.0)
        self.declare_parameter('servo_vmax', 80.0)
        self.declare_parameter('servo_amax', 3000.0)
        self.declare_parameter('servo_jmax', 50000.0)
        # 运动灵敏度
        self.declare_parameter('pos_sensitivity', 50.0)
        self.declare_parameter('rot_sensitivity', 1.0)
        self.declare_parameter('step_size', 2.0)
        # 摇杆死区
        self.declare_parameter('deadzone', 0.15)
        # 初始位姿
        self.declare_parameter('initial_pose', [0.0, 0.0, 300.0, 0.0, 0.0, 0.0])
        # 轴映射（DirectInput 模式）
        self.declare_parameter('axis_left_x', 0)
        self.declare_parameter('axis_left_y', 1)
        self.declare_parameter('axis_right_x', 3)
        self.declare_parameter('axis_right_y', 4)
        self.declare_parameter('axis_dpad_x', 6)
        self.declare_parameter('axis_dpad_y', 7)
        # 按键映射
        self.declare_parameter('btn_x', 0)
        self.declare_parameter('btn_a', 1)
        self.declare_parameter('btn_b', 2)
        self.declare_parameter('btn_y', 3)
        self.declare_parameter('btn_lb', 4)
        self.declare_parameter('btn_rb', 5)
        self.declare_parameter('btn_lt', 6)
        self.declare_parameter('btn_rt', 7)
        self.declare_parameter('btn_back', 8)
        self.declare_parameter('btn_start', 9)

    def _load_parameters(self):
        """将参数值读取到实例变量。"""
        self.control_rate_ = self.get_parameter('control_rate').value
        self.speed_default_ = self.get_parameter('speed_default').value
        self.speed_min_ = self.get_parameter('speed_min').value
        self.speed_max_ = self.get_parameter('speed_max').value
        self.speed_step_ = self.get_parameter('speed_step').value
        # ServoJ 初始化参数
        self.servo_speed_ = self.get_parameter('servo_speed').value
        self.servo_vmax_ = self.get_parameter('servo_vmax').value
        self.servo_amax_ = self.get_parameter('servo_amax').value
        self.servo_jmax_ = self.get_parameter('servo_jmax').value
        self.pos_sensitivity_ = self.get_parameter('pos_sensitivity').value
        self.rot_sensitivity_ = self.get_parameter('rot_sensitivity').value
        self.step_size_ = self.get_parameter('step_size').value
        self.deadzone_ = self.get_parameter('deadzone').value
        self.initial_pose_ = list(self.get_parameter('initial_pose').value)

        self.axis_left_x_ = int(self.get_parameter('axis_left_x').value)
        self.axis_left_y_ = int(self.get_parameter('axis_left_y').value)
        self.axis_right_x_ = int(self.get_parameter('axis_right_x').value)
        self.axis_right_y_ = int(self.get_parameter('axis_right_y').value)
        self.axis_dpad_x_ = int(self.get_parameter('axis_dpad_x').value)
        self.axis_dpad_y_ = int(self.get_parameter('axis_dpad_y').value)

        self.btn_a_ = int(self.get_parameter('btn_a').value)
        self.btn_b_ = int(self.get_parameter('btn_b').value)
        self.btn_lb_ = int(self.get_parameter('btn_lb').value)
        self.btn_rb_ = int(self.get_parameter('btn_rb').value)

    # ==================== 工具函数 ====================

    @staticmethod
    def _apply_deadzone(value, deadzone):
        """对摇杆原始值应用死区并重新映射到 [0, 1] 范围。

        Args:
            value: 摇杆原始值 [-1.0, 1.0]
            deadzone: 死区阈值

        Returns:
            映射后的值，死区内返回 0.0
        """
        if abs(value) < deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def _publish_servol(self):
        """发布当前目标位姿到 servol 话题。"""
        msg = ServolMove()
        msg.target_pose = [float(v) for v in self.target_pose_]
        msg.step_size = self.step_size_
        msg.coord = 1  # 基座标系
        self.servol_pub_.publish(msg)

    # ==================== ServoJ 初始化（状态机）====================
    # _init_state: 0=等待服务 1=等待set_mode 2=等待set_speed
    #              3=等待open_servoj 4=完成

    def _init_servoj(self):
        """ServoJ 初始化状态机：每步非阻塞，由定时器驱动。"""
        state = self._init_state

        if state == 4:
            return

        # ---- 状态 0：等待所有服务就绪 ----
        if state == 0:
            if not self._set_mode_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().info(
                    '等待 tl_driver 服务就绪...', throttle_duration_sec=3.0)
                return
            if not self._set_speed_client.wait_for_service(timeout_sec=0.1):
                return
            if not self._open_servoj_client.wait_for_service(timeout_sec=0.1):
                return
            self.get_logger().info('ServoJ 初始化中...')
            # 发送 set_mode 请求
            req = SetCurrentMode.Request()
            req.mode = 2
            self._init_future = self._set_mode_client.call_async(req)
            self._init_state = 1
            return

        # ---- 状态 1：等待 set_mode 完成 ----
        if state == 1:
            if not self._init_future.done():
                return  # 下次 tick 再检查
            if not self._init_future.result().success:
                self.get_logger().error(
                    f'设置远程模式失败: {self._init_future.result().message}')
                self._init_state = 4  # 标记完成防止重复报错
                return
            self.get_logger().info('ServoJ 模式已设为远程(2)')
            # 发送 set_speed 请求
            req_s = SetSpeed.Request()
            req_s.speed = self.servo_speed_
            self._init_future = self._set_speed_client.call_async(req_s)
            self._init_state = 2
            return

        # ---- 状态 2：等待 set_speed 完成 ----
        if state == 2:
            if not self._init_future.done():
                return
            if not self._init_future.result().success:
                self.get_logger().error(
                    f'设置 ServoJ 速度失败: {self._init_future.result().message}')
                self._init_state = 4
                return
            self.get_logger().info(f'ServoJ 速度已设为 {self.servo_speed_}')
            # 发送 open_servoj 请求
            req_o = OpenServoJ.Request()
            req_o.vmax = [self.servo_vmax_] * 7
            req_o.amax = [self.servo_amax_] * 7
            req_o.jmax = [self.servo_jmax_] * 7
            self._init_future = self._open_servoj_client.call_async(req_o)
            self._init_state = 3
            return

        # ---- 状态 3：等待 open_servoj 完成 ----
        if state == 3:
            if not self._init_future.done():
                return
            if not self._init_future.result().success:
                self.get_logger().error(
                    f'开启 ServoJ 失败: {self._init_future.result().message}')
                self._init_state = 4
                return
            self.get_logger().info('ServoJ 已开启，遥操作就绪 ✅')
            self._init_state = 4
            return

    def _close_servoj(self):
        """关闭 ServoJ 模式（非阻塞，发请求不等回复）。"""
        if self._init_state < 3:  # 还没到 open_servoj 步骤，无需关闭
            return
        try:
            self._close_servoj_client.call_async(Trigger.Request())
            self.get_logger().info('ServoJ 已关闭')
        except Exception as e:
            self.get_logger().error(f'关闭 ServoJ 异常: {e}')

    # ==================== 话题回调 ====================

    def _joy_callback(self, msg):
        """缓存最新的摇杆状态（由 ROS2 线程调用）。"""
        # 使用简单交换代替锁，避免在回调中创建锁对象
        self._latest_joy = msg

    # ==================== 主控制循环 ====================

    def _control_loop(self):
        """定时器驱动的控制循环：读取摇杆 → 计算增量 → 发布 servol。"""
        joy = self._latest_joy
        if joy is None:
            return

        now = time.time()

        # 校验数据完整性：检查实际存在的轴数和键数
        needed_axes = max(self.axis_left_x_, self.axis_left_y_,
                          self.axis_right_x_, self.axis_right_y_)
        needed_btns = max(self.btn_a_, self.btn_b_, self.btn_lb_, self.btn_rb_)
        if len(joy.axes) <= needed_axes or len(joy.buttons) <= needed_btns:
            self.get_logger().warning(
                f'摇杆数据异常: axes={len(joy.axes)}, buttons={len(joy.buttons)}',
                throttle_duration_sec=5.0)
            return

        # 十字键可能因手柄型号不同而缺失，安全读取
        dpad_y = (joy.axes[self.axis_dpad_y_]
                  if self.axis_dpad_y_ < len(joy.axes) else 0.0)
        has_dpad = self.axis_dpad_y_ < len(joy.axes)

        # ========== A 键：回零（防抖 500ms） ==========
        if joy.buttons[self.btn_a_] == 1 and (now - self._last_a_press) > 0.5:
            self._last_a_press = now
            self.target_pose_ = list(self.initial_pose_)
            self.speed_value_ = self.speed_default_
            self._publish_servol()
            self.get_logger().info(f'回零 → {self.initial_pose_}')
            return

        # ========== B 键：停止（防抖 500ms） ==========
        if joy.buttons[self.btn_b_] == 1 and (now - self._last_b_press) > 0.5:
            self._last_b_press = now
            self.get_logger().info('停止运动')
            return  # 不发布任何新目标，让当前运动自然结束

        # ========== 速度调节（十字键上下，防抖 300ms，范围 0-100）==========
        if has_dpad and dpad_y != 0.0 and (now - self._last_dpad_time) > 0.3:
            if dpad_y > 0.0:  # 上（部分手柄十字键上为正）
                self.speed_value_ = min(self.speed_max_,
                                        self.speed_value_ + self.speed_step_)
            else:  # 下
                self.speed_value_ = max(self.speed_min_,
                                        self.speed_value_ - self.speed_step_)
            self._last_dpad_time = now
            self.get_logger().info(f'速度: {self.speed_value_:.0f}')

        # ========== 读取摇杆（带死区） ==========
        lx = self._apply_deadzone(joy.axes[self.axis_left_x_], self.deadzone_)
        ly = self._apply_deadzone(joy.axes[self.axis_left_y_], self.deadzone_)
        rx = self._apply_deadzone(joy.axes[self.axis_right_x_], self.deadzone_)
        ry = self._apply_deadzone(joy.axes[self.axis_right_y_], self.deadzone_)

        # 所有摇杆都在死区内的提前返回
        if not any(abs(v) > 0.001 for v in (lx, ly, rx, ry)):
            return

        # ========== 计算运动增量 ==========
        dt = 1.0 / self.control_rate_
        scale = self.speed_value_ / 100.0  # 0-100 → 0.0-1.0 系数

        # 左摇杆 → X/Y 平移
        dx = lx * self.pos_sensitivity_ * scale * dt
        dy = ly * self.pos_sensitivity_ * scale * dt

        # 右摇杆上下 → Z 平移
        dz = ry * self.pos_sensitivity_ * scale * dt

        # 默认：右摇杆左右 → 偏航（绕 Z 轴旋转）
        dyaw = rx * self.rot_sensitivity_ * scale * dt

        # LB 按下 → 右摇杆左右 → 翻滚（绕 X 轴）
        # RB 按下 → 右摇杆左右 → 俯仰（绕 Y 轴）
        lb = joy.buttons[self.btn_lb_] == 1
        rb = joy.buttons[self.btn_rb_] == 1
        roll, pitch = 0.0, 0.0

        if lb and not rb:
            roll = rx * self.rot_sensitivity_ * scale * dt
            dyaw = 0.0
        elif rb and not lb:
            pitch = rx * self.rot_sensitivity_ * scale * dt
            dyaw = 0.0
        elif lb and rb:
            # LB+RB 同时按下时，保持偏航模式
            pass

        # ========== 更新目标位姿 ==========
        self.target_pose_[0] += dx
        self.target_pose_[1] += dy
        self.target_pose_[2] += dz
        self.target_pose_[3] += roll
        self.target_pose_[4] += pitch
        self.target_pose_[5] += dyaw

        # ========== 发布 servol 指令 ==========
        self._publish_servol()


def main(args=None):
    rclpy.init(args=args)
    node = F710TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，退出')
    finally:
        node._close_servoj()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

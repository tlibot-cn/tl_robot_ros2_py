#!/usr/bin/env python3
import time
import threading
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from tl_driver.lib import tl_interface as nrc
import xrobotoolkit_sdk as xrt

# ===================== 轴数选择配置（切换 6 / 7）=====================
ARM_AXIS_MODE = 6 # 修改这里：6 = 6轴模式（TCB605系列），7 = 7轴模式（TCB710系列）
# =========================================================================

# ========== 全局参数 ==========
pose_data = [0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
grip_data = 0.0
running = True
vr_home_pose = None
base_arm_quat = None
vr_home_quat = None

data_lock = threading.Lock()

# 位置控制
POS_SCALE = 0.5
POS_DEADZONE = 0.005
MAX_POS_DELTA_MM = 300.0

# ROS2
ROS2_PUBLISH_MODE = 2
# 根据轴数模式动态初始化目标关节列表
latest_target_joints = [0.0] * ARM_AXIS_MODE
LOG_INTERVAL = 1.0
last_log_time = 0.0

# ========== 奇异点防护参数 ==========
# 关节硬限位（避开 ±180°）
JOINT_LIMITS_7 = [
    (-180, 180),
    (-180, 180),
    (-180, 180),
    (-180, 180),
    (-180, 180),
    (-170, 170),   # Joint6 限制 ±170°
    (-170, 170)    # Joint7 限制 ±170°
]
JOINT_LIMITS_6 = [
    (-180, 180),
    (-180, 180),
    (-180, 180),
    (-180, 180),
    (-180, 180),
    (-170, 170)    # Joint6 限制 ±170°
]
# 前后帧关节突变阈值
JOINT_JUMP_THRESHOLD = 30.0
# 奇异区减速：|角度|>160° 降速
SINGULAR_ANGLE = 160.0
SINGULAR_SCALE = 0.2

# ========== VR读取线程 ==========
def device_read_thread():
    global pose_data, grip_data, running
    try:
        xrt.init()
        print("[INFO] 遥感设备已连接")
    except Exception as e:
        print(f"[ERROR] 遥感设备初始化失败: {e}")
        return

    while running:
        try:
            pose = xrt.get_right_controller_pose()
            grip = xrt.get_right_grip()
            with data_lock:
                pose_data = pose
                grip_data = grip
            time.sleep(0.01)
        except Exception as e:
            time.sleep(0.05)
            continue
    xrt.close()
    print("[INFO] 遥感设备已断开")

# ========== 机械臂控制类 ==========
class ArmRobot:
    def __init__(self):
        self.socket_cmd_6001 = -1
        self.socket_servo_7000 = -1
        self.connected = False
        self.arm_lock = threading.Lock()
        self.last_joints = None  # 上一帧关节角

    def connect_dual_port(self):
        with self.arm_lock:
            if self.connected:
                print("[INFO] 机械臂已连接，无需重复连接")
                return
            try:
                self.socket_cmd_6001 = nrc.connect_robot("192.168.1.13", "6001")
                self.socket_servo_7000 = nrc.connect_robot("192.168.1.13", "7000")
                if self.socket_cmd_6001 <= 0 or self.socket_servo_7000 <= 0:
                    raise RuntimeError("端口连接失败")
                self.connected = True
                print("[INFO] 双端口连接成功")
                time.sleep(0.5)
            except Exception as e:
                self.socket_cmd_6001 = -1
                self.socket_servo_7000 = -1
                self.connected = False
                raise e

    def demo_servoJ_init(self):
        with self.arm_lock:
            if not self.connected:
                raise RuntimeError("机械臂未连接")
            fd = self.socket_cmd_6001
            fd2 = self.socket_servo_7000
            nrc.set_servo_state(fd, 1)
            time.sleep(1)
            nrc.set_current_mode(fd, 2)
            nrc.set_speed(fd, 25)
            nrc.set_servo_poweron(fd)
            vmax = [80]*7
            amax = [3000]*7
            jmax = [50000]*7
            nrc.open_servoJ(fd2, vmax, amax, jmax)
            print("[INFO] ServoJ 已开启")
            time.sleep(0.2)

    def get_rpy2quat(self, rx, ry, rz):
        with self.arm_lock:
            if not self.connected:
                return [1.0,0.0,0.0,0.0]
            rpy = nrc.VectorDouble()
            rpy.push_back(rx)
            rpy.push_back(ry)
            rpy.push_back(rz)
            quat_res = nrc.VectorDouble()
            for _ in range(4):
                quat_res.push_back(0.0)
            nrc.get_rpy2quat(self.socket_cmd_6001, rpy, quat_res)
            return [quat_res[0], quat_res[1], quat_res[2], quat_res[3]]

    def quat2rpy(self, q):
        w, x, y, z = q
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x*x + y*y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1, min(1, sinp)))
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return [roll, pitch, yaw]

    def quat_multiply(self, q1, q2):
        w1,x1,y1,z1 = q1
        w2,x2,y2,z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return [w,x,y,z]

    def quat_inverse(self, q):
        w,x,y,z = q
        return [w, -x, -y, -z]

    def get_arm_cartesian_pose(self):
        with self.arm_lock:
            if not self.connected:
                return None
            pos = nrc.MoveCmd()
            nrc.get_joint_position(self.socket_cmd_6001, ARM_AXIS_MODE, pos.targetPosValue)
            return list(pos.targetPosValue)

    def get_inverse_kinematics(self, x, y, z, rx, ry, rz):
        with self.arm_lock:
            if not self.connected:
                return None
            fd = self.socket_cmd_6001
            originPos = nrc.VectorDouble()
            originPos.push_back(x)
            originPos.push_back(y)
            originPos.push_back(z)
            originPos.push_back(rx)
            originPos.push_back(ry)
            originPos.push_back(rz)
            originPos.push_back(0.0)
            targetPos = nrc.VectorDouble()
            for _ in range(7):
                targetPos.push_back(0.0)
            ret = nrc.get_origin_coord_to_target_coord_robot(fd,1,1,originPos,0,targetPos)
            if ret != 0:
                return None
            full_joints = list(targetPos)
            # 根据轴数模式截取结果
            if ARM_AXIS_MODE == 6:
                return full_joints[:6]
            else:
                return full_joints

    # ========== 关节限位检查 ==========
    def clamp_joints(self, joints):
        limits = JOINT_LIMITS_6 if ARM_AXIS_MODE == 6 else JOINT_LIMITS_7
        clamped = []
        for i, val in enumerate(joints):
            low, high = limits[i]
            clamped.append(max(low, min(high, val)))
        return clamped

    # ========== 关节连续性检查 ==========
    def joints_safe(self, new_joints):
        if self.last_joints is None:
            self.last_joints = new_joints.copy()
            return True
        for a, b in zip(new_joints, self.last_joints):
            if abs(a - b) > JOINT_JUMP_THRESHOLD:
                print(f"[WARN] 关节突变过大: {abs(a-b):.1f}°")
                return False
        self.last_joints = new_joints.copy()
        return True

    def servoJ_send(self, joint_target):
        global latest_target_joints, last_log_time
        with self.arm_lock:
            if not self.connected:
                return
            # 1. 硬限位
            joint_target = self.clamp_joints(joint_target)
            # 2. 突变检查
            if not self.joints_safe(joint_target):
                return
            latest_target_joints = joint_target.copy()

            # 统一转为指令下发（底层接口固定7轴参数）
            if ARM_AXIS_MODE == 6:
                send_joints = joint_target + [0.0]
            else:
                send_joints = joint_target

            nrc.set_servoJ_pos(self.socket_servo_7000, send_joints)
            now = time.time()
            if now - last_log_time > LOG_INTERVAL:
                print(f"[JOINT] {[round(i,3) for i in joint_target]}")
                last_log_time = now

    def close_servo(self):
        with self.arm_lock:
            if self.connected:
                nrc.close_servoJ(self.socket_servo_7000)
                nrc.set_servo_poweroff(self.socket_cmd_6001)
                time.sleep(1)
                nrc.set_servo_state(self.socket_cmd_6001,0)
            self.connected = False

# ========== ROS2节点 ==========
class ArmTeleopNode(Node):
    def __init__(self, arm):
        super().__init__('arm_teleop_node')
        self.arm = arm
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = [f"joint{i+1}" for i in range(ARM_AXIS_MODE)]
        self.create_timer(0.05, self.publish_joints)

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        if ROS2_PUBLISH_MODE == 2:
            msg.position = [math.radians(p) for p in latest_target_joints]
        self.joint_pub.publish(msg)

# ========== 主控制循环（奇异区减速 + 最短旋转） ==========
def main_control_loop(arm):
    global running, vr_home_pose, base_arm_quat, vr_home_quat
    CONTROL_PERIOD = 0.01
    base_x=base_y=base_z=0.0
    base_rx=base_ry=base_rz=0.0

    while running:
        t0 = time.time()
        with data_lock:
            pose = pose_data.copy()
            grip = grip_data
        x, y, z, qx, qy, qz, qw = pose
        current_vr_quat = [qx, qy, qz, qw]

        try:
            if xrt.get_A_button() and arm.connected:
                # A键回零，按当前轴数归零
                zero_joints = [0.0] * ARM_AXIS_MODE
                arm.servoJ_send(zero_joints)
                vr_home_pose = None
                base_arm_quat = None
                vr_home_quat = None
                arm.last_joints = None
                time.sleep(1)
                continue
        except:
            pass

        if grip > 0.9 and arm.connected:
            if vr_home_pose is None:
                cart = arm.get_arm_cartesian_pose()
                if cart and len(cart)>=6:
                    base_x, base_y, base_z = cart[0:3]
                    base_rx, base_ry, base_rz = cart[3:6]
                    base_arm_quat = arm.get_rpy2quat(base_rx, base_ry, base_rz)
                    vr_home_pose = [x,y,z]
                    vr_home_quat = current_vr_quat.copy()
        else:
            vr_home_pose = None
            base_arm_quat = None
            vr_home_quat = None

        if vr_home_pose is not None and arm.connected:
            dx = x - vr_home_pose[0]
            dy = y - vr_home_pose[1]
            dz = z - vr_home_pose[2]
            if abs(dx)<POS_DEADZONE: dx=0
            if abs(dy)<POS_DEADZONE: dy=0
            if abs(dz)<POS_DEADZONE: dz=0

            # ========== 奇异区减速 ==========
            if arm.last_joints is not None:
                if ARM_AXIS_MODE == 6:
                    j5, j6 = arm.last_joints[4], arm.last_joints[5]
                    if abs(j5) > SINGULAR_ANGLE or abs(j6) > SINGULAR_ANGLE:
                        scale = SINGULAR_SCALE
                    else:
                        scale = 1.0
                else:
                    j6, j7 = arm.last_joints[5], arm.last_joints[6]
                    if abs(j6) > SINGULAR_ANGLE or abs(j7) > SINGULAR_ANGLE:
                        scale = SINGULAR_SCALE
                    else:
                        scale = 1.0
            else:
                scale = 1.0

            arm_delta_X = -dz * 1000 * POS_SCALE * scale
            arm_delta_Y = -dx * 1000 * POS_SCALE * scale
            arm_delta_Z =  dy * 1000 * POS_SCALE * scale
            arm_delta_X = max(-MAX_POS_DELTA_MM, min(MAX_POS_DELTA_MM, arm_delta_X))
            arm_delta_Y = max(-MAX_POS_DELTA_MM, min(MAX_POS_DELTA_MM, arm_delta_Y))
            arm_delta_Z = max(-MAX_POS_DELTA_MM, min(MAX_POS_DELTA_MM, arm_delta_Z))

            target_x = base_x + arm_delta_X
            target_y = base_y + arm_delta_Y
            target_z = base_z + arm_delta_Z

            # ========== 姿态最短旋转（防止跳变 ±360°） ==========
            vr_quat_wxyz = [qw, qx, qy, qz]
            vr_home_quat_wxyz = [vr_home_quat[3], vr_home_quat[0], vr_home_quat[1], vr_home_quat[2]]
            # 保证最短旋转：点积为负则取反
            if sum(a*b for a,b in zip(vr_quat_wxyz, vr_home_quat_wxyz)) < 0:
                vr_quat_wxyz = [-x for x in vr_quat_wxyz]

            vr_inv = arm.quat_inverse(vr_home_quat_wxyz)
            vr_delta = arm.quat_multiply(vr_quat_wxyz, vr_inv)

            wd, xd, yd, zd = vr_delta
            vr_delta_mapped = [wd, -zd, -xd, yd]
            target_quat = arm.quat_multiply(vr_delta_mapped, base_arm_quat)
            target_rx, target_ry, target_rz = arm.quat2rpy(target_quat)
            target_ry = -target_ry

            ik = arm.get_inverse_kinematics(target_x, target_y, target_z, target_rx, target_ry, target_rz)
            if ik:
                arm.servoJ_send(ik)

        dt = time.time() - t0
        if CONTROL_PERIOD - dt > 0:
            time.sleep(CONTROL_PERIOD - dt)

# ========== 主入口 ==========
def main():
    global running
    arm = ArmRobot()

    try:
        rclpy.init()
        node = ArmTeleopNode(arm)

        vr_thread = threading.Thread(target=device_read_thread, daemon=True)
        vr_thread.start()

        ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
        ros_thread.start()

        print(f"[INFO] 当前模式：{ARM_AXIS_MODE}轴机械臂")
        print("[INFO] 正在连接机械臂...")
        arm.connect_dual_port()
        arm.demo_servoJ_init()
        print(f"[INFO] {ARM_AXIS_MODE}轴机械臂就绪，开始遥操作（已加固奇异点）")

        main_control_loop(arm)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C 退出")
    finally:
        running = False
        arm.close_servo()
        rclpy.shutdown()
        print("[INFO] 程序结束")

if __name__ == '__main__':
    main()

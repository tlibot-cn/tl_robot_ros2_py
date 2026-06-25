#!/usr/bin/env python3
"""天链机械臂 F710 手柄遥操作 — Gazebo 仿真桥接节点。

在 Gazebo 仿真环境中模拟 tl_driver 的 servol 控制链路：
  1. 订阅 /tl_driver/set_servol_pos（ServolMove）
  2. 使用 Pinocchio 库对目标笛卡尔位姿做逆运动学（IK）
  3. 将求解的关节角度发送到 Gazebo 的 position controller

无需 MoveIt2，IK 在节点内本地计算。

已安装依赖：
  - pip3 install pinocchio（纯 Python 调用，依赖已就绪）
  - ros-humble-urdf-parser-py（系统自带）
"""

import math
import os
import threading

import numpy as np
import pinocchio

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from tl_ros2_interface.msg import ServolMove


class ServolSimBridge(Node):
    """Servol → Gazebo 仿真桥接节点。

    使用 Pinocchio 做 IK 求解，不依赖 MoveIt2。
    """

    # 以下由 _init_pinocchio 根据实际模型动态设置
    ndof = None
    joint_names = None

    def __init__(self):
        super().__init__("tl_teleop_f710_sim_bridge")

        # ===== 参数 =====
        self.declare_parameter("arm_type", "tcb605")
        self.declare_parameter(
            "position_controller_topic", "/tcb_group_position_controller/commands"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("ik_eps", 1e-4)
        self.declare_parameter("ik_max_iter", 200)
        self.declare_parameter("ik_dt", 0.5)

        self._arm_type = self.get_parameter("arm_type").value
        self._pos_ctrl_topic = self.get_parameter("position_controller_topic").value
        self._js_topic = self.get_parameter("joint_state_topic").value
        self._ik_eps = self.get_parameter("ik_eps").value
        self._ik_max_iter = int(self.get_parameter("ik_max_iter").value)
        self._ik_dt = self.get_parameter("ik_dt").value

        # ===== 初始化 Pinocchio 运动学模型 =====
        self._model = None
        self._data = None
        self._tip_joint_id = None
        self._tip_frame_id = None
        self._init_pinocchio()

        # ===== 内部状态 =====
        self._current_joints = np.zeros(self.ndof)  # 最新关节角度（弧度）
        self._joints_lock = threading.Lock()

        # ===== 回调组 =====
        self._sub_cb_group = ReentrantCallbackGroup()
        self._pub_cb_group = ReentrantCallbackGroup()

        # ===== 订阅 =====
        self._servol_sub = self.create_subscription(
            ServolMove,
            "/tl_driver/set_servol_pos",
            self._servol_callback,
            10,
            callback_group=self._sub_cb_group,
        )

        self._js_sub = self.create_subscription(
            JointState,
            self._js_topic,
            self._joint_state_callback,
            10,
            callback_group=self._sub_cb_group,
        )

        # ===== 发布：position controller =====
        self._pos_pub = self.create_publisher(
            Float64MultiArray, self._pos_ctrl_topic, 10, callback_group=self._pub_cb_group
        )

        self.get_logger().info(
            "仿真桥接节点已启动（Pinocchio IK, "
            f"max_iter={self._ik_max_iter}, eps={self._ik_eps})"
        )

    # -------- Pinocchio 初始化 --------

    def _init_pinocchio(self):
        """从 URDF 文件加载 Pinocchio 模型。"""
        arm_type = self._arm_type
        urdf_paths = [
            os.path.expanduser(f"~/tl_robot_ros2_py/src/tl_description/urdf/{arm_type}.urdf"),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg_path = get_package_share_directory("tl_description")
            urdf_paths.insert(0, os.path.join(pkg_path, "urdf", f"{arm_type}.urdf"))
        except Exception:
            pass

        loaded = False
        for path in urdf_paths:
            if os.path.exists(path):
                self.get_logger().info(f"加载 URDF: {path}")
                self._model = pinocchio.buildModelFromUrdf(path)
                loaded = True
                break

        if not loaded:
            self.get_logger().fatal(f"找不到 URDF 文件，尝试过: {urdf_paths}")
            raise FileNotFoundError(f"URDF not found: {urdf_paths}")

        self._data = self._model.createData()
        # 根据模型动态确定关节数和末端名称
        self.ndof = self._model.nq
        tip_joint_name = f"joint{self.ndof}"
        tip_link_name = f"link{self.ndof}"
        self.joint_names = [f"joint{i+1}" for i in range(self.ndof)]
        self._tip_joint_id = self._model.getJointId(tip_joint_name)
        self._tip_frame_id = self._model.getFrameId(tip_link_name)
        # 重新初始化当前关节缓存
        self._current_joints = np.zeros(self.ndof)
        self.get_logger().info(
            f"Pinocchio 模型加载完成: {self._model.name}, "
            f"nq={self._model.nq}, njoints={self._model.njoints}, "
            f"tip_joint={tip_joint_name} id={self._tip_joint_id}, "
            f"tip_frame={tip_link_name} id={self._tip_frame_id}"
        )

    # -------- IK 求解（阻尼伪逆法） --------

    def _solve_ik(self, x, y, z, rx, ry, rz, q_guess=None):
        """使用阻尼伪逆法求解 IK。

        Args:
            x, y, z: 目标位置 (m)
            rx, ry, rz: 目标姿态欧拉角 (rad)
            q_guess: 初始关节角 (ndarray, ndof 维)，None 则用当前关节角

        Returns:
            q: 关节角 (ndarray, ndof 维)，失败返回 None
        """
        if q_guess is None:
            with self._joints_lock:
                q_guess = self._current_joints.copy()

        # 构建目标位姿 SE3
        c, s = math.cos, math.sin
        cr, sr = c(rx * 0.5), s(rx * 0.5)
        cp, sp = c(ry * 0.5), s(ry * 0.5)
        cy, sy = c(rz * 0.5), s(rz * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        R = pinocchio.Quaternion(qw, qx, qy, qz).toRotationMatrix()
        target = pinocchio.SE3(R, np.array([x, y, z]))

        q = q_guess.copy().astype(np.float64)
        success = False

        for _ in range(self._ik_max_iter):
            pinocchio.forwardKinematics(self._model, self._data, q)
            pinocchio.updateFramePlacements(self._model, self._data)
            dMi = target.actInv(self._data.oMf[self._tip_frame_id])
            err = pinocchio.log(dMi).vector

            if np.linalg.norm(err) < self._ik_eps:
                success = True
                break

            J = pinocchio.computeJointJacobian(self._model, self._data, q, self._tip_joint_id)
            v = -np.linalg.pinv(J) @ err
            q = pinocchio.integrate(self._model, q, v * self._ik_dt)

        if not success:
            return None

        return q

    # -------- 话题回调 --------

    def _joint_state_callback(self, msg):
        """缓存最新关节状态。"""
        positions = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                positions[name] = msg.position[i]
        if len(positions) == self.ndof:
            with self._joints_lock:
                self._current_joints = np.array([positions[name] for name in self.joint_names])

    def _servol_callback(self, msg):
        """收到 servol 指令 → IK 求解 → 发送到 Gazebo position controller。"""
        target = list(msg.target_pose)
        if len(target) < 6:
            self.get_logger().error(f"target_pose 长度不足: {len(target)}")
            return

        # 单位换算：servol 使用 mm，Pinocchio 使用 m
        x_m = target[0] / 1000.0
        y_m = target[1] / 1000.0
        z_m = target[2] / 1000.0

        # IK 求解
        q = self._solve_ik(x_m, y_m, z_m, target[3], target[4], target[5])

        if q is None:
            self.get_logger().warning(
                f"IK 求解失败 target=({target[0]:.1f}, {target[1]:.1f}, " f"{target[2]:.1f})",
                throttle_duration_sec=2.0,
            )
            return

        # 更新关节缓存为 IK 结果
        with self._joints_lock:
            self._current_joints = q.copy()

        # 发送到 position controller（弧度）
        cmd_msg = Float64MultiArray()
        cmd_msg.data = q.tolist()
        self._pos_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServolSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
import math
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.declare_parameter('target_topic', '/perception/target_xyz')
        self.declare_parameter('cmd_vel_topic', '/uav/cmd_vel_body')
        self.declare_parameter('control_frame', 'base_link_frd')
        self.declare_parameter('kx', 0.35)
        self.declare_parameter('ky', 0.18)
        self.declare_parameter('kz', 0.40)
        self.declare_parameter('k_yaw', 0.95)
        self.declare_parameter('desired_dist', 3.8)
        self.declare_parameter('yaw_enable_vy_deg', 18.0)
        self.declare_parameter('yaw_lpf_tau', 0.45)
        self.declare_parameter('vy_lpf_tau', 0.80)
        self.declare_parameter('max_vx', 3.0)
        self.declare_parameter('max_vy', 3.0)
        self.declare_parameter('max_vz', 1.0)
        self.declare_parameter('max_yaw_rate', 3.0)
        self.declare_parameter('tf_timeout_sec', 0.1)

        target_topic = str(self.get_parameter('target_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.control_frame = str(self.get_parameter('control_frame').value)
        self.sub = self.create_subscription(PointStamped, target_topic, self.callback, 10)
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 追踪增益参数。
        self.kx = float(self.get_parameter('kx').value)
        self.ky = float(self.get_parameter('ky').value)
        self.kz = float(self.get_parameter('kz').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.desired_dist = float(self.get_parameter('desired_dist').value)
        yaw_enable_vy_deg = float(self.get_parameter('yaw_enable_vy_deg').value)
        self.yaw_enable_vy_rad = max(math.radians(yaw_enable_vy_deg), 1e-6)

        # 侧向速度与偏航角速度一阶低通。
        self.yaw_lpf_tau = max(float(self.get_parameter('yaw_lpf_tau').value), 0.0)
        self.vy_lpf_tau = max(float(self.get_parameter('vy_lpf_tau').value), 0.0)
        self.max_vx = abs(float(self.get_parameter('max_vx').value))
        self.max_vy = abs(float(self.get_parameter('max_vy').value))
        self.max_vz = abs(float(self.get_parameter('max_vz').value))
        self.max_yaw_rate = abs(float(self.get_parameter('max_yaw_rate').value))
        self.tf_timeout_sec = max(float(self.get_parameter('tf_timeout_sec').value), 0.0)
        self._last_time = None
        self._yaw_filt = 0.0
        self._vy_filt = 0.0

        self.get_logger().info(
            f"tracker_node started: target_topic={target_topic} cmd_vel_topic={cmd_vel_topic}"
        )

    def callback(self, msg):
        try:
            pt_frd = self.tf_buffer.transform(
                msg, self.control_frame, timeout=Duration(seconds=self.tf_timeout_sec)
            )
        except Exception as exc:
            self.get_logger().warn(f"TF transform failed: {exc}")
            return

        X, Y, Z = pt_frd.point.x, pt_frd.point.y, pt_frd.point.z
        yaw_error = math.atan2(Y, X)

        # 前向速度控制目标距离。
        vx = self.kx * (X - self.desired_dist)

        # 偏航对齐前抑制侧向速度，减少横漂。
        beta = 1.0 - min(abs(yaw_error) / self.yaw_enable_vy_rad, 1.0)
        vy = beta * self.ky * Y

        vz = self.kz * Z
        yaw_rate = self.k_yaw * yaw_error

        # vy 与 yaw_rate 低通滤波。
        now = self.get_clock().now().nanoseconds
        if self._last_time is None:
            dt = 0.0
        else:
            dt = (now - self._last_time) * 1e-9
        self._last_time = now
        if dt > 0.0:
            yaw_alpha = dt / (self.yaw_lpf_tau + dt)
            vy_alpha = dt / (self.vy_lpf_tau + dt)
            self._yaw_filt += yaw_alpha * (yaw_rate - self._yaw_filt)
            self._vy_filt += vy_alpha * (vy - self._vy_filt)
        else:
            self._yaw_filt = yaw_rate
            self._vy_filt = vy

        yaw_rate = self._yaw_filt
        vy = self._vy_filt

        # 输出限幅。
        vx = max(min(vx, self.max_vx), -self.max_vx)
        vy = max(min(vy, self.max_vy), -self.max_vy)
        vz = max(min(vz, self.max_vz), -self.max_vz)
        yaw_rate = max(min(yaw_rate, self.max_yaw_rate), -self.max_yaw_rate)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = yaw_rate
        self.pub.publish(cmd)

        # self.get_logger().info(
        #     f"cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}"
        # )
        # self.get_logger().info(f"FRD XYZ: ({X:.2f}, {Y:.2f}, {Z:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

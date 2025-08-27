import math
from typing import List, Sequence

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, SetParametersResult
from sensor_msgs.msg import JointState


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class SimpleJointStatePublisher(Node):
    """
    JointState publisher tối giản, tương thích URDF:
      - Joints: base_yaw (±π), shoulder_pitch (±π/2), elbow_pitch (±π/2)
      - Tham số:
          * positions (rad)
          * use_sine_demo (demo dao động trong giới hạn)
          * rate_hz (tần số publish)
          * frame_id (gắn vào header để RViz khỏi phàn nàn)
    """

    DEFAULT_NAMES = ['base_yaw', 'shoulder_pitch', 'elbow_pitch']
    DEFAULT_LOWER = [-math.pi, -math.pi/2, -math.pi/2]
    DEFAULT_UPPER = [ math.pi,  math.pi/2,  math.pi/2]

    def __init__(self):
        super().__init__('js_pub')  # tên node riêng để tránh đụng với node chuẩn

        # -------- Declare params --------
        self.declare_parameter('joint_names', self.DEFAULT_NAMES)
        self.declare_parameter('positions', [0.0, 0.0, 0.0])
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('use_sine_demo', False)
        self.declare_parameter('limits_lower', self.DEFAULT_LOWER)
        self.declare_parameter('limits_upper', self.DEFAULT_UPPER)
        self.declare_parameter('frame_id', 'base')  # <<< thêm frame_id

        # -------- Read params --------
        self.joint_names: List[str] = list(self.get_parameter('joint_names').value)
        self.positions: List[float] = list(self.get_parameter('positions').value) or [0.0] * len(self.joint_names)
        self.use_sine_demo: bool = bool(self.get_parameter('use_sine_demo').value)
        self.frame_id = 'base'

        # Giới hạn
        lower = list(self.get_parameter('limits_lower').value)
        upper = list(self.get_parameter('limits_upper').value)
        self.limits_lower = self._pad_or_trim(lower, len(self.joint_names), fill=-math.inf)
        self.limits_upper = self._pad_or_trim(upper, len(self.joint_names), fill= math.inf)
        self._fix_limits_shape()

        # Rate + timer
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(self._period_from_rate(self.rate_hz), self._tick)

        # Dynamic param callback
        self.add_on_set_parameters_callback(self._on_params)

        self.t = 0.0
        self._sanitize_positions()
        self.get_logger().info(
            f'Publishing joint_states for: {self.joint_names} @ {self.rate_hz:.1f} Hz, frame_id="{self.frame_id}"'
        )
        self.get_logger().info('TIP: Trong RViz, đặt Global Options → Fixed Frame = base (hoặc base_link nếu URDF dùng tên đó).')

    # ---------- Utilities ----------
    def _period_from_rate(self, rate_hz: float) -> float:
        rate = max(0.1, float(rate_hz))
        return 1.0 / rate

    @staticmethod
    def _pad_or_trim(seq: Sequence[float], n: int, fill: float = 0.0) -> List[float]:
        s = list(seq[:n])
        while len(s) < n:
            s.append(fill)
        return s

    def _fix_limits_shape(self):
        for i in range(len(self.joint_names)):
            lo, up = self.limits_lower[i], self.limits_upper[i]
            if lo > up:
                self.get_logger().warn(f'limits_lower[{i}] > limits_upper[{i}] → hoán đổi.')
                self.limits_lower[i], self.limits_upper[i] = up, lo

    def _sanitize_positions(self):
        n = len(self.joint_names)
        self.positions = self._pad_or_trim(self.positions, n, fill=0.0)
        self.positions = [clamp(p, self.limits_lower[i], self.limits_upper[i]) for i, p in enumerate(self.positions)]

    # ---------- Param callback ----------
    def _on_params(self, params: List[Parameter]) -> SetParametersResult:
        try:
            for p in params:
                if p.name == 'positions' and p.type_ == Parameter.TYPE_DOUBLE_ARRAY:
                    arr = list(p.value)
                    if len(arr) != len(self.joint_names):
                        self.get_logger().warn('`positions` length mismatch với joint_names; bỏ qua.')
                    else:
                        self.positions = [clamp(arr[i], self.limits_lower[i], self.limits_upper[i]) for i in range(len(arr))]
                        self.get_logger().info(f'Updated positions -> {self.positions}')

                elif p.name == 'rate_hz' and p.type_ in (Parameter.TYPE_DOUBLE, Parameter.TYPE_INTEGER):
                    new_rate = max(0.1, float(p.value))
                    if abs(new_rate - self.rate_hz) > 1e-6:
                        self.rate_hz = new_rate
                        self.timer.cancel()
                        self.timer = self.create_timer(self._period_from_rate(self.rate_hz), self._tick)
                        self.get_logger().info(f'Updated rate_hz -> {self.rate_hz:.1f} Hz')

                elif p.name == 'use_sine_demo' and p.type_ == Parameter.TYPE_BOOL:
                    self.use_sine_demo = bool(p.value)
                    self.get_logger().info(f'Updated use_sine_demo -> {self.use_sine_demo}')

                elif p.name == 'limits_lower' and p.type_ == Parameter.TYPE_DOUBLE_ARRAY:
                    self.limits_lower = self._pad_or_trim(list(p.value), len(self.joint_names), fill=-math.inf)
                    self._fix_limits_shape()
                    self._sanitize_positions()
                    self.get_logger().info('Updated limits_lower')

                elif p.name == 'limits_upper' and p.type_ == Parameter.TYPE_DOUBLE_ARRAY:
                    self.limits_upper = self._pad_or_trim(list(p.value), len(self.joint_names), fill=math.inf)
                    self._fix_limits_shape()
                    self._sanitize_positions()
                    self.get_logger().info('Updated limits_upper')

                elif p.name == 'frame_id' and p.type_ == Parameter.TYPE_STRING:
                    self.frame_id = str(p.value).strip()
                    self.get_logger().info(f'Updated frame_id -> "{self.frame_id}"')

            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().error(f'Param update failed: {e}')
            return SetParametersResult(successful=False)

    # ---------- Main tick ----------
    def _tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id  # <<< gán frame_id
        msg.name = list(self.joint_names)

        if self.use_sine_demo:
            self.t += self._period_from_rate(self.rate_hz)
            pos = []
            for i, (lo, up) in enumerate(zip(self.limits_lower, self.limits_upper)):
                mid = 0.5 * (lo + up)
                amp = 0.45 * (up - lo)
                w = 0.6 + 0.2 * i
                pos.append(mid + amp * math.sin(self.t * w))
            msg.position = pos
        else:
            msg.position = [clamp(self.positions[i], self.limits_lower[i], self.limits_upper[i])
                            for i in range(len(self.joint_names))]

        # velocity/effort có thể để trống; nếu muốn điền 0 cùng độ dài:
        # n = len(msg.name)
        # msg.velocity = [0.0]*n
        # msg.effort   = [0.0]*n

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimpleJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
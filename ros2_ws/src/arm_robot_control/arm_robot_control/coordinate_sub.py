import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Point
import numpy as np
import tf_transformations
from tf2_ros import Buffer, TransformListener


# Arm lengths (m)
L0 = 0.04
L1 = 0.12
L2 = 0.165

def inverse_kinematics(x, y, z, L0=L0, L1=L1, L2=L2, phi_deg=-45, return_radians=False):
    phi = np.radians(phi_deg)

    if x == 0 and y == 0:
        t1 = 0.0
    else:
        t1 = np.arctan2(y, x)

    nx = x * np.cos(t1) + y * np.sin(t1)
    ny = z - L0

    wx = nx - L2 * np.cos(phi)
    wy = ny - L2 * np.sin(phi)

    d2 = wx**2 + wy**2

    if d2 < 1e-9:
        raise ValueError("Wrist point too close to base (unreachable)")

    c2 = (d2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = np.clip(c2, -1.0, 1.0)
    s2 = np.sqrt(max(0.0, 1 - c2**2))

    t3 = np.arctan2(s2, c2)
    t2 = np.arctan2(wy, wx) - np.arctan2(L2 * s2, L1 + L2 * c2)

    t3 = phi - t2

    if return_radians:
        return (t1, t2, t3)

    sol_deg = tuple(np.degrees(a) for a in (t1, t2, t3))
    return sol_deg


class PixelToRobot(Node):
    def __init__(self):
        super().__init__('pixel_to_robot')
        self.fx = self.fy = self.cx = self.cy = None
        self.shape_name = None
        self.default_depth = 0.245

        # TF2 buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.info_callback,
            10
        )

        self.sub_coord = self.create_subscription(
            Point,
            '/object/coordinate',
            self.pixel_callback,
            10
        )

        self.sub_name = self.create_subscription(
            String,
            '/object/name',
            self.name_callback,
            10
        )

        # Publisher
        self.pub_angles = self.create_publisher(
            Float32MultiArray,
            '/servo_angles',
            10
        )

    def info_callback(self, msg: CameraInfo):
        K = msg.k
        self.fx, self.fy = float(K[0]), float(K[4])
        self.cx, self.cy = float(K[2]), float(K[5])
        self.get_logger().info(
            f"Loaded camera intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
        )

    def name_callback(self, msg: String):
        self.shape_name = msg.data
        self.get_logger().debug(f"Received shape name: {self.shape_name}")

    def pixel_callback(self, msg: Point):
        if self.fx is None:
            self.get_logger().warn("No camera intrinsics yet; ignoring coordinate")
            return

        # Convert pixel to camera coordinates
        u = float(msg.x)
        v = float(msg.y)
        Z = self.default_depth

        # u = 640 - u 
        # v = 480 - v 
        Xc = (u - self.cx) * Z / self.fx
        Yc = (v - self.cy) * Z / self.fy
        Zc = Z
        P_cam = np.array([[Xc], [Yc], [Zc]])

        # camera_link -> robot_link
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame="base_link",  
                source_frame="camera_link",  
                time=rclpy.time.Time() 
            )
        except Exception as e:
            self.get_logger().warn(f"No TF from camera_link to robot_link yet: {e}")
            return

        # Extract translation
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z

        # Extract rotation
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w

        # Quaternion -> rotation matrix
        R_tf = tf_transformations.quaternion_matrix([qx, qy, qz, qw])[:3, :3]
        t_tf = np.array([[tx], [ty], [tz]])

        # Apply transform: robot_frame = R * camera_frame + t
        P_robot = R_tf @ P_cam + t_tf
        xr, yr, zr = P_robot.flatten()

        try:
            angles = inverse_kinematics(xr, yr, zr)
        except Exception as e:
            self.get_logger().error(
                f"IK failure for point ({xr:.3f},{yr:.3f},{zr:.3f}): {e}"
            )
            return

        # Publish angles
        out = Float32MultiArray()
        out.data = [float(angles[0]), float(angles[1]), float(angles[2])]
        self.pub_angles.publish(out)

        if self.shape_name:
            self.get_logger().info(
                f"Detected {self.shape_name}: pixel({u:.1f},{v:.1f},Z={Z:.3f}) -> "
                f"robot({xr:.3f},{yr:.3f},{zr:.3f}) -> angles {out.data}"
            )
            self.shape_name = None
        else:
            self.get_logger().info(
                f"Pixel({u:.1f},{v:.1f},Z={Z:.3f}) -> "
                f"robot({xr:.3f},{yr:.3f},{zr:.3f}) -> angles {out.data}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PixelToRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
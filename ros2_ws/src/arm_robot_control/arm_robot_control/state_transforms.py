import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math 
from tf_transformations import quaternion_from_euler


'''
ros2 run tf2_ros static_transform_publisher 0.3 0 0.245 0 0 3.14159 base_link camera_link
'''
class StateTransforms(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()
    
    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = 0.3
        t.transform.translation.y = 0 
        t.transform.translation.z = 0.245 
        
        roll = 0
        pitch = 0
        yaw = math.radians(180)
        q = quaternion_from_euler(roll, pitch, yaw)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.broadcaster.sendTransform(t)
        self.get_logger().info('Publisher static transform between bash_link and camera_link')

def main(args = None):
    rclpy.init(args = args)
    node = StateTransforms()
    rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
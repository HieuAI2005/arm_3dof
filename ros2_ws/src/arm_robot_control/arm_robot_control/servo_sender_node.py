import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class SerialSenderNode(Node):
    def __init__(self):
        super().__init__('serial_sender')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/servo_angles',
            self.listener_callback,
            10
        )
        self.serial_port = '/dev/ttyACM0'
        self.baudrate = 9600
        self.ser = None

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            time.sleep(2) 
            self.get_logger().info(f"Connected serial: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Error of serial: {e}")

    def listener_callback(self, msg):
        data = msg.data
        msg_str = ','.join(str(x) for x in data) + '\n'
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(msg_str.encode())
                self.get_logger().info(f"Done: {msg_str.strip()}")
            except Exception as e:
                self.get_logger().error(f"Error of data: {e}")
        else:
            self.get_logger().error("Close!")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

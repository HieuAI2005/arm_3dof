import rclpy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from rclpy.node import Node
import cv2 

class ImageDisplay(Node):
    def __init__(self):
        super().__init__('camera_sub')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        print("Done")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        cv2.imshow('Camera', frame)
        cv2.waitKey(1)
        
def main(args = None):
    rclpy.init(args = args)
    node = ImageDisplay()  
    try:
        rclpy.spin(node)   
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown() 
        
if __name__ == '__main__':
    main()
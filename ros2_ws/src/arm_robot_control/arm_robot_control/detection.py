import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class Detection(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self.roi_x, self.roi_y, self.roi_w, self.roi_h = 20, 50, 580, 350
        self.min_area = 100
        self.max_objects = 10 
        
        self.first = True
        self.last_publish_time = time.time()
        self.publish_interval = 10

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.publisher_image = self.create_publisher(Image, '/image/object_detect', 10)
        self.publisher_coor = self.create_publisher(Point, '/object/coordinate', 10)
        self.name_pub = self.create_publisher(String, '/object/name', 10)

    def detect_shape(self, cnt, approx=None):
        peri = cv2.arcLength(cnt, True)
        if approx is None:
            approx = cv2.approxPolyDP(cnt, 0.015 * peri, True)
        vertices = len(approx)
        shape_name = 'unknown'

        area = cv2.contourArea(cnt)
        if area == 0:
            return shape_name

        circularity = 4 * np.pi * area / (peri * peri) if peri > 0 else 0.0
        if vertices == 3:
            shape_name = 'triangle'
        elif vertices == 4:
            x, y, w, h = cv2.boundingRect(approx)
            ar = w / float(h) if h != 0 else 0.0
            if 0.9 <= ar <= 1.1:
                shape_name = 'square'
            else:
                shape_name = 'rectangle'
        else:
            if circularity >= 0.80:
                shape_name = 'circle'
            else:
                shape_name = 'polygon'
        return shape_name

    def shape_detection(self, img):
        h_img, w_img = img.shape[:2]
        rx = max(0, min(self.roi_x, w_img - 1))
        ry = max(0, min(self.roi_y, h_img - 1))
        rw = max(1, min(self.roi_w, w_img - rx))
        rh = max(1, min(self.roi_h, h_img - ry))

        roi = img[ry: ry + rh, rx: rx + rw].copy()
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.015 * peri, True)
            shape_name = self.detect_shape(cnt, approx=approx)

            M = cv2.moments(cnt)
            if M.get('m00', 0) != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
            else:
                x, y, w, h = cv2.boundingRect(approx)
                cX, cY = x + w // 2, y + h // 2

            cX_img = cX + rx
            cY_img = cY + ry

            centers.append((cX_img, cY_img, shape_name))

            cv2.drawContours(roi, [approx], -1, (0, 255, 0), 2)
            cv2.putText(roi, shape_name, (max(0, cX - 30), max(0, cY - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        img[ry: ry + rh, rx: rx + rw] = roi
        cv2.rectangle(img, (rx, ry), (rx + rw, ry + rh), (0, 0, 255), 2)
        return img, centers

    def image_callback(self, msg):
        if self.first == False:
            now = time.time()
            if now - self.last_publish_time < self.publish_interval:
                return  

            self.last_publish_time = now 

        self.first = False
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        processed_img, centers = self.shape_detection(cv_image)

        try:
            image_out = self.bridge.cv2_to_imgmsg(processed_img, encoding='bgr8')
            self.publisher_image.publish(image_out)
        except Exception as e:
            self.get_logger().error(f'CvBridge Error (publish): {e}')

        count = 0
        for (x, y, name) in centers:
            if name in ('circle', 'square', 'triangle'):
                coord = Point()
                coord.x = float(x)
                coord.y = float(y)
                coord.z = 0.0
                self.publisher_coor.publish(coord)

                name_msg = String()
                name_msg.data = name
                self.name_pub.publish(name_msg)

                count += 1
                self.get_logger().info(f"Published: ({x:.1f}, {y:.1f}) - {name}")

                if count >= self.max_objects:
                    break

def main(args=None):
    rclpy.init(args=args)
    shape_detect = Detection()
    try:
        rclpy.spin(shape_detect)
    except KeyboardInterrupt:
        pass
    finally:
        shape_detect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import os
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from cv_bridge import CvBridge
from typing import Tuple

class Detector(Node):

    def __init__(self):
        super().__init__('detector')
        self.cv_bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.target_pub = self.create_publisher(Detection2D, "detection/detector", 10)
        self.detecting = False

        self.get_logger().info("Detector Node has been started.")


    def publish_detection(self, bbox: Tuple[int, int, int, int]):
        x, y, w, h = bbox
        detection_msg = Detection2D()

        # Convert bbox coordinates to integers
        x, y, w, h = int(x), int(y), int(w), int(h)

        detection_msg.bbox.center.x = float(x + w // 2) # Cx bbox
        detection_msg.bbox.center.y = float(y + h // 2) # Cy bbox
        detection_msg.bbox.size_x = float(w) # Width of bbox
        detection_msg.bbox.size_y = float(h) # Height of bbox

        self.target_pub.publish(detection_msg)


    def image_callback(self, msg):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if not self.detecting:
                bbox = cv2.selectROI(frame, False)
                self.detecting = True
                # print(bbox)

                self.publish_detection(bbox=bbox)

        except Exception as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))



def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

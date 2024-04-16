import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, BoundingBox2D
from cv_bridge import CvBridge, CvBridgeError
import time
from typing import Tuple

def get_xywh_from_bbox_2d(bbox: BoundingBox2D) -> Tuple[int, int, int, int]:
    """
    Calculate the coordinates and dimensions of a 2D bounding box.

    Parameters:
    - bbox (BoundingBox2D): The 2D bounding box object containing center position and size.

    Returns:
    - Tuple[int, int, int, int]: A tuple containing four integers representing the x-coordinate,
      y-coordinate, width, and height of the bounding box.
    """
    center = bbox.center
    size_x = bbox.size_x
    size_y = bbox.size_y

    x = int(center.x - (size_x / 2))
    y = int(center.y - (size_y / 2))
    w = int(size_x)
    h = int(size_y)

    return x, y, w, h


class Tracker(Node):

    def __init__(self):
        super().__init__("tracker")
        self.cv_bridge_ = CvBridge()
        self.tracker_ = cv2.TrackerKCF_create()
        self.tracker_was_init_ = False
        self.new_bbox_: Tuple[int, int, int, int] = None
        self.prev_time_ = time.time()
        self.fps_ = 0

        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2D, "detection/detector", self.detection_callback, 10
        )
        self.detection_pub = self.create_publisher(
            Image, "detection/tracker", 10
        )

        self.get_logger().info("Tracker Node has been started.")


    def image_callback(self, msg):
        try:
            frame = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if not self.tracker_was_init_ and self.new_bbox_ is None:
                self.get_logger().info("Skip tracking due to missing initialization of tracker.")  
                return
  
            if self.new_bbox_:
                self.tracker_.init(frame, self.new_bbox_)
                # self.new_bbox is not None hence we can initialize it. 
                if not self.tracker_was_init_:
                    self.tracker_was_init_ = True

                self.get_logger().info("Tracker was reinitialized due to new bbox.")
                self.new_bbox_ = None

            ok, tracker_bbox = self.tracker_.update(frame)
            if ok:
                self.get_logger().info("Tracking ok")
                pt1 = (int(tracker_bbox[0]), int(tracker_bbox[1]))
                pt2 = (
                    int(tracker_bbox[0] + tracker_bbox[2]),
                    int(tracker_bbox[1] + tracker_bbox[3]),
                )
                cv2.rectangle(frame, pt1=pt1, pt2=pt2, color=(0, 255, 0), thickness=2)

                # Calculate FPS
                current_time = time.time()
                self.fps_ = 1 / (current_time - self.prev_time_)
                self.prev_time_ = current_time

                # Display FPS on frame
                cv2.putText(
                    frame,
                    f"FPS: {int(self.fps_)}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )

                try:
                    detection_msg = self.cv_bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
                except CvBridgeError as e:
                    self.get_logger().error(str(e))
            
                self.detection_pub.publish(detection_msg)

            else:
                self.get_logger().info("Tracking failure")

        except Exception as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))


    def detection_callback(self, msg: Detection2D):
        if msg.bbox:
            self.get_logger().info("Got detection from detector.")
            self.new_bbox_ = get_xywh_from_bbox_2d(bbox=msg.bbox)


def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2


class ROSBridge(Node):
    def __init__(self):
        super().__init__('triage_ros_bridge')

        self.bridge = CvBridge()
        self.latest_images = {
            "drone1": None,
            "drone2": None,
            "drone3": None,
            "drone4": None,
            "drone5": None,
        }
        self.latest_location = None

        self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.create_subscription(
            PointStamped,
            '/triage/location',
            self.location_callback,
            10
        )

        self.get_logger().info("ROSBridge node started")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_img)
            img_bytes = jpeg.tobytes()

            # Mirror single feed into all 5 drones
            for k in self.latest_images:
                self.latest_images[k] = img_bytes

        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

    def location_callback(self, msg):
        self.latest_location = {
            "lat": msg.point.y,
            "lon": msg.point.x
        }


def start_ros(node):
    rclpy.spin(node)

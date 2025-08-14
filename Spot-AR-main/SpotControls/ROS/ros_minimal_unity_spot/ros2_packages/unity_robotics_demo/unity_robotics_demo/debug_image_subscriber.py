import rclpy
from rclpy.node import Node

from unity_robotics_demo_msgs.msg import SpotImage
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

import sys
import numpy as np
from cv_bridge import CvBridge
from cv2 import imwrite

test_file_name = "/Test_image.jpg"

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            SpotImage,
            "spot_image_" + sys.argv[1],
            self.listener_callback,
            50)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        cv_image = CvBridge().imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
        imwrite(test_file_name, cv_image)
        self.get_logger().info("File saved as: " + test_file_name)
        self.get_logger().info("--------------------------")


def main(args=None):
    rclpy.init(args=args)

    subscriber = ImageSubscriber()

    rclpy.spin(subscriber)


if __name__ == '__main__':
    main()
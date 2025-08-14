import rclpy
from rclpy.node import Node

from unity_robotics_demo_msgs.srv import GetSpotCamera

import sys
import numpy as np
from cv_bridge import CvBridge
from cv2 import imwrite

test_file_name = "/Test_image.jpg"

class ImageClientAsync(Node):

	def __init__(self):
		super().__init__('image_client_async')
		self.cli = self.create_client(GetSpotCamera, 'get_spot_image')
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.req = GetSpotCamera.Request()

	def send_request(self, camera_name, pixel_format, quality):
		self.req.camera_name = camera_name
		self.req.pixel_format = pixel_format
		self.req.quality = quality
		self.future = self.cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()


def main():
	rclpy.init()

	image_client = ImageClientAsync()
	response = image_client.send_request(str(sys.argv[1]), str(sys.argv[2]), int(sys.argv[3]))

	# https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
	cv_image = CvBridge().imgmsg_to_cv2(response.image, desired_encoding='passthrough')
	imwrite(test_file_name, cv_image)
	image_client.get_logger().info(f'Result of get_spot_image for {str(sys.argv[1])}: test image saved as {test_file_name}')

	image_client.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
# ROS2
import rclpy
from rclpy.node import Node
# ROS2 messages/services
from unity_robotics_demo_msgs.srv import GetSpotCamera
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
# OpenCV
import numpy as np
import cv2
from cv_bridge import CvBridge
# Boston Dynamics
import argparse
import bosdyn.client
import bosdyn.client.util
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client import ResponseError, RpcError, create_standard_sdk


def pixel_format_type_strings():
	names = image_pb2.Image.PixelFormat.keys()
	return names[1:]

def pixel_format_string_to_enum(enum_string):
	return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)

class GetSpotImageService(Node):

	def __init__(self, image_client):
		super().__init__('get_spot_image_service')
		self.srv = self.create_service(GetSpotCamera, 'get_spot_image', self.image_callback)
		self.bridge = CvBridge()
		self.image_client = image_client
		self.i = 0

	def image_callback(self, request, response):
		## Get camera
		camera_name = request.camera_name
		self.get_logger().info(f'Incoming request for camera: {camera_name}')
		## Get image
		'''
		# https://docs.ros2.org/latest/api/builtin_interfaces/msg/Time.html
		timestamp = Time()
		timestamp.sec = 10000
		timestamp.nanosec = 100
		# https://docs.ros2.org/latest/api/std_msgs/msg/Header.html
		image_header = Header() 
		image_header.stamp = timestamp
		image_header.frame_id = camera_name
		# https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
		image = Image()
		image.header = image_header
		image.height = 100
		image.width = 100
		image.encoding = "TODO"
		image.is_bigendian = 1
		image.step = 800
		image.data = [1, 2, 3]
		'''
		'''
		cv_image = np.zeros((100,100,3), np.uint8) # https://stackoverflow.com/questions/12881926/create-a-new-rgb-opencv-image-using-python
		'''

		'''
		pixel_format = pixel_format_string_to_enum(request.pixel_format)
		'''

		pixel_format = pixel_format_string_to_enum(request.pixel_format)
		image_request = [
			build_image_request(source, pixel_format=pixel_format)
			for source in [camera_name]
		]
		image_responses = self.image_client.get_image(image_request)
		img = None
		for image in image_responses:
			num_bytes = 1  # Assume a default of 1 byte encodings.
			if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
				dtype = np.uint16
				extension = '.png'
			else:
				if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
					num_bytes = 3
				elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
					num_bytes = 4
				elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
					num_bytes = 1
				elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
					num_bytes = 2
				dtype = np.uint8
				extension = '.jpg'

				img = np.frombuffer(image.shot.image.data, dtype=dtype)
				if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
					try:
						# Attempt to reshape array into a RGB rows X cols shape.
						img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_bytes))
					except ValueError:
						# Unable to reshape the image data, trying a regular decode.
						img = cv2.imdecode(img, -1)
				else:
					img = cv2.imdecode(img, -1)

		# https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
		#image = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
		ros_image = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")

		## Broadcast image response
		response.image = ros_image
		self.i = self.i + 1
		return response


def main():
	bosdyn.client.util.setup_logging()

	# Get input args
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Setup robot
	sdk = create_standard_sdk('SENSEable_ROSCamera')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)
	image_client = robot.ensure_client(ImageClient.default_service_name)

	# Spin up ROS2 node
	rclpy.init()
	service = GetSpotImageService(image_client)
	rclpy.spin(service)
	rclpy.shutdown()


if __name__ == '__main__':
	main()
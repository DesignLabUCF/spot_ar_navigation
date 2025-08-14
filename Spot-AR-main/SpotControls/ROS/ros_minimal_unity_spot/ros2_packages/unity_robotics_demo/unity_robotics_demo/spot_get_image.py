import sys
# ROS2
import rclpy
from rclpy.node import Node
# ROS2 messages/services
from unity_robotics_demo_msgs.msg import SpotImage
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
# Image processing
import numpy as np
import cv2
from cv_bridge import CvBridge
#from scipy import ndimage # TODO replace with cheaper alternative
# Boston Dynamics
import argparse
import bosdyn.client
import bosdyn.client.util
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
# Dynamic
#_LOGGER = logging.getLogger('ros_image_plugin')
PUBLISH_FPS = 5
topic = "spot_image_" # Must append camera name

'''
ROTATION_ANGLE = {
	'back_fisheye_image': 0,
	'frontleft_fisheye_image': -78,
	'frontright_fisheye_image': -102,
	'left_fisheye_image': 0,
	'right_fisheye_image': 180
}
'''


def pixel_format_type_strings():
	names = image_pb2.Image.PixelFormat.keys()
	return names[1:]

def pixel_format_string_to_enum(enum_string):
	return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)

class ImagePublisher(Node):

	def __init__(self, image_client, camera_name, pixel_format):
		super().__init__('image_publisher')
		self.topic = topic + camera_name
		self.publisher_ = self.create_publisher(SpotImage, self.topic, 10)
		timer_period = 1.0 / float(PUBLISH_FPS)  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.bridge = CvBridge()
		self.image_client = image_client
		self.camera_name = camera_name
		self.pixel_format = pixel_format

	def timer_callback(self):
		self.get_logger().info(f'Publishing for topic: {self.topic}')
		## Get image
		pixel_format = pixel_format_string_to_enum(self.pixel_format)
		image_request = [
			build_image_request(source, pixel_format=pixel_format)
			for source in [self.camera_name]
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

			'''
			# Rotate image. Uses ndimage from scipy, so likely need to find alternative means to trim space in future
			img = ndimage.rotate(img, ROTATION_ANGLE[self.camera_name])
			'''

		# Swap red and blue channels for format 'PIXEL_FORMAT_RGB_U8' because Unity does not support a BGR24 image format
		img = img[:, :, ::-1]

		# https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
		#image = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
		ros_image = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")

		# Create rest of message
		msg = SpotImage()
		msg.camera_name = self.camera_name
		msg.pixel_format = self.pixel_format
		msg.quality = 50
		msg.image = ros_image

		## Broadcast image response
		self.publisher_.publish(msg)
		self.i = self.i + 1

'''
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
'''

def main():
	spot_ip = sys.argv[1]
	camera_name = sys.argv[2]
	pixel_format = sys.argv[3]

	bosdyn.client.util.setup_logging()

	# Get input args
	#parser = argparse.ArgumentParser()
	#bosdyn.client.util.add_base_arguments(parser)
	#options = parser.parse_args()

	# Setup robot
	sdk = create_standard_sdk('SENSEable_ROSCamera_' + camera_name)
	robot = sdk.create_robot(spot_ip)
	bosdyn.client.util.authenticate(robot)
	image_client = robot.ensure_client(ImageClient.default_service_name)

	# Spin up ROS2 node
	rclpy.init()
	pub = ImagePublisher(image_client, camera_name, pixel_format)
	rclpy.spin(pub)

if __name__ == '__main__':
	main()
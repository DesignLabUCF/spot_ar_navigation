# ROS
import rclpy
from rclpy.node import Node
from unity_robotics_demo_msgs.msg import SpotTransform
from unity_robotics_demo_msgs.msg import TagAndCameraTransform
# Python
import argparse
import time
import logging
import threading
# Boston Dynamics
import bosdyn.client
import bosdyn.client.util
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api import geometry_pb2
from bosdyn.api import world_object_pb2
from bosdyn.client.frame_helpers import (get_odom_tform_body, get_vision_tform_body, get_a_tform_b, add_edge_to_tree, get_frame_names)
from bosdyn.util import now_timestamp
from bosdyn.client.world_object import WorldObjectClient, make_add_world_object_req
# Dynamic
_LOGGER = logging.getLogger('ros_transform_plugin')
APRIL_TAG_ID = 4
VISION_CHECK_FPS = 20
TRANSFORM_PUBLISH_FPS = 10
current_vision_position = None
current_vision_rotation = None
most_recent_april_position = None
most_recent_april_rotation = None

def update_transforms(robot_state_client, world_object_client):
	global current_vision_position
	global current_vision_rotation
	global most_recent_april_position
	global most_recent_april_rotation

	while True:
		# Make a robot state request
		state = robot_state_client.get_robot_state()
		kinematic_state = state.kinematic_state
		# Get robot position
		robot_transform_tree = kinematic_state.transforms_snapshot
		vision_transform = get_vision_tform_body(robot_transform_tree)
		current_vision_position = vision_transform.position
		current_vision_rotation = vision_transform.rotation
		# Find fiducual in world objects detected list
		world_objects = world_object_client.list_world_objects().world_objects
		for obj in world_objects: # Look at every world object
			if obj.name == ("world_obj_apriltag_" + str(APRIL_TAG_ID)): # This world object is our desired april tag
				# Get this april tag's transform
				april_tree = obj.transforms_snapshot
				april_key = "filtered_fiducial_" + str(APRIL_TAG_ID)
				april_from_vision = get_a_tform_b(april_tree, "vision", april_key)
				# Update our global tracking values
				most_recent_april_position = april_from_vision.position
				most_recent_april_rotation = april_from_vision.rotation
		# Repeat with a small lapse between
		time.sleep(1.0 / VISION_CHECK_FPS)


class TransformPublisher(Node):

	def __init__(self):
		super().__init__('transform_publisher')
		self.publisher_ = self.create_publisher(TagAndCameraTransform, 'spot_tagandcamera', 10)
		timer_period = 1.0 / float(TRANSFORM_PUBLISH_FPS)  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0	
		
	def timer_callback(self):
		global current_vision_position
		global current_vision_rotation
		global most_recent_april_position
		global most_recent_april_rotation

		# April Tag
		april_transform = SpotTransform()
		if not (most_recent_april_position == None): # An april tag has been found
			april_transform.pos_x = float(most_recent_april_position.x)
			april_transform.pos_y = float(most_recent_april_position.y)
			april_transform.pos_z = float(most_recent_april_position.z)
			april_transform.rot_x = float(most_recent_april_rotation.x)
			april_transform.rot_y = float(most_recent_april_rotation.y)
			april_transform.rot_z = float(most_recent_april_rotation.z)
			april_transform.rot_w = float(most_recent_april_rotation.w)
		else:
			april_transform.pos_x = float(-100.0)
			april_transform.pos_y = float(-100.0)
			april_transform.pos_z = float(-100.0)
			april_transform.rot_x = float(-100.0)
			april_transform.rot_y = float(-100.0)
			april_transform.rot_z = float(-100.0)
			april_transform.rot_w = float(-100.0)
		# Camera
		camera_transform = SpotTransform()
		camera_transform.pos_x = float(current_vision_position.x)
		camera_transform.pos_y = float(current_vision_position.y)
		camera_transform.pos_z = float(current_vision_position.z)
		camera_transform.rot_x = float(current_vision_rotation.x)
		camera_transform.rot_y = float(current_vision_rotation.y)
		camera_transform.rot_z = float(current_vision_rotation.z)
		camera_transform.rot_w = float(current_vision_rotation.w)	
		# Combined
		transform = TagAndCameraTransform()
		transform.tag = april_transform
		transform.camera = camera_transform
		# Publish message
		self.get_logger().info(f'Publishing: {transform}')
		self.publisher_.publish(transform)	
		self.i = self.i + 1


def main():
	bosdyn.client.util.setup_logging()

	# Get input args
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Setup robot
	sdk = create_standard_sdk('SENSEable_ROSStatus')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)
	robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
	world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)

	# Launch camera read and transform getting thread
	#self._monitor_thread = threading.Thread(target=self.update_transforms, daemon=True, args=(device,)) # Daemon threads will auto-die when main thread does
	update_thread = threading.Thread(target=update_transforms, daemon=True, args=(robot_state_client, world_object_client,)) # Daemon threads will auto-die when main thread does
	update_thread.start()

	# Start ROS
	rclpy.init()
	pub = TransformPublisher()
	rclpy.spin(pub)


if __name__ == '__main__':
	main()
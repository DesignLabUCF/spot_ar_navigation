# ROS
import rclpy
from rclpy.node import Node
from unity_robotics_demo_msgs.msg import SpotJoint
from unity_robotics_demo_msgs.msg import SpotJoints
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
# Dynamic
_LOGGER = logging.getLogger('ros_joints_plugin')
JOINTS_PUBLISH_FPS = 10
joints = []

def update_joints(robot_state_client):
	global joints

	while True:
		# Make a robot state request
		state = robot_state_client.get_robot_state()
		kinematic_state = state.kinematic_state
		# Get joint values
		joints.clear()
		for joint in kinematic_state.joint_states: # JointState - https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/robot_state.proto#L406
			j = SpotJoint()
			j.name = joint.name
			j.position = joint.position.value
			j.velocity = joint.velocity.value
			j.acceleration = joint.acceleration.value
			j.load = joint.load.value
			joints.append(j)
		# Repeat with a small lapse between
		time.sleep(1.0 / JOINTS_PUBLISH_FPS)


class JointsPublisher(Node):

	def __init__(self):
		super().__init__('joints_publisher')
		self.publisher_ = self.create_publisher(SpotJoints, 'joints', 10)
		#timer_period = 0.25  # seconds
		timer_period = 1.0 / float(JOINTS_PUBLISH_FPS)  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

	def publish_joints(self, joints):
		# Create message
		joints_msg = SpotJoints()
		for i in range(0, len(joints)):
			joints_msg.joints[i] = joints[i]
		# Publish message
		self.get_logger().info(f'Publishing: {joints_msg}')
		self.publisher_.publish(joints_msg)


	'''
	def publish_joint(self, name, position, velocity, acceleration, load):
		# Create message
		joint = SpotJoint()
		joint.name = name
		joint.position = position
		joint.velocity = velocity
		joint.acceleration = acceleration
		joint.load = load
		# Publish message
		self.get_logger().info(f'Publishing: {joint}')
		self.publisher_.publish(joint)	
	'''	
		
	def timer_callback(self):
		global joints
		'''
		for joint in joints:
			self.publish_joint(joint[0], joint[1], joint[2], joint[3], joint[4])
		'''
		self.publish_joints(joints)
		self.i = self.i + 1


def main():
	bosdyn.client.util.setup_logging()

	# Get input args
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Setup robot
	sdk = create_standard_sdk('SENSEable_ROSJoints')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)
	robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

	# Launch joints update thread
	update_thread = threading.Thread(target=update_joints, daemon=True, args=(robot_state_client,)) # Daemon threads will auto-die when main thread does
	update_thread.start()

	# Start ROS
	rclpy.init()
	pub = JointsPublisher()
	rclpy.spin(pub)


if __name__ == '__main__':
	main()
# ROS
import rclpy
from rclpy.node import Node
from unity_robotics_demo_msgs.msg import SpotVelocity
# Python
import sys
from datetime import datetime
import time
import os
import threading
import subprocess
import numpy as np
import math
# Boston Dynamics
import logging
from bosdyn.client.util import setup_logging
import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from bosdyn.api import geometry_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.util import duration_str, format_metric, secs_to_hms
import bosdyn.geometry
from bosdyn.client.math_helpers import Quat, SE3Pose, quat_to_eulerZYX

#from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2


# ROS parameters
ROS_TOPIC = "velocity"
# Movement parameters - Custom
#VELOCITY_BASE_SPEED = 1.0 # m/s
#VELOCITY_BASE_ANGULAR = 0.8 # rad/sec
#VELOCITY_STRAFE_SPEED = 0.6
# Movement parameters - Same as tablet with STAIRS MODE on
VELOCITY_BASE_SPEED = 1.0 
VELOCITY_BASE_ANGULAR = 0.75
VELOCITY_STRAFE_SPEED = 0.35
# Movement parameters - other
VELOCITY_CMD_DURATION = 0.25
COMMAND_INPUT_RATE = 0.1
HEIGHT_MULTIPLIER = 0.25
STICK_THRESHOLD = 0.15 # Percentage of stick pressure applied needed to issue commands
# Dynamic
_LOGGER = logging.getLogger('ros_velocity_plugin')

class AsyncRobotState(AsyncPeriodicQuery):
	"""Grab robot state."""
	def __init__(self, robot_state_client):
		super(AsyncRobotState, self).__init__('robot_state', robot_state_client, _LOGGER,
											  period_sec=0.2)
	def _start_query(self):
		return self._client.get_robot_state_async()

class RobotDriver():
	def __init__(self, robot):
		self._robot = robot
		# Create clients -- do not use the for communication yet.
		self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
		try:
			self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
			self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
		except:
			# Not the estop.
			self._estop_client = None
			self._estop_endpoint = None
		self._power_client = robot.ensure_client(PowerClient.default_service_name)
		self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
		self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
		self._robot_state_task = AsyncRobotState(self._robot_state_client)
		#self._lock = threading.Lock()
		self._estop_keepalive = None
		self._exit_check = None
		# Stuff that is set in start()
		self._robot_id = None
		self._lease_keepalive = None
		self.standing_mode = False
		self.mobility_params = self.set_mobility_params()

	def start(self):
		"""Begin communication with the robot."""
		# Construct our lease keep-alive object, which begins RetainLease calls in a thread.
		self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
											   return_at_exit=True, on_failure_callback=self.lease_retain_error)

		self._robot_id = self._robot.get_id()
		if self._estop_endpoint is not None:
			self._estop_endpoint.force_simple_setup(
			)  # Set this endpoint as the robot's sole estop.

	def shutdown(self):
		"""Release control of robot as gracefully as possible."""
		_LOGGER.info('Shutting down RobotDriver.')
		if self._estop_keepalive:
			# This stops the check-in thread but does not stop the robot.
			self._estop_keepalive.shutdown()
		if self._lease_keepalive:
			try:
				self._lease_keepalive.shutdown()
			except:
				pass # TODO

	@property
	def robot_state(self):
		"""Get latest robot state proto."""
		#return self._robot_state_task.proto
		return self._robot_state_client.get_robot_state()

	def _try_grpc(self, desc, thunk):
		try:
			return thunk()
		except (ResponseError, RpcError, LeaseBaseError) as err:
			_LOGGER.info(f'Failed {desc}: {err}')
			return None

	def _try_grpc_async(self, desc, thunk):

		def on_future_done(fut):
			try:
				fut.result()
			except (ResponseError, RpcError, LeaseBaseError) as err:
				_LOGGER.info(f'Failed {desc}: {err}')
				return None

		future = thunk()
		future.add_done_callback(on_future_done)

	def _quit_program(self):
		self._sit()
		if self._exit_check is not None:
			self._exit_check.request_exit()

	def _toggle_estop(self):
		"""toggle estop on/off. Initial state is ON"""
		if self._estop_client is not None and self._estop_endpoint is not None:
			if not self._estop_keepalive:
				self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
			else:
				self._try_grpc('stopping estop', self._estop_keepalive.stop)
				self._estop_keepalive.shutdown()
				self._estop_keepalive = None

	def _toggle_lease(self):
		"""toggle lease acquisition. Initial state is acquired"""
		if self._lease_client is not None:
			if self._lease_keepalive is None:
				self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
													   return_at_exit=True, on_failure_callback=self.lease_retain_error)
			else:
				self._lease_keepalive.shutdown()
				self._lease_keepalive = None

	def _start_robot_command(self, desc, command_proto, end_time_secs=None):

		def _start_command():
			self._robot_command_client.robot_command(command=command_proto,
													 end_time_secs=end_time_secs)

		self._try_grpc(desc, _start_command)

	def _self_right(self):
		self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

	def _sit(self):
		self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

	def _stand(self):
		self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

	def _stop(self):
		self._start_robot_command('stop', RobotCommandBuilder.stop_command())

	def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
		self._start_robot_command(
			desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=self.mobility_params),
			end_time_secs=time.time() + VELOCITY_CMD_DURATION)

	def _toggle_power(self):
		power_state = self._power_state()
		if power_state is None:
			_LOGGER.info('Could not toggle power because power state is unknown')
			return

		if power_state == robot_state_proto.PowerState.STATE_OFF:
			self._try_grpc_async('powering-on', self._request_power_on)
		else:
			self._try_grpc('powering-off', self._safe_power_off)

	def _request_power_on(self):
		request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
		return self._power_client.power_command_async(request)

	def _safe_power_off(self):
		self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

	def _power_state(self):
		state = self.robot_state
		if not state:
			return None
		return state.power_state.motor_power_state

	def toggle_standing_mode(self):
		self.set_standing_mode(not self.standing_mode)

	def set_standing_mode(self, should_be_standing):
		self.standing_mode = should_be_standing

	def set_stance(self, yaw=0, pitch=0, roll=0, height=0):
		#footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)
		footprint_R_body = bosdyn.geometry.EulerZXY(yaw=yaw, roll=roll, pitch=pitch)
		#cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
		cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body, body_height=height)
		self._robot_command_client.robot_command(cmd)

	def lease_retain_error(self, error):
		_LOGGER.info("Lease error encountered, shutting down container gracefully...")
		self.shutdown()
		shutdown_ros()
		raise Exception("Exiting...")

	def go_to(self, x, y, heading):
		tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
			goal_x=x, goal_y=y,
			goal_heading=heading, frame_name=VISION_FRAME_NAME, params=self.mobility_params,
			body_height=0.0, locomotion_hint=spot_command_pb2.HINT_AUTO)
		end_time = 20.0
		movement_on = True # TODO
		powered_on = True # TODO
		if movement_on and powered_on:
			#Issue the command to the robot
			self._robot_command_client.robot_command(lease=None, command=tag_cmd,
													 end_time_secs=time.time() + end_time)
			# #Feedback to check and wait until the robot is in the desired position or timeout
			#start_time = time.time()
			#current_time = time.time()
			#while (not self.final_state() and current_time - start_time < end_time):
			#    time.sleep(.25)
			#    current_time = time.time()

	def calculate_heading(self, goal_x, goal_y):
		# Calculate two points
		robot_rt_world = get_vision_tform_body(self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
		robot_to_object_ewrt_world = np.array(
			[goal_x - robot_rt_world.x, goal_y - robot_rt_world.y, 0])
		robot_to_object_ewrt_world_norm = robot_to_object_ewrt_world / np.linalg.norm(
			robot_to_object_ewrt_world)
		return self.get_desired_angle(robot_to_object_ewrt_world_norm)		

	def go_to_point_absolute(self, x, y, z):
		self.go_to(x, y, self.calculate_heading(x, y))

	# Adapted from fidicual_follow.py
	def go_to_point_from_april_tag(self, offset_x, offset_y, april_pos_x, april_pos_y, april_rot_x, april_rot_y, april_rot_z, april_rot_w, invert_heading):
		desired_world_pose, angle_desired = self.offset_tag_pose(offset_x, offset_y, april_pos_x, april_pos_y, april_rot_x, april_rot_y, april_rot_z, april_rot_w, invert_heading)
		'''
		#Command the robot to go to the tag in kinematic odometry frame
		mobility_params = self.set_mobility_params()
		tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
			goal_x=desired_world_pose[0], goal_y=desired_world_pose[1],
			goal_heading=angle_desired, frame_name=VISION_FRAME_NAME, params=mobility_params,
			body_height=0.0, locomotion_hint=spot_command_pb2.HINT_AUTO)
		end_time = 20.0

		movement_on = True # TODO
		powered_on = True # TODO
		if movement_on and powered_on:
			#Issue the command to the robot
			self._robot_command_client.robot_command(lease=None, command=tag_cmd,
													 end_time_secs=time.time() + end_time)
			# #Feedback to check and wait until the robot is in the desired position or timeout
			#start_time = time.time()
			#current_time = time.time()
			#while (not self.final_state() and current_time - start_time < end_time):
			#    time.sleep(.25)
			#    current_time = time.time()
		'''
		self.go_to(desired_world_pose[0], desired_world_pose[1], angle_desired)

	# Adapted from fidicual_follow.py
	def offset_tag_pose(self, offset_x, offset_y, april_pos_x, april_pos_y, april_rot_x, april_rot_y, april_rot_z, april_rot_w, invert_heading):
		# Calculate rotation to be applied to the coordinate offset - https://dev.bostondynamics.com/_modules/bosdyn/client/math_helpers
		tag_quat = Quat(april_rot_w, april_rot_x, april_rot_y, april_rot_z)
		applied_rotation = tag_quat.to_yaw() # Can also pull the Yaw rotation from quat_to_eulerZYX(tag_quat)
		if(math.degrees(applied_rotation) < 0): # Boston dynamics interprets greater than 180 degrees as negative
			applied_rotation = math.radians(math.degrees(applied_rotation) + 360.0)
		_LOGGER.info(f"Rotation Angle to be applied: {applied_rotation}")
		# Apply that rotation
		rotated_offset = self.rotate((0, 0), (offset_x, offset_y), applied_rotation) # This has been validated
		_LOGGER.info(f"Original Offset: ({offset_x}, {offset_y})")
		_LOGGER.info(f"Rotated Offset: ({rotated_offset[0]}, {rotated_offset[1]})")
		goto_rt_world = np.array([
			april_pos_x - rotated_offset[0],
			april_pos_y - rotated_offset[1]
		])
		_LOGGER.info(f"Original tag position: ({april_pos_x}, {april_pos_y})")
		_LOGGER.info(f"Calculated goal: ({goto_rt_world[0]}, {goto_rt_world[1]})")
		# Calculate heading from current position to goal to set robots rotation
		'''
		robot_rt_world = get_vision_tform_body(self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
		robot_to_object_ewrt_world = np.array(
			[goto_rt_world[0] - robot_rt_world.x, goto_rt_world[1] - robot_rt_world.y, 0])
		robot_to_object_ewrt_world_norm = robot_to_object_ewrt_world / np.linalg.norm(
			robot_to_object_ewrt_world)
		heading = self.get_desired_angle(robot_to_object_ewrt_world_norm)
		'''
		heading = self.calculate_heading(goto_rt_world[0], goto_rt_world[1])
		_LOGGER.info(f"Heading inverted: {invert_heading}")
		if invert_heading:
			heading = heading + math.radians(180.0)
		_LOGGER.info(f"Heading: {heading}")

		return goto_rt_world, heading
	
	# https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python
	def rotate(self, origin, point, angle):
		ox, oy = origin
		px, py = point

		qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
		qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
		return qx, qy

	# Adapted from fidicual_follow.py
	def get_desired_angle(self, xhat):
		"""Compute heading based on the vector from robot to object."""
		zhat = [0.0, 0.0, 1.0]
		yhat = np.cross(zhat, xhat)
		mat = np.array([xhat, yhat, zhat]).transpose()
		return Quat.from_matrix(mat).to_yaw()

	# Adapted from fidicual_follow.py
	def set_mobility_params(self):
		"""Set robot mobility params to disable obstacle avoidance."""
		'''
		obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=True,
													disable_vision_foot_obstacle_avoidance=True,
													disable_vision_foot_constraint_avoidance=True,
													obstacle_avoidance_padding=.001)
		'''
		body_control = self.set_default_body_control()
		#if self._limit_speed:
		#limit_speed = False # TODO
		#avoid_obstacles = True
		max_x_vel = VELOCITY_BASE_SPEED
		max_y_vel = VELOCITY_STRAFE_SPEED
		max_ang_vel = VELOCITY_BASE_ANGULAR # 1.6 appears to be max
		speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=max_x_vel, y=max_y_vel), angular=max_ang_vel))
		#speed_limit = None
		stairs_mode = 2 # Off = 1, On = 2, Auto = 3 (https://github.com/boston-dynamics/spot-sdk/blob/6a03ae12056a74e65fd9715128e0c40762dfa7ba/protos/bosdyn/api/spot/robot_command.proto#L76)
		mobility_params = spot_command_pb2.MobilityParams(
			vel_limit=speed_limit,
			body_control=body_control,
			locomotion_hint=spot_command_pb2.HINT_AUTO,
			stairs_mode=stairs_mode)
		return mobility_params
		'''
		if limit_speed:
			speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(
				linear=Vec2(x=max_x_vel, y=max_y_vel), angular=max_ang_vel))
			if not avoid_obstacles:
				mobility_params = spot_command_pb2.MobilityParams(
					obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control,
					locomotion_hint=spot_command_pb2.HINT_AUTO)
			else:
				mobility_params = spot_command_pb2.MobilityParams(
					vel_limit=speed_limit, body_control=body_control,
					locomotion_hint=spot_command_pb2.HINT_AUTO)
		elif not avoid_obstacles:
			mobility_params = spot_command_pb2.MobilityParams(
				obstacle_params=obstacles, body_control=body_control,
				locomotion_hint=spot_command_pb2.HINT_AUTO)
		else:
			#When set to none, RobotCommandBuilder populates with good default values
			mobility_params = None
		return mobility_params
		'''

	# Adapted from fidicual_follow.py
	@staticmethod
	def set_default_body_control():
		"""Set default body control params to current body position"""
		footprint_R_body = bosdyn.geometry.EulerZXY()
		position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
		rotation = footprint_R_body.to_quaternion()
		pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
		point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
		traj = trajectory_pb2.SE3Trajectory(points=[point])
		return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

class VelocitySubscriber(Node):

	def __init__(self, robot_driver):
		# ROS
		super().__init__('spot_velocity_controller')
		self.subscription = self.create_subscription(
			SpotVelocity,
			ROS_TOPIC,
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning
		# Spot
		self.robot_driver = robot_driver
		_LOGGER.info("Subscribing to ROS topic: " + ROS_TOPIC)


	def listener_callback(self, msg):
		command_type = msg.command_type
		_LOGGER.info("--------------------------")
		_LOGGER.info(f"Command of type '{command_type}' received. ")
		if command_type == "velocity":
			# Log to console
			_LOGGER.info("v_x:")
			_LOGGER.info(str(msg.v_x))
			_LOGGER.info("v_y:")
			_LOGGER.info(str(msg.v_y))
			_LOGGER.info("v_rot:")
			_LOGGER.info(str(msg.v_rot))
			# Set up input params from ROS message
			v_x = msg.v_x * VELOCITY_BASE_SPEED
			v_y = msg.v_y * VELOCITY_STRAFE_SPEED
			v_rot = msg.v_rot * VELOCITY_BASE_ANGULAR
			# Issue command
			self.robot_driver._velocity_cmd_helper(desc='move', v_x=v_x, v_y=v_y, v_rot=v_rot)
		elif command_type == "absolute_point":
			_LOGGER.info("--------------------------")
			_LOGGER.info("absolute_x:")
			_LOGGER.info(str(msg.absolute_x))
			_LOGGER.info("absolute_y:")
			_LOGGER.info(str(msg.absolute_y))
			_LOGGER.info("absolute_z:")
			_LOGGER.info(str(msg.absolute_z))			
			self.robot_driver.go_to_point_absolute(msg.absolute_x, msg.absolute_y, msg.absolute_z)
		elif command_type == "relative_point":
			_LOGGER.info("--------------------------")       
			 # Log to console
			_LOGGER.info("offset_x:")
			_LOGGER.info(str(msg.offset_x))
			_LOGGER.info("offset_y:")
			_LOGGER.info(str(msg.offset_y))
			# Issue command
			#self.robot_driver.go_to_point(msg.point_x, msg.point_y, msg.point_z)
			self.robot_driver.go_to_point_from_april_tag(msg.offset_x, msg.offset_y, msg.april_pos_x, msg.april_pos_y, msg.april_rot_x, msg.april_rot_y, msg.april_rot_z, msg.april_rot_w, msg.invert_heading)
		else:
			_LOGGER.info("ERROR: Command type not recognized")


def shutdown_ros():
	_LOGGER.info("ROS node shutting down...")
	rclpy.shutdown()

def give_time_for_action(action_name, duration):
	_LOGGER.info(f"Giving time for {action_name} action to complete...")
	for i in range(int(duration), 0, -1):
		_LOGGER.info(str(i) + "...")
		time.sleep(1)

def main():
	# Init
	setup_logging()
	import argparse
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Init robot
	_LOGGER.info("Authenticating Spot...")
	sdk = create_standard_sdk('SENSEable_ROSVelocity')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)
	robot.start_time_sync()

	# Launch driver
	_LOGGER.info("Launching robot driver...")
	robot_driver = RobotDriver(robot)
	try:
		robot_driver.start()
	except (ResponseError, RpcError) as err:
		_LOGGER.error('Failed to initialize robot communication: %s', err)
		return

	# Issue starting commands with pauses in between to allow for completion
	time.sleep(2.0)
	robot_driver._toggle_estop()
	#_LOGGER.info("Giving time for estop action to complete...")
	#time.sleep(4.0)
	give_time_for_action("e-stop", 2)
	robot_driver._toggle_power()
	#_LOGGER.info("Giving time for power-on action to complete...")
	#time.sleep(8.0)
	give_time_for_action("power-on", 6)
	robot_driver._stand()
	#_LOGGER.info("Giving time for stand action to complete...")
	#time.sleep(4.0)
	give_time_for_action("stand", 2)
	   
	# Start ROS
	#rclpy.init(args=None)
	rclpy.init()
	velocity_subscriber = VelocitySubscriber(robot_driver)
	rclpy.spin(velocity_subscriber)


if __name__ == '__main__':
	main()
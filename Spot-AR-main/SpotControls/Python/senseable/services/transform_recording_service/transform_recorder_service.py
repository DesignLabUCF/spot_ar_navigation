# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

import logging

from google.protobuf import json_format

import bosdyn.client.util
from bosdyn.api import data_acquisition_pb2, data_acquisition_plugin_service_pb2_grpc
from bosdyn.client.data_acquisition_plugin_service import (Capability, DataAcquisitionPluginService,
														   DataAcquisitionStoreHelper)
from bosdyn.client.data_acquisition_store import DataAcquisitionStoreClient
from bosdyn.client.directory_registration import (DirectoryRegistrationClient,
												  DirectoryRegistrationKeepAlive)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.server_util import GrpcServiceRunner
from bosdyn.client.util import setup_logging

from threading import Thread
from time import sleep
import csv
import os
#from google.protobuf import timestamp_pb2
import datetime
import pandas as pd
from bosdyn.client.frame_helpers import (get_odom_tform_body, get_vision_tform_body, get_a_tform_b)
from bosdyn.client.world_object import WorldObjectClient


### Service 
DIRECTORY_NAME = 'data-acquisition-transform-recorder'
AUTHORITY = 'data-acquisition-transform-recorder'

CAPABILITY = Capability(name='transform', description='Position and rotation data', channel_name='transform')
#SERVICE_TYPE = 'transform-recorder'
#_LOGGER = logging.getLogger('battery_plugin')
_LOGGER = logging.getLogger('transform_recorder_plugin')

### Data recording
recording_active = False
recording_thread = None
UPDATE_FPS = 20.0
APRIL_TAG_ID = 4
TIMESTAMP_FORMAT = "%m-%d-%Y_%H-%M-%S-%f" # [:-3] at end after formatted to convert to milliseconds
JOINT_NAMES = ["fl.hx", "fl.hy", "fl.kn", "fr.hx", "fr.hy", "fr.kn", "hl.hx", "hl.hy", "hl.kn", "hr.hx", "hr.hy", "hr.kn"]

class ServiceAdapter:
	def __init__(self, sdk_robot):
		self.client = sdk_robot.ensure_client(RobotStateClient.default_service_name)
		self.world_object_client = sdk_robot.ensure_client(WorldObjectClient.default_service_name) # For detecting april tags
		self.most_recent_april_position = None
		self.most_recent_april_rotation = None

	## TODO
	def get_transform_data(self, request: data_acquisition_pb2.AcquirePluginDataRequest,
						 store_helper: DataAcquisitionStoreHelper):
		"""Save the latest data to the data store."""
		data_id = data_acquisition_pb2.DataIdentifier(action_id=request.action_id,
													  channel=CAPABILITY.channel_name)

		state = self.client.get_robot_state(timeout=1)

		# Read in transform data from state
		#timestamp, vision_position = self.get_transform(state)

		# Check if the request has been cancelled.
		store_helper.cancel_check()

		# All the data we need is now collected, so we can tell clients that we have moved on to
		# saving the data.
		store_helper.state.set_status(data_acquisition_pb2.GetStatusResponse.STATUS_SAVING)

		# Populate data acquisition store message.
		if not recording_active:
			message = data_acquisition_pb2.AssociatedMetadata()
			message.reference_id.action_id.CopyFrom(request.action_id)
			message.metadata.data.update({
				'recording_start_pos':
					json_format.MessageToJson(self.get_transform(state)[1]),
			})
			_LOGGER.info('Transform recording starting...')
			self.start_recording()
		else:
			message = data_acquisition_pb2.AssociatedMetadata()
			message.reference_id.action_id.CopyFrom(request.action_id)
			message.metadata.data.update({
				'recording_end_pos':
					json_format.MessageToJson(self.get_transform(state)[1]),
			})
			_LOGGER.info('Transform recording ending...')
			self.end_recording()

		# Store the data and manage store state.
		store_helper.store_metadata(message, data_id)

	def get_transform(self, robot_state):
		# State
		kinematic_state = robot_state.kinematic_state # https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/robot_state.proto
		transform_tree = kinematic_state.transforms_snapshot # https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/robot_state.proto#L380

		# Body position/rotation
		''' # Originally pulled position and rotation values directly, but values returned were incorrect. Seems to function properly when using the functions from frame_helpers.py
		vision_transform = transform_tree.child_to_parent_edge_map["vision"]
		vision_pose = vision_transform.parent_tform_child
		vision_position = vision_pose.position # .x, .y, .z - Vec3 - https://github.com/boston-dynamics/spot-sdk/blob/6a03ae12056a74e65fd9715128e0c40762dfa7ba/protos/bosdyn/api/geometry.proto#L92
		vision_rotation = vision_pose.rotation # .x, .y, .z, and .w - https://github.com/boston-dynamics/spot-sdk/blob/6a03ae12056a74e65fd9715128e0c40762dfa7ba/protos/bosdyn/api/geometry.proto#L92
		'''
		vision_transform = get_vision_tform_body(transform_tree)
		vision_position = vision_transform.position # .x, .y, .z - Vec3
		vision_rotation = vision_transform.rotation # .x, .y, .z, and .w
		vision_velocity = kinematic_state.velocity_of_body_in_vision

		# Joints
		joints = []
		for joint in kinematic_state.joint_states: # JointState - https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/robot_state.proto#L406
			#_LOGGER.info(joint.name)
			joints.append([joint.name, joint.position.value, joint.velocity.value, joint.acceleration.value, joint.load.value])
			pass # TODO

		# Timestamp
		timestamp = kinematic_state.acquisition_timestamp # https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/robot_state.proto#L380
		timestamp = datetime.datetime.fromtimestamp(timestamp.ToMicroseconds() / 1000000.0) # https://googleapis.dev/python/protobuf/latest/google/protobuf/timestamp_pb2.html

		# Relevant April tag readings
		world_objects = self.world_object_client.list_world_objects().world_objects
		for obj in world_objects:
			if(obj.name == ("world_obj_apriltag_" + str(APRIL_TAG_ID))): # Desired april tag is currently visible
				april_transform = obj.transforms_snapshot
				april_key = "filtered_fiducial_" + str(APRIL_TAG_ID)
				april_from_vision = get_a_tform_b(april_transform, "vision", april_key)
				self.most_recent_april_position = april_from_vision.position
				self.most_recent_april_rotation = april_from_vision.rotation

		#_LOGGER.info(kinematic_state.acquisition_timestamp)
		return timestamp, vision_position, vision_rotation, vision_velocity, self.most_recent_april_position, self.most_recent_april_rotation, joints

	def start_recording(self):
		global recording_active
		global recording_thread
		recording_active = True
		recording_thread = Thread(target=self.transform_loop)
		recording_thread.start()
		#recording_thread.join()

	def end_recording(self):
		global recording_active
		recording_active = False # Will also kill thread
		# Saving handled in thread

	def get_april_tag_values(self, april_position, april_rotation):
		if (april_position == None) or (april_rotation == None): # April tag was not yet detected
			return [None, None, None, None, None, None, None] # 3 for position, 4 for quaternion
		else:
			return [april_position.x, april_position.y, april_position.z, april_rotation.x, april_rotation.y, april_rotation.z, april_rotation.w]


	def get_joint_values(self, joints): # joints.append([joint.name, joint.position, joint.velocity, joint.acceleration, joint.load])
		joint_values = []
		for joint in joints:
			joint_values.extend([
				joint[1],
				joint[2],
				joint[3],
				joint[4]
				])
		return joint_values

	def get_joint_columns(self):
		column_names = []
		for joint_name in JOINT_NAMES:
			column_names.extend([
				joint_name + "_pos",
				joint_name + "_vel",
				joint_name + "_acc",
				joint_name + "_load"
				])
		return column_names

	def transform_loop(self):
		global recording_active
		# Init
		transform_data = [] # Use pandas for final output, but list for appending every tick. Adding row to pandas list is inefficient
		# Continously loop and record the transform
		while recording_active:
			state = self.client.get_robot_state(timeout=1)
			# Pull info
			timestamp, vision_position, vision_rotation, vision_velocity, april_position, april_rotation, joints = self.get_transform(state)
			# Log
			_LOGGER.info(timestamp)
			transform_data.append([
				timestamp.strftime(TIMESTAMP_FORMAT)[:-3],
				vision_position.x,
				vision_position.y,
				vision_position.z,
				vision_rotation.x,
				vision_rotation.y,
				vision_rotation.z,
				vision_rotation.w,
				vision_velocity.linear.x, # m/s
				vision_velocity.linear.y, # m/s
				vision_velocity.linear.z, # m/s
				vision_velocity.angular.x, # (rad/s)
				vision_velocity.angular.y, # (rad/s)
				vision_velocity.angular.z, # (rad/s)
				*self.get_april_tag_values(april_position, april_rotation), # Unpack validated april tag readings
				*self.get_joint_values(joints) # Unpack joint values
				])
			sleep(1.0 / UPDATE_FPS)
		# Save to output .csv
		directory_name = "/data/transform_recordings"
		if not os.path.exists(directory_name):
			os.makedirs(directory_name)
		transform_filename = os.path.join(directory_name, timestamp.strftime(TIMESTAMP_FORMAT)[:-3]) + ".csv" # use last timestamp as filename
		_LOGGER.info("Writing to file:" + transform_filename)
		'''
		with open(transform_filename, "w") as csvfile:
			writer = csv.writer(csvfile)
			for t in transforms:
				trans = t[0]
				timestamp = t[1].strftime(TIMESTAMP_FORMAT) # UTC
				writer.writerow((trans, timestamp))
		'''
		df = pd.DataFrame(transform_data, columns=
			["timestamp_utc",
			"pos_x_vision_body",
			"pos_y_vision_body",
			"pos_z_vision_body",
			"quat_x_vision_body",
			"quat_y_vision_body",
			"quat_z_vision_body",
			"quat_w_vision_body",
			"vel_linear_x_body",
			"vel_linear_y_body",
			"vel_linear_z_body",
			"vel_angular_x_body",
			"vel_angular_y_body",
			"vel_angular_z_body",
			"pos_x_vision_april",
			"pos_y_vision_april",
			"pos_z_vision_april",
			"quat_x_vision_april",
			"quat_y_vision_april",
			"quat_z_vision_april",
			"quat_w_vision_april",
			*self.get_joint_columns() # Unpack joint column names (Each joint should have a column for every value)
			])
		df.to_csv(transform_filename, sep=",")
		_LOGGER.info("Data saved!")


def make_servicer(sdk_robot):
	"""Create the data acquisition servicer for the battery data."""
	adapter = ServiceAdapter(sdk_robot)
	return DataAcquisitionPluginService(sdk_robot, [CAPABILITY], adapter.get_transform_data,
										logger=_LOGGER)


def run_service(sdk_robot, port):
	"""Create and run the battery plugin service."""
	# Proto service specific function used to attach a servicer to a server.
	add_servicer_to_server_fn = data_acquisition_plugin_service_pb2_grpc.add_DataAcquisitionPluginServiceServicer_to_server

	# Instance of the servicer to be run.
	return GrpcServiceRunner(make_servicer(sdk_robot), add_servicer_to_server_fn, port,
							 logger=_LOGGER)


if __name__ == '__main__':
	# Define all arguments used by this service.
	import argparse
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	bosdyn.client.util.add_service_endpoint_arguments(parser)
	options = parser.parse_args()

	# Setup logging to use either INFO level or DEBUG level.
	setup_logging(options.verbose)

	# Create and authenticate a bosdyn robot object.
	sdk = bosdyn.client.create_standard_sdk('TransformRecorderPlugin')

	'''
	import bosdyn.mission.remote_client
	sdk.register_service_client(bosdyn.mission.remote_client.RemoteClient,
								service_name=DIRECTORY_NAME)
	'''

	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)

	# Create a service runner to start and maintain the service on background thread.
	service_runner = run_service(robot, options.port)

	# Use a keep alive to register the service with the robot directory.
	dir_reg_client = robot.ensure_client(DirectoryRegistrationClient.default_service_name)
	keep_alive = DirectoryRegistrationKeepAlive(dir_reg_client, logger=_LOGGER)
	keep_alive.start(DIRECTORY_NAME, DataAcquisitionPluginService.service_type, AUTHORITY,
	                 options.host_ip, service_runner.port)
	#keep_alive.start(DIRECTORY_NAME, SERVICE_TYPE, AUTHORITY,
	#				 options.host_ip, service_runner.port)

	# Attach the keep alive to the service runner and run until a SIGINT is received.
	with keep_alive:
		service_runner.run_until_interrupt()

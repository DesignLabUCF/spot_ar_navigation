# ROS2
import rclpy
from rclpy.node import Node
# ROS2 messages/services
from unity_robotics_demo_msgs.srv import TransformRecordingServiceInfo
# General
import time
# Boston Dynamics
import argparse
import bosdyn.client
import bosdyn.client.util
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.data_acquisition import DataAcquisitionClient
from bosdyn.api import data_acquisition_pb2
from bosdyn.client.data_acquisition_helpers import acquire_and_process_request, cancel_acquisition_request, download_data_REST, issue_acquire_data_request, make_time_query_params


class RecordingHelperService(Node):

	def __init__(self, robot):
		super().__init__('get_spot_image_service')
		self.srv = self.create_service(TransformRecordingServiceInfo, 'recording_helper', self.request_callback)
		self.get_logger().info("Waiting for request...")
		self.robot = robot
		self.i = 0

	def request_callback(self, request, response):
		## Get camera
		self.get_logger().info(f'Incoming request for should_start_recording: {request.should_start_recording}')
		## TODO query the service and get info
		currently_recording = False # TODO get
		if (not currently_recording) and request.should_start_recording: # No recording active and we want to record
			succesfully_called = self.trigger_recording_service()
		elif currently_recording and request.should_start_recording: # Recording should be stopped
			pass # TODO
		else: # Just checking, so nothing needs to be done
			pass

		# TODO
		response.able_to_record = False
		response.is_recording = False
		response.succesfully_started_recording = False

		## Broadcast response
		#response.image = ros_image
		self.i = self.i + 1
		return response

	def trigger_recording_service(self):
		self.get_logger().info("Calling transform_recorder_service...")
		try:
			data_acq_client = self.robot.ensure_client(DataAcquisitionClient.default_service_name)
			now = self.robot.time_sync.robot_timestamp_from_local_secs(time.time())
			group_name = f'DataAcquisitionExample_{now.ToJsonString().replace(":", "-")}' # Just going to leave this as is
			acquisition_requests = data_acquisition_pb2.AcquisitionRequestList()
			acquisition_requests.data_captures.extend([data_acquisition_pb2.DataCapture(name='transform')]) # transform_record_service capability name
			# Issue request to start/stop recording
			request_id, action_id = issue_acquire_data_request(data_acq_client, acquisition_requests, group_name, 'AcquisitionsToCancel')
			self.get_logger().info("transform_recorder_service succesfully called.")
			return True
		except Exception as e:
			self.get_logger().info("Unable to call transform_recorder_service. Ensure robot IP on service is correct")
			return False


def main():
	bosdyn.client.util.setup_logging()

	# Get input args
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Setup robot
	sdk = create_standard_sdk('SENSEable_ROSRecordingHelper')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)

	# Spin up ROS2 node
	rclpy.init()
	service = RecordingHelperService(robot)
	rclpy.spin(service)
	rclpy.shutdown()


if __name__ == '__main__':
	main()
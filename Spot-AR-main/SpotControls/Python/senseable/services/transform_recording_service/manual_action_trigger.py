### python manual_action_trigger.py ROBOT_IP
### python manual_action_trigger.py 192.168.80.3

import sys
import argparse
import time

import bosdyn.client.util
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.api import data_acquisition_pb2
from bosdyn.client.data_acquisition import DataAcquisitionClient
from bosdyn.client.data_acquisition_helpers import issue_acquire_data_request

if __name__ == '__main__':

	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Init robot
	sdk = create_standard_sdk('SENSEable_ManualRecordingTrigger')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)
	robot.time_sync.wait_for_sync()

	# Launch service call
	print("Calling transform_recorder_service...")
	data_acq_client = robot.ensure_client(DataAcquisitionClient.default_service_name)
	now = robot.time_sync.robot_timestamp_from_local_secs(time.time())
	group_name = f'DataAcquisitionExample_{now.ToJsonString().replace(":", "-")}' # Just going to leave this as is
	acquisition_requests = data_acquisition_pb2.AcquisitionRequestList()
	acquisition_requests.data_captures.extend([data_acquisition_pb2.DataCapture(name='transform')]) # transform_record_service capability name
	request_id, action_id = issue_acquire_data_request(data_acq_client, acquisition_requests, group_name, 'AcquisitionsToCancel') # Issue request to start/stop recording
	if not (request_id == None):
		print("transform_recorder_service succesfully called.")
	else:
		print("Call failed!")
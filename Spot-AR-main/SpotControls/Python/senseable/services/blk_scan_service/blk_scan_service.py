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
                                                           DataAcquisitionStoreHelper, RequestCancelledError)
from bosdyn.client.data_acquisition_store import DataAcquisitionStoreClient
from bosdyn.client.directory_registration import (DirectoryRegistrationClient,
                                                  DirectoryRegistrationKeepAlive)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.server_util import GrpcServiceRunner
from bosdyn.client.util import setup_logging

import os
import datetime
from time import sleep
from google.protobuf import timestamp_pb2

'''
    Unsure how to extend the deadline past thirty seconds in the code here.
    For now, when creating the action on the tablet, the timeout deadline can be manually set there.
'''


### Service 
DIRECTORY_NAME = 'data-acquisition-blk-scan'
AUTHORITY = 'data-acquisition-scan'
CAPABILITY = Capability(name='scan', description='trigger the blk scanner', channel_name='scan')
_LOGGER = logging.getLogger('blk_scan_plugin')

TIMESTAMP_FORMAT = "%m-%d-%Y_%H-%M-%S-%f" # [:-3] at end after formatted to convert to milliseconds
DESIRED_TIMEOUT_SECONDS = 45.0

class ServiceAdapter:
    def __init__(self, sdk_robot):
        self.client = sdk_robot.ensure_client(RobotStateClient.default_service_name)
        self.response_var = None

    ## TODO
    def get_data(self, request: data_acquisition_pb2.AcquirePluginDataRequest, store_helper: DataAcquisitionStoreHelper):
        """Save the latest data to the data store."""
        data_id = data_acquisition_pb2.DataIdentifier(action_id=request.action_id,
                                                      channel=CAPABILITY.channel_name)

        state = self.client.get_robot_state(timeout=1)

        _LOGGER.info("data_collect_fn")

        # Check if the request has been cancelled.
        store_helper.cancel_check()

        # Ensure timeout does not occur (https://dev.bostondynamics.com/docs/concepts/writing_services_for_data_acquisition)
        pass # TODO

        # All the data we need is now collected, so we can tell clients that we have moved on to
        # saving the data.
        store_helper.state.set_status(data_acquisition_pb2.GetStatusResponse.STATUS_SAVING)

        # Run Scan
        self.run_scan(store_helper)

        # Get scan timestamp
        kinematic_state = state.kinematic_state
        timestamp = kinematic_state.acquisition_timestamp
        timestamp = datetime.datetime.fromtimestamp(timestamp.ToMicroseconds() / 1000000.0)

        # Populate data acquisition store message.
        message = data_acquisition_pb2.AssociatedMetadata()
        message.reference_id.action_id.CopyFrom(request.action_id)
        message.metadata.data.update({
            'blk_scan_timestamp':
                timestamp.strftime(TIMESTAMP_FORMAT)[:-3],
        })

        # To save data as raw bytes, the store_data function is used. (https://dev.bostondynamics.com/docs/concepts/writing_services_for_data_acquisition)
        #store_helper.store_data(BYTES_DATA, data_id)

        # Store the data and manage store state.
        store_helper.store_metadata(message, data_id)


    def acquire_response(self, request:data_acquisition_pb2.AcquirePluginDataRequest, response:data_acquisition_pb2.AcquirePluginDataResponse):
        self.response_var = response
        return True


    def run_scan(self, store_helper):
        self.extend_timeout_deadline()
        try:
            # Data collection and storage here
            for i in range(0, 60):
                store_helper.cancel_check()
                #_LOGGER.info()
                sleep(1) # TODO
        except RequestCancelledError:
            # Perform cleanup here
            pass
            #_LOGGER.info("ERROR: Scan timeout.")
            raise
        
        return 0


    # To delay timeout request (https://dev.bostondynamics.com/docs/concepts/writing_services_for_data_acquisition)
    # Does not seem to work. Cannot figure out why and their documentation on the topic is almsot non-existant. No examples available
    def extend_timeout_deadline(self):
        original_deadline_ms = self.response_var.header.request_received_timestamp.ToMicroseconds()
        original_deadline = self.response_var.header.request_received_timestamp
        extended_deadline_dt = datetime.datetime.fromtimestamp(original_deadline.ToMicroseconds() / 1000000.0) + datetime.timedelta(seconds=DESIRED_TIMEOUT_SECONDS)
        extended_deadline_total_seconds = int(extended_deadline_dt.timestamp())
        extended_deadline_proto = timestamp_pb2.Timestamp(seconds=extended_deadline_total_seconds) # https://stackoverflow.com/questions/49161633/how-do-i-create-a-protobuf3-timestamp-in-python
        # Debug, print deadline info
        _LOGGER.info(str("Original deadline:" + datetime.datetime.fromtimestamp(original_deadline.ToMicroseconds() / 1000000.0).strftime(TIMESTAMP_FORMAT)[:-3]))
        _LOGGER.info(str("Extended deadline_dt:" + extended_deadline_dt.strftime(TIMESTAMP_FORMAT)[:-3]))
        _LOGGER.info(str("Extended deadline_proto:" + datetime.datetime.fromtimestamp(extended_deadline_proto.ToMicroseconds() / 1000000.0).strftime(TIMESTAMP_FORMAT)[:-3]))
        _LOGGER.info(str("Extended deadline_dt_seconds:" + str(extended_deadline_total_seconds)))

        # Update deadline on response
        self.response_var.timeout_deadline.CopyFrom(extended_deadline_proto) # https://stackoverflow.com/questions/18376190/attributeerror-assignment-not-allowed-to-composite-field-task-in-protocol-mes

        _LOGGER.info("acquire_response_fn")
        _LOGGER.info("Header")
        _LOGGER.info(self.response_var.header)
        _LOGGER.info("Header - Recieved TS")
        _LOGGER.info(self.response_var.header.request_received_timestamp)
        _LOGGER.info("Header - Response TS")
        _LOGGER.info(self.response_var.header.response_timestamp)


def make_servicer(sdk_robot):
    """Create the data acquisition servicer for the battery data."""
    adapter = ServiceAdapter(sdk_robot)
    return DataAcquisitionPluginService(sdk_robot, [CAPABILITY], data_collect_fn=adapter.get_data, acquire_response_fn=adapter.acquire_response,
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
    sdk = bosdyn.client.create_standard_sdk('BLKScanPlugin')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    # Create a service runner to start and maintain the service on background thread.
    service_runner = run_service(robot, options.port)

    # Use a keep alive to register the service with the robot directory.
    dir_reg_client = robot.ensure_client(DirectoryRegistrationClient.default_service_name)
    keep_alive = DirectoryRegistrationKeepAlive(dir_reg_client, logger=_LOGGER)
    keep_alive.start(DIRECTORY_NAME, DataAcquisitionPluginService.service_type, AUTHORITY,
                     options.host_ip, service_runner.port)

    # Attach the keep alive to the service runner and run until a SIGINT is received.
    with keep_alive:
        service_runner.run_until_interrupt()

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
import k4a
from PIL import Image
import kinect_read


### Service 
DIRECTORY_NAME = 'data-acquisition-kinect-scan'
AUTHORITY = 'data-acquisition-scan'
CAPABILITY = Capability(name='scan', description='trigger the kinect scanner', channel_name='scan')
_LOGGER = logging.getLogger('kinect_scan_plugin')

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
            'kinect_scan_timestamp':
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
        '''
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
        '''

        try:
            # Read image
            image_data, log_msg = kinect_read.get_color_image()
            if image_data == None:
                _LOGGER.info(log_msg)
                return 0
            kinect_read.save_img(image_data)
        except:
            raise

        return 0


    '''
    # Modified from the_basics.py - https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/src/python/k4a/examples/the_basics.py
    def get_color_image(self):
        device = k4a.Device.open()
        if device is None:
            _LOGGER.info("ERROR: Kinect open failure. Ensure device is connected. Try unplugging and replugging back in.")
            return None

        device_config = k4a.DeviceConfiguration(color_format=k4a.EImageFormat.COLOR_BGRA32, color_resolution=k4a.EColorResolution.RES_1080P, depth_mode=k4a.EDepthMode.OFF, camera_fps=k4a.EFramesPerSecond.FPS_15, synchronized_images_only=False, depth_delay_off_color_usec=0, wired_sync_mode=k4a.EWiredSyncMode.STANDALONE, subordinate_delay_off_master_usec=0, disable_streaming_indicator=False)

        status = device.start_cameras(device_config)
        if status != k4a.EStatus.SUCCEEDED:
            _LOGGER.info("ERROR: Kinect camera start failure.")
            return None

        wait_status = device.start_imu()
        if wait_status != k4a.EWaitStatus.SUCCEEDED:
            _LOGGER.info("ERROR: Kinect IMU start failure.")
            return None

        #imu_sample = device.get_imu_sample(-1)
        capture = device.get_capture(-1)

        color_image = capture.color
        color_image_data = color_image.data

        device.stop_cameras()
        device.stop_imu()

        return color_image_data

    def save_img(self, image_data):
        # Convert to PIL format for easy save
        img = Image.fromarray(image_data)
        # Set up saving params
        dir_path = "/data/kinect"
        filename = datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S-%f")[:-3] + ".png"
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        # Save
        img.save(os.path.join(dir_path, filename))
    '''


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

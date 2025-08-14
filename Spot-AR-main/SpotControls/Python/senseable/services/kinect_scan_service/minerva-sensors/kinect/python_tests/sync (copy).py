'''
k4a_sync.py

A simple program that captures synced depth image from all connected Azure Kinects.
One Azure Kinect acts as the master that drives the capture of all the other Azure Kinects.

Copyright (c) Microsoft Corporation. All rights reserved.
Licensed under the MIT License.
Kinect For Azure SDK.

https://raw.githubusercontent.com/JonathanESantos/Azure-Kinect-Sensor-SDK/user/jonsanto/AKDK_Sync_Python/src/python/k4a/examples/k4a_sync.py
'''

import threading
import time

# This will import all the public symbols into the k4a namespace.
import k4a


def k4a_capture_thread(cv:threading.Condition, device:k4a.Device=None, captures_list:list=None, index:int=-1):
    with cv:
        cv.wait()

    if (device is not None and captures_list is not None):
        #print('Getting a capture from device ' + str(index) + ' with serial ' + device.serial_number[2:-1] + '.\n')
        captures_list[index] = device.get_capture(-1)


def k4a_sync_main():
    
    num_devices = k4a.Device.get_device_count()
    devices_list = [None] * num_devices
    captures_list = [None] * num_devices

    # Open all connected devices.
    # IMPORTANT: The aux cable should be plugged into the "SYNC OUT" of the master 
    # and into the "SYNC IN" of all other devices. Daisy-chaining is allowed.
    for d in range(num_devices):
        devices_list[d] = k4a.Device.open(d)
        if devices_list[d] is None:
            raise Exception('Failed to open device ' + str(d) + '.')

    # Find the master device and put it at the end of the devices list (for convenience).
    # This is not fool-proof, but the master device is taken to be the one that has
    # the SYNC OUT connected and the SYNC IN not connected.
    for device in devices_list:
        if (device.sync_out_connected is True and device.sync_in_connected is False):
            devices_list.remove(device)
            devices_list.append(device)

    # Print devices info. The last one should be the master (sync out is true, sync in is false).
    print('')
    for index, device in enumerate(devices_list):
        print('Device ' + str(index) + ':')
        print(device.serial_number[2:-1]) # A bug stores the b'' in the serial number, so remove the b and apostrophes.
        print(str(device.hardware_version))
        print('Sync out: ' + str(device.sync_out_connected))
        print('Sync in: ' + str(device.sync_in_connected))
        print('')

    # Start all cameras. Configure the last device as the master.
    # Note that the color camera MUST be started on the master even if the color capture is not used.
    for index, device in enumerate(devices_list):

        # Set color exposure time to manual. Recommended for synchronicity, otherwise
        # differences in auto exposure between devices may cause drift to bring the devices out of sync.
        device.set_color_control(k4a.EColorControlCommand.EXPOSURE_TIME_ABSOLUTE,
            k4a.EColorControlMode.MANUAL,
            30)

        # The start_cameras() function requires a device configuration which
        # specifies the modes in which to put the color and depth cameras.
        # See DeviceConfiguration, EImageFormat, EColorResolution, EDepthMode,
        # EFramesPerSecond, and EWiredSyncMode.

        sync_mode = k4a.EWiredSyncMode.SUBORDINATE
        delay_off_master_usec = 160 # Delay off master to avoid IR interference.
        if (device.sync_out_connected is True and device.sync_in_connected is False):
            sync_mode = k4a.EWiredSyncMode.MASTER
            delay_off_master_usec = 0 # The master must have 0 as subordinate delay.

        device_config = k4a.DeviceConfiguration(
            color_format=k4a.EImageFormat.COLOR_BGRA32,
            color_resolution=k4a.EColorResolution.RES_1080P,
            depth_mode=k4a.EDepthMode.NFOV_2X2BINNED,
            camera_fps=k4a.EFramesPerSecond.FPS_15,
            synchronized_images_only=True,
            depth_delay_off_color_usec=0,
            wired_sync_mode=sync_mode,
            subordinate_delay_off_master_usec=delay_off_master_usec,
            disable_streaming_indicator=False)

        status = device.start_cameras(device_config)
        if status != k4a.EStatus.SUCCEEDED:
            raise Exception('Failed to start device ' + str(index) + '.')

    # Get a capture from each device.
    # Note: For simplicity, this example is non-threaded. In production code, consider a 
    # producer-consumer design where each device are producers of captures and a consumer 
    # makes use of the synchronized captures. In such a design, error handling needs to be
    # built into the design for cases where a synchronized capture cannot be obtained from
    # at one or more devices. 
    # 
    # In addition, Python's Global Interpreter Lock may mean that
    # using the threading package may not be fast enough depending on the application to 
    # capture synchronized frames from each device. For multicore devices, using the 
    # multiprocessing package may be a better approach, or use the C API for collecting
    # synchronized frames which does not suffer the overhead of Python's GIL.
    cv = threading.Condition()
    for index, device in enumerate(devices_list):
        thread_x = threading.Thread(target=k4a_capture_thread, args=(cv, device, captures_list, index))
        thread_x.start()

    time.sleep(0.5)
    with cv:
        cv.notify_all()
    
    # Wait for each device to get a single capture. In production code, this is not robust as
    # there is no guarantee that sleeping for some time will yield a capture. A more robust
    # error handling mechanism should be used.
    time.sleep(1.0) 

    # Print depth capture info.
    # Note: There is no guarantee that multi-device captures are synchronized from independently
    # calling get_capture(). Checking the system_timestamp_nsec field in the capture object to check
    # whether the captures are synchronized will likely yield a time difference that is more than
    # expected. There is an open ticket (https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1665)
    # where cameras using external sync seem to not be synced well enough. Furthermore, continuously
    # grabbing captures may yield a system timestamp that is not monotonic despite the documentation 
    # specifying that the system clock is monotonically increasing.
    for index, device in enumerate(devices_list):
        print('\nPrint capture info from device ' + str(index) + ' with serial ' + device.serial_number[2:-1] + '.')
        print(captures_list[index].depth)

    # Stop all connected devices and close them.
    for device in devices_list:
        device.stop_cameras()
        device.close()

if __name__ == '__main__':
    k4a_sync_main()

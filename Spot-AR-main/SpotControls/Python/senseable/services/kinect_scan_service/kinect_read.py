import os
from datetime import datetime
import k4a
from PIL import Image


# https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/src/python/k4a/README.md


'''
# Modified from the_basics.py - https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/src/python/k4a/examples/the_basics.py
def read_kinect():
    # Open a device using the static function Device.open().
    device = k4a.Device.open()
    if device is None:
        exit(-1)

    # Print the device serial number, hardware_version, and
    # color control capabilities.
    print(device.serial_number)
    print(device.hardware_version)
    print(device.color_ctrl_cap)

    # In order to start capturing frames, need to start the cameras.
    # The start_cameras() function requires a device configuration which
    # specifies the modes in which to put the color and depth cameras.
    # See DeviceConfiguration, EImageFormat, EColorResolution, EDepthMode,
    # EFramesPerSecond, and EWiredSyncMode.
    ## Default:
    device_config = k4a.DeviceConfiguration(
        color_format=k4a.EImageFormat.COLOR_BGRA32,
        color_resolution=k4a.EColorResolution.RES_1080P,
        depth_mode=k4a.EDepthMode.WFOV_2X2BINNED,
        camera_fps=k4a.EFramesPerSecond.FPS_15,
        synchronized_images_only=True,
        depth_delay_off_color_usec=0,
        wired_sync_mode=k4a.EWiredSyncMode.STANDALONE,
        subordinate_delay_off_master_usec=0,
        disable_streaming_indicator=False)
    ## Depth mode on:
    #device_config = k4a.DeviceConfiguration(color_format=k4a.EImageFormat.COLOR_BGRA32, color_resolution=k4a.EColorResolution.RES_1080P, depth_mode=k4a.EDepthMode.WFOV_2X2BINNED, camera_fps=k4a.EFramesPerSecond.FPS_15, synchronized_images_only=True, depth_delay_off_color_usec=0, wired_sync_mode=k4a.EWiredSyncMode.STANDALONE, subordinate_delay_off_master_usec=0, disable_streaming_indicator=False)
    ## Depth mode off:
    device_config = k4a.DeviceConfiguration(color_format=k4a.EImageFormat.COLOR_BGRA32, color_resolution=k4a.EColorResolution.RES_1080P, depth_mode=k4a.EDepthMode.OFF, camera_fps=k4a.EFramesPerSecond.FPS_15, synchronized_images_only=False, depth_delay_off_color_usec=0, wired_sync_mode=k4a.EWiredSyncMode.STANDALONE, subordinate_delay_off_master_usec=0, disable_streaming_indicator=False)

    status = device.start_cameras(device_config)
    if status != k4a.EStatus.SUCCEEDED:
        exit(-1)

    # The IMUs can be started but only after starting the cameras.
    # Note that it returns an EWaitStatus rather than a EStatus.
    wait_status = device.start_imu()
    if wait_status != k4a.EWaitStatus.SUCCEEDED:
        exit(-1)

    imu_sample = device.get_imu_sample(-1)
    print(imu_sample)

    # Get a capture.
    # The -1 tells the device to wait forever until a capture is available.
    capture = device.get_capture(-1)

    # Print the color, depth, and IR image details and the temperature.
    #print(capture.color)
    #print(capture.depth)
    #print(capture.ir)
    #print(capture.temperature)

    # The capture object has fields for the color, depth, and ir images.
    # These are container classes; the actual image data is stored as a
    # numpy ndarray object in a field called data. Users can query these
    # fields directly.
    color_image = capture.color
    color_image_data = color_image.data

    # Get the image width and height in pixels, and stride in bytes.
    #width_pixels = color_image.width_pixels
    #height_pixels = color_image.height_pixels
    #stride_bytes = color_image.stride_bytes

    # There is no need to stop the cameras since the deleter will stop
    # the cameras, but it's still prudent to do it explicitly.
    device.stop_cameras()
    device.stop_imu()

    return color_image_data # TODO add more
'''

def get_color_image():
    device = k4a.Device.open()
    if device is None:
        return None, "ERROR: Kinect open failure. Ensure device is connected. Try unplugging and replugging back in."

    ## Depth mode off:
    device_config = k4a.DeviceConfiguration(color_format=k4a.EImageFormat.COLOR_BGRA32, color_resolution=k4a.EColorResolution.RES_1080P, depth_mode=k4a.EDepthMode.OFF, camera_fps=k4a.EFramesPerSecond.FPS_15, synchronized_images_only=False, depth_delay_off_color_usec=0, wired_sync_mode=k4a.EWiredSyncMode.STANDALONE, subordinate_delay_off_master_usec=0, disable_streaming_indicator=False)
    ## Depth mode on:
    #device_config = k4a.DeviceConfiguration(color_format=k4a.EImageFormat.COLOR_BGRA32, color_resolution=k4a.EColorResolution.RES_1080P, depth_mode=k4a.EDepthMode.WFOV_2X2BINNED, camera_fps=k4a.EFramesPerSecond.FPS_15, synchronized_images_only=True, depth_delay_off_color_usec=0, wired_sync_mode=k4a.EWiredSyncMode.STANDALONE, subordinate_delay_off_master_usec=0, disable_streaming_indicator=False)

    status = device.start_cameras(device_config)
    if status != k4a.EStatus.SUCCEEDED:
        return None, "ERROR: Kinect camera start failure."

    wait_status = device.start_imu()
    if wait_status != k4a.EWaitStatus.SUCCEEDED:
        return None, "ERROR: Kinect IMU start failure."

    #imu_sample = device.get_imu_sample(-1)
    capture = device.get_capture(-1)

    color_image = capture.color
    color_image_data = capture.color.data
    #color_image_data = swap_red_and_blue_in_img(color_image_data)

    # Convert to PIL format for easy rgb channel manipulation and saving
    pil_img = Image.fromarray(color_image_data)
    r, g, b, a = pil_img.split()
    pil_img = Image.merge('RGB', (b, g, r))

    device.stop_cameras()
    device.stop_imu()

    return pil_img, ""

def save_img(pil_img):
    # Set up saving params
    dir_path = "/data/kinect"
    filename = datetime.now().strftime("%m-%d-%Y_%H-%M-%S-%f")[:-3] + ".png"
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)
    # Save
    pil_img.save(os.path.join(dir_path, filename))

# Far too slow, moving to Pillow instead for speed
'''
def swap_red_and_blue_in_img(image_data):
    for i in range(0, len(image_data)):
        row = image_data[i]
        for j in row:
            #pixel = row[j]
            temp = image_data[i][j][0]
            image_data[i][j][0] = image_data[i][j][2]
            image_data[i][j][2] = temp
    return image_data
'''

def get_depth_image():
    device = k4a.Device.open()
    if device is None:
        return None, "ERROR: Kinect open failure. Ensure device is connected. Try unplugging and replugging back in."

    ## Depth mode on:
    device_config = k4a.DeviceConfiguration(color_format=k4a.EImageFormat.COLOR_BGRA32, color_resolution=k4a.EColorResolution.RES_1080P, depth_mode=k4a.EDepthMode.OFF, camera_fps=k4a.EFramesPerSecond.FPS_15, synchronized_images_only=False, depth_delay_off_color_usec=0, wired_sync_mode=k4a.EWiredSyncMode.STANDALONE, subordinate_delay_off_master_usec=0, disable_streaming_indicator=False)

    status = device.start_cameras(device_config)
    if status != k4a.EStatus.SUCCEEDED:
        return None, "ERROR: Kinect camera start failure."

    wait_status = device.start_imu()
    if wait_status != k4a.EWaitStatus.SUCCEEDED:
        return None, "ERROR: Kinect IMU start failure."    

    #imu_sample = device.get_imu_sample(-1)
    capture = device.get_capture(-1)

    color_image = capture.color
    color_image_data = capture.color.data

    pil_img = Image.fromarray(color_image_data)

    device.stop_cameras()
    device.stop_imu()

    return pil_img, ""
    

if __name__ == '__main__':
	# Read image
	#img_data = read_kinect()
    img_data = get_color_image()

    '''
	# Convert to desired format
	img = Image.fromarray(img_data)

	# Set up saving params
	dir_path = "/data/kinect"
	filename = datetime.now().strftime("%m-%d-%Y_%H-%M-%S-%f")[:-3] + ".png"
	if not os.path.exists(dir_path):
		os.makedirs(dir_path)

	# Save
	img.save(os.path.join(dir_path, filename))
    '''
    save_img(img_data)
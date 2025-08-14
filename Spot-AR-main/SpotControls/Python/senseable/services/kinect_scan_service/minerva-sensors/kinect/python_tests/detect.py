# This will import all the public symbols into the k4a namespace.
import k4a

import threading
import time
import numpy as np
import cv2

def k4a_sync_main():
    num_devices = k4a.Device.get_device_count()
    print("   Number of Devices detected: ", num_devices)




if __name__ == '__main__':
    print("Kincet Capture Program \n ----------------------")
    k4a_sync_main()

import sys
import time
import socket
from datetime import datetime
import os
from cv2 import VideoCapture, imwrite


# Webcam Help:
# https://www.geeksforgeeks.org/how-to-capture-a-image-from-webcam-in-python/

### Default Server params
host_address = ""

#port_id = 9999
port_id = 21004

buffer_size = 1024 # Must be power of 2

### Spot params
robot_ip = "192.168.200.45" # Defauls

path = os.path.join("data", "webcam")

camera_symlink = "/dev/test-webcam1" # Linux udev SYMLINK


def main(argv):
	assert len(argv) > 1, "Two arguments must be passed in. One for Spot's IP address, and one for port to host."
	robot_ip = str(argv[0])
	port_id = int(argv[1])

	camera = VideoCapture(camera_symlink)

	### Launch server
	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server.bind((host_address, port_id))
	print("Server launching...")
	print("Attempting to host at: " + host_address + ":" + str(port_id))
	server.listen()
	print("Server launched!")
	print("Spot IP:", robot_ip)

	### Loop n' listen
	while True:
		client, address = server.accept()
		print("Connection recieved from:", address)
		client.send("Connection made.\n".encode())
		
		print("Writing to file:", path)
		result = False
		try:
			if not os.path.exists(path):
				os.makedirs(path)
			file_name = datetime.now().strftime("%d_%m_%Y-%H_%M_%S") + ".jpg"
			file_name = os.path.join(path, file_name)
			result, image = camera.read()
			if result:
				imwrite(file_name, image)
				print("File write success!")
			else:
				print("Error: Webcam inaccessible.")
		except:
			print("File write fail.")
		client.send(("Image save status: " + str(result) + ".\n").encode())

		client.send("Disconnecting...\n".encode())
		time.sleep(3.0)
		client.close()

if __name__ == '__main__':
	main(sys.argv[1:])
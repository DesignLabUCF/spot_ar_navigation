import sys
import socket
from cv2 import imwrite
import numpy as np
import time

# Help:
# https://www.youtube.com/watch?v=0TFWtfFY87U
# https://stackoverflow.com/questions/34653875/python-how-to-send-data-over-tcp

### Default Server params
#host_address = "127.0.0.1" # Local pc?
host_address = "192.168.200.45" # Spot?

#port_id = 9999
port_id = 21001

buffer_size = 1024 # Must be power of 2
#buffer_size = 4096 # Must be power of 2

def main(argv):
	assert len(argv) > 1, "Two arguments must be passed in. One for IP address to ping, and one for port."
	host_address = str(argv[0])
	port_id = int(argv[1])

	print("Attempting to connect to host at: " + host_address + ":" + str(port_id))
	host = socket.gethostname()
	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server.connect((host_address, port_id))

	print(server.recv(buffer_size).decode())

	success = str(server.recv(buffer_size).decode())
	print("Image get status:", success)
	if(success=="True"):
		## Numpy array type - essentially what type of image it is
		dtype = server.recv(buffer_size).decode()
		'''
		print("dtype-str:",dtype)
		print("dtype:", np.dtype(dtype))
		print("npuint8:", (np.dtype(dtype)==np.uint8))
		'''
		## Image shape. Likely sent as flat, so need to reshape next
		shape = server.recv(buffer_size).decode()
		shape = eval(shape) # String to int tuple
		## Image bytes
		img_msg = server.recv(buffer_size)
		'''
		f = open("test_client.txt", "w")
		f.write(img_msg.decode())
		f.close()
		'''
		## Image created from image bytes
		#img = np.frombuffer(img_msg, dtype=np.dtype(dtype))
		#print("Pre-Shape:", str(img.shape))
		#print("Pre-Size:", str(img.size))
		#img = np.reshape(img, shape)
		#imwrite("test_" + str(time.time()).split(".")[0] + ".png", img)
		## Print info
		print("Recieved dtype:", str(dtype))
		print("Recieved shape:", str(shape))
		print("Recieved image msg:", str(img_msg.decode()))
	data = server.recv(buffer_size).decode()
	print(data)

	server.close()

if __name__ == '__main__':
	main(sys.argv[1:])
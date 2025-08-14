import sys
import time
import socket
from spot_image import get_image

# Help:
# https://www.youtube.com/watch?v=0TFWtfFY87U
# https://stackoverflow.com/questions/34653875/python-how-to-send-data-over-tcp

### Default Server params
#host_address = "0.0.0.0" # Local host?
host_address = ""

#port_id = 9999
port_id = 21001

### Spot params
robot_ip = "192.168.200.45" # Defauls

camera_name = "back_fisheye_image" # Defauls

pixel_format = "PIXEL_FORMAT_RGB_U8" # Defauls

def main(argv):
	assert len(argv) > 3, "Three arguments must be passed in. One for Spot's IP address, one for port to host, one for Spot camera name, and one for pixel format."
	robot_ip = str(argv[0])
	port_id = int(argv[1])
	camera_name = str(argv[2])
	pixel_format = str(argv[3])

	### Test
	'''
	from cv2 import imwrite
	succeeded, img = get_image((robot_ip, "--image-sources", camera_name, "--pixel-format", pixel_format))
	print("Status:", succeeded)
	print("Type:", type(img))
	print("NP-Type:", type(img.dtype))
	imwrite(str(time.time()).split(".")[0] + ".png", img)
	'''

	### Launch server
	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server.bind((host_address, port_id))
	print("Server launching...")
	print("Attempting to host at: " + host_address + ":" + str(port_id))
	server.listen()
	print("Server launched!")
	print("Spot IP:", robot_ip)
	print("Spot Camera:", camera_name)
	print("Spot Camera Pixel Format:", pixel_format)

	### Loop n' listen
	while True:
		client, address = server.accept()
		print("Connection recieved from:", address)
		client.send(("Connection made to: " + str(robot_ip) + "." + str(port_id)).encode())# + "\n").encode())
		succeeded, img = get_image((robot_ip, "--image-sources", camera_name, "--pixel-format", pixel_format))
		#succeeded = False
		print("Image retrieval status:", succeeded)
		client.send((str(succeeded)).encode())
		if succeeded:
			dtype = img.dtype
			print("Image dtype:", str(dtype))
			shape = img.shape
			print("Image shape:", str(img.shape))
			client.send((str(dtype)).encode())
			client.send((str(shape)).encode())
			#client.send(img.flatten().tobytes()) # Bytes?
			client.send("ImageHERE".encode())
			'''
			f = open("test_serv.txt", "w")
			f.write((str(img.flatten())))
			f.close()
			'''
		client.send("Disconnecting...\n".encode())
		
		time.sleep(3.0)
		client.close()

if __name__ == '__main__':
	main(sys.argv[1:])
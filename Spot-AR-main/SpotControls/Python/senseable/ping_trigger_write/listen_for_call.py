import sys
import time
import socket
from datetime import datetime
import os


# Help:
# https://www.youtube.com/watch?v=0TFWtfFY87U
# https://stackoverflow.com/questions/34653875/python-how-to-send-data-over-tcp

### Default Server params
host_address = ""

#port_id = 9999
port_id = 21003

buffer_size = 1024 # Must be power of 2

### Spot params
robot_ip = "192.168.200.45" # Defauls


def main(argv):
	assert len(argv) > 2, "Three arguments must be passed in. One for Spot's IP address, one for port to host, and one for write file name."
	robot_ip = str(argv[0])
	port_id = int(argv[1])
	file_name = str(argv[2])
	assert ".txt" in file_name, "File name must be .txt"

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
		
		print("Writing to file:", file_name)
		file_text = client.recv(buffer_size).decode()
		try:
			if not os.path.exists("data"):
				os.makedirs("data")
			with open(os.path.join("data", file_name), "w") as f:
				f.write(datetime.now().strftime("%d/%m/%Y %H:%M:%S"))
				f.write("\n")
				f.write(file_text)
				f.write("\n")
		except:
			print("File write fail.")
		print("File write success!")

		client.send("Disconnecting...\n".encode())
		time.sleep(3.0)
		client.close()

if __name__ == '__main__':
	main(sys.argv[1:])
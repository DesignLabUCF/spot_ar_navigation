import sys
import time
import socket

# Help:
# https://www.youtube.com/watch?v=0TFWtfFY87U
# https://stackoverflow.com/questions/34653875/python-how-to-send-data-over-tcp

### Default Server params
#host_address = "0.0.0.0" # Local host?
host_address = ""

#port_id = 9999
port_id = 21002

### Spot params
robot_ip = "192.168.200.45" # Defauls


def main(argv):
	assert len(argv) > 1, "Two arguments must be passed in. One for Spot's IP address, and one for port to host."
	robot_ip = str(argv[0])
	port_id = int(argv[1])

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
		client.send("This is a test string sent from the server to the client. Seeing this is great news!\n".encode())
		client.send("Disconnecting...\n".encode())
		time.sleep(3.0)
		client.close()

if __name__ == '__main__':
	main(sys.argv[1:])
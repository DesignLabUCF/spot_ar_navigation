import sys
import socket
import time

# Help:
# https://www.youtube.com/watch?v=0TFWtfFY87U
# https://stackoverflow.com/questions/34653875/python-how-to-send-data-over-tcp

### Default Server params
host_address = "192.168.200.45" # Spot?

port_id = 21003

buffer_size = 1024 # Must be power of 2

def main(argv):
	assert len(argv) > 2, "Three arguments must be passed in. One for IP address to ping, one for port, and one for string to save."
	host_address = str(argv[0])
	port_id = int(argv[1])
	data_to_send = str(argv[2])

	print("Attempting to connect to host at: " + host_address + ":" + str(port_id))
	host = socket.gethostname()
	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server.connect((host_address, port_id))

	print(server.recv(buffer_size).decode())

	print("Sending string to write to file:", data_to_send)
	server.send(data_to_send.encode())

	print(server.recv(buffer_size).decode())
	#print(server.recv(buffer_size).decode())

	server.close()

if __name__ == '__main__':
	main(sys.argv[1:])
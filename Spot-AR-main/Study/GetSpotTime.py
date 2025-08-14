import sys
import os
import paramiko # For ssh
from datetime import datetime, timedelta

def confirm_ssh():
	paths = os.environ["PATH"].split(";")
	for pv in paths: # Walk thru every path directory to see if ssh available in cmd
		try:
			path_dir = os.listdir(pv)
			for i in range(0, len(path_dir)):
				sub = path_dir[i].lower()
				if "ssh.exe" in sub:
					return True
					print("ssh located.")
		except OSError as e:
			print(e)
	return False

def execute_command(client, print_output, command):
	command = "cd " + current_directory + " && " + command
	ssh_stdin, ssh_stdout, ssh_stderr = client.exec_command(command)
	ssh_stdout.channel.recv_exit_status()
	# Interpret output
	output_lines = ssh_stdout.readlines()
	for i in range(0, len(output_lines)):
		output_lines[i] = output_lines[i].replace("\n", "")
	if print_output:
		print("Output:")
		print(output_lines)
	# Interpret error
	error_lines = ssh_stderr.readlines()
	for i in range(0, len(error_lines)):
		error_lines[i] = error_lines[i].replace("\n", "")
	if print_output:
		print("Errors:")
		print(error_lines)
	return output_lines, error_lines

def main(argv):
	## Check if commands installed and configured.
	assert confirm_ssh(), "SSH not available on PATH. Please ensure it is installed and available in command prompt." # TODO make a pop-up instead

	## Check dog log in information
	assert len(argv) == 4, "Pass in Spot's IP, Port, username, and password."
	server = argv[0]
	port = argv[1]
	username = argv[2]
	password = argv[3]

	## SSH to Spot
	client = None
	ssh_connected = False
	try:
		## SSH
		client = paramiko.SSHClient()
		client.load_system_host_keys()
		client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		client.connect(server, port, username=username, password=password)
		ssh_connected = True
	except paramiko.AuthenticationException:
		print("Error: Log-in credentials failed")
	# TODO check its fine

	time_offsets = []
	number_to_perform = 30

	if ssh_connected:
		for i in range(0, number_to_perform):
			# Init
			now_spot = None
			now_local = None
			# Run time command on Spot
			ssh_stdin, ssh_stdout, ssh_stderr = client.exec_command("timedatectl")
			ssh_stdout.channel.recv_exit_status()
			# Interpret output
			output_lines = ssh_stdout.readlines()
			for i in range(0, len(output_lines)):
				output_lines[i] = output_lines[i].replace("\n", "")
			# Parse time (Spot)
			output_time = output_lines[0].strip().split(" ")
			date = output_time[3]
			year = int(date.split("-")[0])
			month = int(date.split("-")[1])
			day = int(date.split("-")[2])
			time = output_time[4]
			hour = int(time.split(":")[0]) - 4 # account for UTC to EST
			minute = int(time.split(":")[1])
			second = int(time.split(":")[2])
			timezone = output_time[5]
			#print(date)
			#print(time)
			now_spot = datetime(year, month, day, hour, minute, second)
			# Parse time (local)
			now_local = datetime.now()
			# Check offset
			if now_local != None and now_spot != None:
				time_offsets.append((now_spot - now_local).seconds)
		
		# Output average of all time offsets
		if len(time_offsets) > 0:
			time_offset_avg = sum(time_offsets) / len(time_offsets)
		print(f"Spot is {time_offset_avg} seconds ahead of local machine.")

	# Exit
	if client != None:
		client.close()

if __name__ == "__main__":
	main(sys.argv[1:])
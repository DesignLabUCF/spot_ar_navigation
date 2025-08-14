import sys
from datetime import datetime
import time
import subprocess
import bluetooth
#import pyaudio
import multiprocessing # Used over thread because we can kill these at will, which is likely needed mid-audio play
import logging

#from bosdyn.client.util import setup_logging
#import bosdyn.client.util
#from bosdyn.client.robot_state import RobotStateClient
#from bosdyn.client import ResponseError, RpcError, create_standard_sdk
#import bosdyn.api.robot_state_pb2 as robot_state_proto

from bosdyn.client.util import setup_logging
import bosdyn.client.util
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_state import RobotStateClient
import bosdyn.api.robot_state_pb2 as robot_state_proto


#TEST_AUDIO_PATH = "/test_audio.mp3"
TEST_AUDIO_PATH = "/beep.mp3"
_LOGGER = logging.getLogger('bluetooth_plugin')

# Play audio in console using mpg123. Best used when launched as it's own thread
# TODO it is not exiting the process for some reason, need to fix
def play_audio_console(audio_path):
	try:
		#run_cmd = f"mpg123 -f 8000 {audio_path}"
		run_cmd = ['mpg123', '-f', '8000', audio_path] # https://stackoverflow.com/questions/18962785/oserror-errno-2-no-such-file-or-directory-while-using-python-subprocess-wit
		#subprocess.run(run_cmd, shell=False, stdout=subprocess.PIPE)
		subprocess.Popen(run_cmd, shell=False, stdout=subprocess.PIPE)
	except Exception as e:
		_LOGGER.info(e)
		return False
	return True

def still_connected(mac_address):
	# https://stackoverflow.com/questions/40325218/python-bluetooth-check-connection-status
	stdoutdata = subprocess.getoutput("hcitool con")
	if mac_address in stdoutdata.split():
		return True
	else:
		return False

def get_current_status(client):
	state = client.get_robot_state() # TODO replace with 'robot_state' property

	# E-stop
	current_estop = None
	for estop_state in state.estop_states:
		if estop_state.type == estop_state.TYPE_SOFTWARE:
			current_estop = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
			break
	#output_msg(f'Estop {estop_status} (thread: {thread_status})')

	# Power
	current_power_status = None
	power_state = state.power_state
	motor_power_state = power_state.motor_power_state
	if motor_power_state == robot_state_proto.PowerState.STATE_OFF:
		current_power_status = "Off"
	elif motor_power_state == robot_state_proto.PowerState.STATE_ON:
		current_power_status = "On"
	#output_msg("Motor Power: " + current_power_status)

	# Lease
	current_lease_owner = None 
	leases = lease_client.list_leases_full()
	resources = leases.resources
	for resource in resources:
		resource_name = resource.resource
		owner = resource.lease_owner
		owner_clientname = owner.client_name
		owner_username = owner.user_name
		is_stale = is_stale # Can be taken
		if resource_name == "body":
			current_lease_owner = owner_clientname
		elif resource_name == "mobility":
			pass

	return current_estop, current_power_status, current_lease_owner

	'''
	# Lease
	lease_client.list_leases_full()
	#if lease_keep_alive is None:
	if self._lease_keepalive is None:
		alive = 'STOPPED'
		lease = 'RETURNED'
	else:
		try:
			_lease = self._lease_keepalive.lease_wallet.get_lease()
			lease = f'{_lease.lease_proto.resource}:{_lease.lease_proto.sequence}'
		except bosdyn.client.lease.Error:
			lease = '...'
		if self._lease_keepalive.is_alive():
			alive = 'RUNNING'
		else:
			alive = 'STOPPED'
	#output_msg(f'Lease {lease} THREAD:{alive}')
	'''

def main():
	global e_stop_status
	global power_status
	global lease_status

	setup_logging()

	# Get input args
	import argparse
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Setup robot
	#sdk = create_standard_sdk('RobotStateClient')
	sdk = create_standard_sdk('SENSEable_WASDClient')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)
	robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
	lease_client = robot.ensure_client(LeaseClient.default_service_name)

	try:
		# Discover and read in device info
		_LOGGER.info("Discovering bluetooth devices for 10 seconds...")
		nearby_devices = bluetooth.discover_devices(duration=10, lookup_names=True)
		# TODO - Account for no bluetooth dongle found - "[Errno 19] No such device"
		if len(nearby_devices) > 0:
			_LOGGER.info("Discovered the following devices in Python:")
			primary_device = None
			for d in nearby_devices:
				_LOGGER.info(d)
				if "Prime" in d[1]:
					_LOGGER.info("Python found device: " + d[1])
					primary_device = d
					break
			'''
				Connect to device using subprocess and play looping audio
				https://stackoverflow.com/questions/89228/how-do-i-execute-a-program-or-call-a-system-command
			'''
			# Read device info
			device_mac_address = primary_device[0]
			device_name = primary_device[1]
			# Remove device if already connected for some reason (if not connected, should still run command and have no real issues)
			run_arg = f"bluetoothctl remove {device_mac_address}"
			connect_results = subprocess.run(
				[run_arg],
				capture_output=True,
				text=True,
				#stdout=subprocess.PIPE,
				shell=True)
			# Pair and connect with device
			run_arg = f"bluetoothctl --timeout 10 scan on && bluetoothctl trust {device_mac_address} && bluetoothctl pair {device_mac_address} && bluetoothctl connect {device_mac_address}"
			connect_results = subprocess.run(
				[run_arg],
				capture_output=True,
				text=True,
				#stdout=subprocess.PIPE,
				shell=True)
			_LOGGER.info(connect_results.stdout)
			if not ("Connection successful" in connect_results.stdout):
				raise Exception("Bluetooth connecting failed. Exiting")

			# Main loop
			e_stop_status = None
			power_status = None
			lease_status = None
			while True:
				if still_connected(device_mac_address):
					current_status = get_current_status(robot_state_client) # current_estop, current_power_status, current_lease_owner
					# TODO 
					pass
				else:
					raise Exception("Connection to device was terminated. Exiting")	
		else:
			raise Exception("No devices detected. Exiting")
	except Exception as e:
		_LOGGER.info(e)
		# TODO kill multiprocess if running
		return
	finally:
		pass


if __name__ == '__main__':
	main()
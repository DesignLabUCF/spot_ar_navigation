import sys
from datetime import datetime
import time
import threading
from evdev import InputDevice, categorize, ecodes, list_devices

import logging
from bosdyn.client.util import setup_logging

# From WASD
import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
# import bosdyn.api.robot_command_pb2 as robot_command_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from bosdyn.api import geometry_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.util import duration_str, format_metric, secs_to_hms


#import KeyStack


#import keyboard

#import libinput

#from pynput import keyboard

#import tty
#import termios

'''
	https://python-evdev.readthedocs.io/en/latest/usage.html#reading-events-from-a-device
	https://stackoverflow.com/questions/37134686/how-can-i-get-raw-usb-keyboard-data-with-python

	Ideally, we would use the udev name to access the keyboard data, but had trouble getting that.
	Grabbing from the input events in "/dev/input" works for now.
'''

#DEVICE_PATH = "/dev/input/event5"

_LOGGER = logging.getLogger('controls_wasd_plugin')

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1

class KeyStack():
	def __init__(self):
		self.key_stack = []
	def add(self, key):
		self.key_stack.append(key)	
	def remove(self, key):
		if key in self.key_stack:
			self.key_stack.remove(key)
	def get_stack(self):
		return self.key_stack
	def get_readable_ver(self):
		return "Key Stack: " + ' '.join(str(k) for k in self.key_stack)

# Globals
#key_stack = []
key_stack = KeyStack()
input_active = False
lock = threading.Lock()
trigger_restart = False


def output_msg(msg):
	_LOGGER.setLevel(logging.DEBUG)
	_LOGGER.info(msg)
	#print(msg)	

def get_keyboard():
	device = []
	devices = [InputDevice(path) for path in list_devices()]
	for d in devices:
		if "KEYBOARD" in d.name.upper():
			device.append(d)
	return device
	'''
	# TODO have more dynamic grabbing of keyboard. Maybe by device name if contains keyboard?
	return InputDevice(DEVICE_PATH)
	'''
	'''
	return [InputDevice("/dev/input/event5")]
	'''

# Converts keycode value to easy to read string (Example: 'KEY_D' -> 'D')
def parse_keycode(keycode):
	return keycode[4:]

def read_inputs(device, robot_driver):
	global key_stack
	global input_active
	global trigger_restart

	input_active = True;

	try:
		for event in device.read_loop():
			if event.type == ecodes.EV_KEY:
				cat_event = categorize(event) # Should cast from an 'InputEvent' to 'KeyEvent'
				'''
					https://python-evdev.readthedocs.io/en/latest/_modules/evdev/events.html#InputEvent.timestamp
					cat_event.keycode - The key that was pressed. Example: "KEY_J"
					cat_event.keystate - They state of the key. Example: Down press is 1. Lines up with the key_down, key_up, and key_hold from KeyEvent values
				'''
				keystate = cat_event.keystate
				keycode = cat_event.keycode
				# Key down
				if keystate == 1:
					with lock:
						key_stack.add(keycode)
						output_msg(key_stack.get_readable_ver())
						#robot_driver.drive(parse_keycode(keycode))
				# Key up
				elif keystate == 0:
					with lock:
						if keycode in key_stack.get_stack():
							key_stack.remove(keycode)
							output_msg(key_stack.get_readable_ver())
				# Key held
				elif keystate == 2:
					with lock:
						#robot_driver.drive(parse_keycode(keycode))
						pass
					pass
			# Thread has been killed from outside of it. May only be called on final button press so should find an alternative if this ends up being an issue.
			if input_active == False:
				break
	except Exception as e:
		output_msg("Error in read_input function. Killing thread.")
		output_msg(e)
		trigger_restart = True
		input_active = False
	finally:
		pass

# TODO review and remove
class AsyncRobotState(AsyncPeriodicQuery):
	"""Grab robot state."""

	def __init__(self, robot_state_client):
		super(AsyncRobotState, self).__init__('robot_state', robot_state_client, _LOGGER,
											  period_sec=0.2)

	def _start_query(self):
		return self._client.get_robot_state_async()

# Adapted from wasd.py Boston Dynamics example script
class RobotDriver():
	def __init__(self, robot):
		self._robot = robot
		# Create clients -- do not use the for communication yet.
		self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
		try:
			self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
			self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
		except:
			# Not the estop.
			self._estop_client = None
			self._estop_endpoint = None
		self._power_client = robot.ensure_client(PowerClient.default_service_name)
		self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
		self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
		self._robot_state_task = AsyncRobotState(self._robot_state_client)
		self._lock = threading.Lock()
		self._estop_keepalive = None
		self._exit_check = None
		# Stuff that is set in start()
		self._robot_id = None
		self._lease_keepalive = None    

	def start(self):
		"""Begin communication with the robot."""
		# Construct our lease keep-alive object, which begins RetainLease calls in a thread.
		self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
											   return_at_exit=True)

		self._robot_id = self._robot.get_id()
		if self._estop_endpoint is not None:
			self._estop_endpoint.force_simple_setup(
			)  # Set this endpoint as the robot's sole estop.

	def shutdown(self):
		"""Release control of robot as gracefully as possible."""
		output_msg('Shutting down RobotDriver.')
		if self._estop_keepalive:
			# This stops the check-in thread but does not stop the robot.
			self._estop_keepalive.shutdown()
		if self._lease_keepalive:
			self._lease_keepalive.shutdown()

	'''
	def drive(self, input_command):
		input_command = input_command.upper().strip()
		output_msg("Attempting command: " + input_command)
		if input_command == "R":
			self._self_right()
		elif input_command == "P":
			self._toggle_power()
		elif input_command == "V":
			self._sit()
		elif input_command == "F":
			self._stand()
		elif input_command == "W":
			self._move_forward()
		elif input_command == "S":
			self._move_backward()
		elif input_command == "A":
			self._strafe_left()
		elif input_command == "D":
			self._strafe_right()
		elif input_command == "Q":
			self._turn_left()
		elif input_command == "E":
			self._turn_right()
		elif input_command == "L":
			self._toggle_lease()
		elif input_command == "O":
			self._toggle_estop()
		elif input_command == "T":
			self.display_robot_status()
		else:
			output_msg("Command '" + input_command + "' not recognized.")
	'''

	def drive_single(self, input_command):
		input_command = input_command.upper().strip()
		output_msg("Attempting command: " + input_command)
		output_msg("Comm == T: " + str("T" == input_command))

		if "R" == input_command:
			self._self_right()
			return True
		elif "P" == input_command:
			self._toggle_power()
			return True
		elif "V" == input_command:
			self._sit()
			return True
		elif "F" == input_command:
			self._stand()
			return True
		elif "L" == input_command:
			self._toggle_lease()
			return True
		elif "O" == input_command:
			self._toggle_estop()
			return True
		elif "T" == input_command:
			self.display_robot_status()
			return True

		return False

	def drive(self, input_commands):
		output_msg("Attempting commands: " + ' '.join(str(k) for k in input_commands))

		'''
		if len(input_commands) == 1:
			input_commands = input_commands[0]
		'''

		move_forward = 0
		move_left = 0
		rot_left = 0

		if "W" in input_commands:
			move_forward = move_forward + 1
		if "S" in input_commands:
			move_forward = move_forward - 1
		if "A" in input_commands:
			move_left = move_left + 1
		if "D" in input_commands:
			move_left = move_left - 1
		if "E" in input_commands:
			rot_left = rot_left - 1
		if "D" in input_commands:
			rot_left = rot_left + 1

		move_forward = move_forward * VELOCITY_BASE_SPEED
		move_left = move_left * VELOCITY_BASE_SPEED
		rot_left = rot_left * VELOCITY_BASE_ANGULAR

		#description = ""
		#description = "move"

		#self._velocity_cmd_helper(self, desc="", v_x=move_forward, v_y=move_left, v_rot=rot_left)

		if (move_forward != 0) and (move_left != 0) and (rot_left != 0):
			self._start_robot_command(
						"move_forward", RobotCommandBuilder.synchro_velocity_command(v_x=move_forward, v_y=move_left, v_rot=rot_left),
						end_time_secs=time.time() + VELOCITY_CMD_DURATION)


	@property
	def robot_state(self):
		"""Get latest robot state proto."""
		#return self._robot_state_task.proto
		return self._robot_state_client.get_robot_state()

	def _try_grpc(self, desc, thunk):
		try:
			return thunk()
		except (ResponseError, RpcError, LeaseBaseError) as err:
			output_msg(f'Failed {desc}: {err}')
			return None

	def _try_grpc_async(self, desc, thunk):

		def on_future_done(fut):
			try:
				fut.result()
			except (ResponseError, RpcError, LeaseBaseError) as err:
				output_msg(f'Failed {desc}: {err}')
				return None

		future = thunk()
		future.add_done_callback(on_future_done)

	def _toggle_estop(self):
		"""toggle estop on/off. Initial state is ON"""
		if self._estop_client is not None and self._estop_endpoint is not None:
			if not self._estop_keepalive:
				self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
			else:
				self._try_grpc('stopping estop', self._estop_keepalive.stop)
				self._estop_keepalive.shutdown()
				self._estop_keepalive = None

	def _toggle_lease(self):
		"""toggle lease acquisition. Initial state is acquired"""
		if self._lease_client is not None:
			if self._lease_keepalive is None:
				self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
													   return_at_exit=True)
			else:
				self._lease_keepalive.shutdown()
				self._lease_keepalive = None

	def _start_robot_command(self, desc, command_proto, end_time_secs=None):

		def _start_command():
			self._robot_command_client.robot_command(command=command_proto,
													 end_time_secs=end_time_secs)

		self._try_grpc(desc, _start_command)

	def _self_right(self):
		self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

	def _battery_change_pose(self):
		# Default HINT_RIGHT, maybe add option to choose direction?
		self._start_robot_command(
			'battery_change_pose',
			RobotCommandBuilder.battery_change_pose_command(
				dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT))

	def _sit(self):
		self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

	def _stand(self):
		self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

	def _move_forward(self):
		self._velocity_cmd_helper('move_forward', v_x=VELOCITY_BASE_SPEED)

	def _move_backward(self):
		self._velocity_cmd_helper('move_backward', v_x=-VELOCITY_BASE_SPEED)

	def _strafe_left(self):
		self._velocity_cmd_helper('strafe_left', v_y=VELOCITY_BASE_SPEED)

	def _strafe_right(self):
		self._velocity_cmd_helper('strafe_right', v_y=-VELOCITY_BASE_SPEED)

	def _turn_left(self):
		self._velocity_cmd_helper('turn_left', v_rot=VELOCITY_BASE_ANGULAR)

	def _turn_right(self):
		self._velocity_cmd_helper('turn_right', v_rot=-VELOCITY_BASE_ANGULAR)

	def _stop(self):
		self._start_robot_command('stop', RobotCommandBuilder.stop_command())

	def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
		self._start_robot_command(
			desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
			end_time_secs=time.time() + VELOCITY_CMD_DURATION)

	def _stow(self):
		self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

	def _unstow(self):
		self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())

	def _toggle_power(self):
		power_state = self._power_state()
		if power_state is None:
			output_msg('Could not toggle power because power state is unknown')
			return

		if power_state == robot_state_proto.PowerState.STATE_OFF:
			self._try_grpc_async('powering-on', self._request_power_on)
		else:
			self._try_grpc('powering-off', self._safe_power_off)

	def _request_power_on(self):
		request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
		return self._power_client.power_command_async(request)

	def _safe_power_off(self):
		self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

	def _power_state(self):
		state = self.robot_state
		if not state:
			return None
		return state.power_state.motor_power_state

	def display_robot_status(self):
		output_msg("Display robot status information...")

		state = self._robot_state_client.get_robot_state() # TODO replace with 'robot_state' property

		# E-stop
		if not self._estop_client:
			thread_status = 'NOT ESTOP'
		else:
			thread_status = 'RUNNING' if self._estop_keepalive else 'STOPPED'
		estop_status = '??'
		if state:
			for estop_state in state.estop_states:
				if estop_state.type == estop_state.TYPE_SOFTWARE:
					estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
					break
		output_msg(f'Estop {estop_status} (thread: {thread_status})')

		# Power
		power_state = state.power_state
		motor_power_state = power_state.motor_power_state
		power_status = ""
		if motor_power_state == robot_state_proto.PowerState.STATE_OFF:
			power_status = "Off"
		elif motor_power_state == robot_state_pb2.PowerState.STATE_ON:
			power_status = "On"
		output_msg("Motor Power: " + power_status)

		# Lease
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
		output_msg(f'Lease {lease} THREAD:{alive}')
		

def main():
	global key_stack
	global input_active
	global trigger_restart

	# Get input args
	import argparse
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Init Spot stuff
	setup_logging()
	sdk = create_standard_sdk('SENSEable_WASDClient')
	robot = sdk.create_robot(options.hostname)
	try:
		bosdyn.client.util.authenticate(robot)
		#robot.start_time_sync(options.time_sync_interval_sec) # TODO ???
		robot.start_time_sync()
	except RpcError as err:
		_LOGGER.error('Failed to communicate with robot: %s', err)
		return False

	# Launch driver
	robot_driver = RobotDriver(robot)
	try:
		robot_driver.start()
	except (ResponseError, RpcError) as err:
		_LOGGER.error('Failed to initialize robot communication: %s', err)
		return False

	# Get keyboard information and launch controls thread(s)
	output_msg("Retrieving Keyboard information...")
	devices = []
	try:
		devices = get_keyboard() 
		if devices == []:
			raise Exception("Keyboard unable to be found")
		output_msg("Using device(s): " + str(devices))
	except:
		output_msg("Exiting...")
		return 0

	# Start reading WASD input (dev/input/ may have multiple events tied to the keyboard. Because of this, we will launch a thread for each of these, although only 1 should actually do anything besides loop indefinitely)
	output_msg("Launching WASD controls...")
	input_threads = []
	for device in devices:
		#input_thread = Thread(target=read_inputs)
		#input_thread = Thread(target=lambda: read_inputs(device))
		output_msg("Launching thread for device with path: " + device.path)
		#input_thread = threading.Thread(target=read_inputs, args=(device,))		
		#input_thread = threading.Thread(target=read_inputs, args=(device, robot_driver))
		input_thread = threading.Thread(target=read_inputs, daemon=True, args=(device, robot_driver)) # Daemon threads will auto-die when main thread does
		
		input_threads.append(input_thread)	
		input_thread.start()
	key_stack_prev_tick = []

	# Main program loop
	try:
		while trigger_restart == False:
			#output_msg(key_stack.get_readable_ver())

			exit_frame = False # Can only really execute one command a frame, so use this to exit

			output_msg("Current: " + ' '.join(str(k) for k in key_stack.get_stack()))
			output_msg("Prev:    " + ' '.join(str(k) for k in key_stack_prev_tick))

			# Misc. commands
			if exit_frame == False:
				for k in key_stack.get_stack():
					# New keypress down frame
					if (k in key_stack_prev_tick) == False:
						command_executed = robot_driver.drive_single(parse_keycode(k)) # Trigger single command
						output_msg("Command executed: " + str(command_executed))
						if command_executed:
							exit_frame = True
							break

			# Movement commands
			commands = []
			if exit_frame == False:
				for k in key_stack.get_stack():
					commands.append(parse_keycode(k))
					#robot_driver.drive(parse_keycode(k))
				if len(commands) > 0:
					robot_driver.drive(commands)

			key_stack_prev_tick.clear()
			key_stack_prev_tick = key_stack.get_stack()

			time.sleep(1.0 / 5.0)
			pass
	except Exception as e:
		output_msg(e)
		pass
	finally:
		input_active = False
		robot_driver.shutdown()
		return


if __name__ == '__main__':
	main()
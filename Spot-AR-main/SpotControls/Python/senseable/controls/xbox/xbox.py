import sys
from datetime import datetime
import time
import os
import threading
import bluetooth
import subprocess
# Boston Dynamics
import logging
from bosdyn.client.util import setup_logging
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
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api import trajectory_pb2
# Evdev
from evdev import InputDevice, categorize, ecodes, list_devices
from evdev.ecodes import *
#from evdev.ecodes import EV_KEY, EV_REL, EV_ABS, EV_SYN
#from evdev.ecodes import ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ
# From hello_spot.py
import bosdyn.geometry
# Service
from bosdyn.api import data_acquisition_pb2
from bosdyn.client.data_acquisition import DataAcquisitionClient
from bosdyn.client.data_acquisition_helpers import acquire_and_process_request, cancel_acquisition_request, download_data_REST, issue_acquire_data_request, make_time_query_params


'''
	A script used to emulate the control methods of the Boston Dynamics Spot tablet application. Contains both a walk mode and stance mode. Can swap between the two using the 'start' button.
	If ran in a container, ensure an Xbox controller is connected when the script is ran. If one is not detected, the container will restart (if built using our provided Dockerfile/configuration).

	1. Connect controller
	2. Run application/container
	3. Ensure lease is availble to be taken. Logs will indicate if it is unavailable. Easiest way to manually reset it is to sign onto tablet application, enter 'Observe mode', then sign back out. This script will then automatically take control of the lease.
	4. Press 'right bumper' on controller to deactivate e-stop.
	5. Press 'X' button to start motors. You should hear them on Spot to confirm they are starting up.
	6. Once motots are running, press 'A' button to stand up
	7. Have fun!
	8. When done, manually return lease by pressing 'Y' button.

	To map to your own own buttons, make a new InputDevice object and call .capabilities(verbose=True) on it. This will get a map of all the buttons and you can compare to the default xbox output here. Also consider .active_keys() and .device.absinfo([AXIS_INTEGER]).
	{('EV_SYN', 0): [('SYN_REPORT', 0), ('SYN_CONFIG', 1), ('SYN_DROPPED', 3)], ('EV_KEY', 1): [(['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH'], 304), (['BTN_B', 'BTN_EAST'], 305), (['BTN_NORTH', 'BTN_X'], 307), (['BTN_WEST', 'BTN_Y'], 308), ('BTN_TL', 310), ('BTN_TR', 311), ('BTN_SELECT', 314), ('BTN_START', 315), ('BTN_MODE', 316), ('BTN_THUMBL', 317), ('BTN_THUMBR', 318)], ('EV_ABS', 3): [(('ABS_X', 0), AbsInfo(value=0, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)), (('ABS_Y', 1), AbsInfo(value=-1, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)), (('ABS_Z', 2), AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)), (('ABS_RX', 3), AbsInfo(value=77, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)), (('ABS_RY', 4), AbsInfo(value=-1, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)), (('ABS_RZ', 5), AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)), (('ABS_HAT0X', 16), AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)), (('ABS_HAT0Y', 17), AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0))]}
'''

'''
	Using xboxdrv to get Ubunutu to recognize the controller.
	Using evdev to read that input into Python. Reads /dev/input/event* for input events and translates them here.
	Wired xbox controller also produces onto /dev/input/js0 but this appears to be an older methodology that is legacy, and therefore not usable by evdev library.
'''


# Connection parameters
BLUETOOTH_SCAN_TIME = 5 # seconds
# Movement parameters - Custom
#VELOCITY_BASE_SPEED = 1.0 # m/s
#VELOCITY_BASE_ANGULAR = 1.5  # rad/sec
#VELOCITY_STRAFE_SPEED = VELOCITY_BASE_SPEED
# Movement parameters - Same as tablet with STAIRS MODE on
VELOCITY_BASE_SPEED = 1.0
VELOCITY_BASE_ANGULAR = 0.75 
VELOCITY_STRAFE_SPEED = 0.35
# Movement parameters - other
VELOCITY_CMD_DURATION = 0.25  # seconds
COMMAND_INPUT_RATE = 0.1
HEIGHT_MULTIPLIER = 0.25
STICK_THRESHOLD = 0.15 # Percentage of stick pressure applied needed to issue commands

_LOGGER = logging.getLogger('controls_xbox_plugin')
reading_active = True
transform_record_service_active = False


# Used by robot driver
class AsyncRobotState(AsyncPeriodicQuery):
	"""Grab robot state."""
	def __init__(self, robot_state_client):
		super(AsyncRobotState, self).__init__('robot_state', robot_state_client, _LOGGER,
											  period_sec=0.2)

	def _start_query(self):
		return self._client.get_robot_state_async()

# Handles all robot movement - Adapted from wasd.py Boston Dynamics example scripts
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
		#self._lock = threading.Lock()
		self._estop_keepalive = None
		self._exit_check = None
		# Stuff that is set in start()
		self._robot_id = None
		self._lease_keepalive = None
		self.standing_mode = False
		self.mobility_params = self.set_mobility_params()

	def start(self):
		if self.lease_should_be_taken(): # Our previous iteration owned the lease and wasn't able to give it up because of a bad crash or something. Just snag it back for now
			self._lease_client.take()
		"""Begin communication with the robot."""
		# Construct our lease keep-alive object, which begins RetainLease calls in a thread.
		self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
											   return_at_exit=True, on_failure_callback=self.lease_retain_error)

		self._robot_id = self._robot.get_id()
		if self._estop_endpoint is not None:
			self._estop_endpoint.force_simple_setup(
			)  # Set this endpoint as the robot's sole estop.

	def shutdown(self):
		"""Release control of robot as gracefully as possible."""
		_LOGGER.info('Shutting down RobotDriver.')
		if self._estop_keepalive:
			# This stops the check-in thread but does not stop the robot.
			self._estop_keepalive.shutdown()
		if self._lease_keepalive:
			self._lease_keepalive.shutdown()

	@property
	def robot_state(self):
		"""Get latest robot state proto."""
		#return self._robot_state_task.proto
		return self._robot_state_client.get_robot_state()

	def _try_grpc(self, desc, thunk):
		try:
			return thunk()
		except (ResponseError, RpcError, LeaseBaseError) as err:
			_LOGGER.info(f'Failed {desc}: {err}')
			return None

	def _try_grpc_async(self, desc, thunk):

		def on_future_done(fut):
			try:
				fut.result()
			except (ResponseError, RpcError, LeaseBaseError) as err:
				_LOGGER.info(f'Failed {desc}: {err}')
				return None

		future = thunk()
		future.add_done_callback(on_future_done)

	def _quit_program(self):
		self._sit()
		if self._exit_check is not None:
			self._exit_check.request_exit()

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
			desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=self.mobility_params),
			end_time_secs=time.time() + VELOCITY_CMD_DURATION)

	def _stow(self):
		self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

	def _unstow(self):
		self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())

	def _toggle_power(self):
		power_state = self._power_state()
		if power_state is None:
			_LOGGER.info('Could not toggle power because power state is unknown')
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

	def toggle_standing_mode(self):
		self.set_standing_mode(not self.standing_mode)

	def set_standing_mode(self, should_be_standing):
		self.standing_mode = should_be_standing

	def set_stance(self, yaw=0, pitch=0, roll=0, height=0):
		footprint_R_body = bosdyn.geometry.EulerZXY(yaw=yaw, roll=roll, pitch=pitch)
		cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body, body_height=height)
		self._robot_command_client.robot_command(cmd)		

	def _lease_str(self, lease_keep_alive):
		if lease_keep_alive is None:
			alive = 'STOPPED'
			lease = 'RETURNED'
		else:
			try:
				_lease = lease_keep_alive.lease_wallet.get_lease()
				lease = f'{_lease.lease_proto.resource}:{_lease.lease_proto.sequence}'
			except bosdyn.client.lease.Error:
				lease = '...'
			if lease_keep_alive.is_alive():
				alive = 'RUNNING'
			else:
				alive = 'STOPPED'
		return f'Lease {lease} THREAD:{alive}'

	def _power_state_str(self):
		power_state = self._power_state()
		if power_state is None:
			return ''
		state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
		return f'Power: {state_str[6:]}'  # get rid of STATE_ prefix

	def _estop_str(self):
		if not self._estop_client:
			thread_status = 'NOT ESTOP'
		else:
			thread_status = 'RUNNING' if self._estop_keepalive else 'STOPPED'
		estop_status = '??'
		state = self.robot_state
		if state:
			for estop_state in state.estop_states:
				if estop_state.type == estop_state.TYPE_SOFTWARE:
					estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
					break
		return f'Estop {estop_status} (thread: {thread_status})'

	def _battery_str(self):
		if not self.robot_state:
			return ''
		battery_state = self.robot_state.battery_states[0]
		status = battery_state.Status.Name(battery_state.status)
		status = status[7:]  # get rid of STATUS_ prefix
		if battery_state.charge_percentage.value:
			bar_len = int(battery_state.charge_percentage.value) // 10
			bat_bar = f'|{"=" * bar_len}{" " * (10 - bar_len)}|'
		else:
			bat_bar = ''
		time_left = ''
		if battery_state.estimated_runtime:
			time_left = f'({secs_to_hms(battery_state.estimated_runtime.seconds)})'
		return f'Battery: {status}{bat_bar} {time_left}'


	# TODO
	def display_robot_status(self):
		_LOGGER.info("Display robot status information...")

		#state = self._lease_client.get_robot_state()
		state = self._robot_state_client.get_robot_state() # TODO replace with 'robot_state' property
		#_LOGGER.info(state)

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
		_LOGGER.info(f'Estop {estop_status} (thread: {thread_status})')

		# Power
		power_state = state.power_state
		motor_power_state = power_state.motor_power_state
		power_status = ""
		if motor_power_state == robot_state_proto.PowerState.STATE_OFF:
			power_status = "Off"
		elif motor_power_state == robot_state_proto.PowerState.STATE_ON:
			power_status = "On"
		_LOGGER.info("Motor Power: " + power_status)

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
		_LOGGER.info(f'Lease {lease} THREAD:{alive}')

		# Standing mode
		_LOGGER.info("Standing Mode: " + str(self.standing_mode))

	def lease_retain_error(self, error):
		global reading_active

		_LOGGER.info("Lease error encountered, shutting down robot driver...")
		reading_active = False # Kill control input thread
		self.shutdown()
		raise Exception("Exiting...")

	def lease_should_be_taken(self):
		leases = self._lease_client.list_leases_full()
		resources = leases.resources
		for resource in resources:
			resource_name = resource.resource
			owner = resource.lease_owner
			owner_clientname = owner.client_name
			owner_username = owner.user_name
			is_stale = resource.is_stale # Can be taken
			if resource_name == "body": # Found lease owner
				current_lease_owner = owner_clientname
				if "SENSEable_XboxClient" in current_lease_owner:
					return True
				else:
					return False
		return False

	# Adapted from fidicual_follow.py
	def set_mobility_params(self):
		"""Set robot mobility params to disable obstacle avoidance."""
		body_control = self.set_default_body_control()
		max_x_vel = VELOCITY_BASE_SPEED
		max_y_vel = VELOCITY_STRAFE_SPEED
		max_ang_vel = VELOCITY_BASE_ANGULAR # 1.6 appears to be max
		speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=max_x_vel, y=max_y_vel), angular=max_ang_vel))
		#speed_limit = None
		stairs_mode = 2 # Off = 1, On = 2, Auto = 3 (https://github.com/boston-dynamics/spot-sdk/blob/6a03ae12056a74e65fd9715128e0c40762dfa7ba/protos/bosdyn/api/spot/robot_command.proto#L76)
		mobility_params = spot_command_pb2.MobilityParams(
			vel_limit=speed_limit,
			body_control=body_control,
			locomotion_hint=spot_command_pb2.HINT_AUTO,
			stairs_mode=stairs_mode)
		return mobility_params

	@staticmethod
	def set_default_body_control():
		"""Set default body control params to current body position"""
		footprint_R_body = bosdyn.geometry.EulerZXY()
		position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
		rotation = footprint_R_body.to_quaternion()
		pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
		point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
		traj = trajectory_pb2.SE3Trajectory(points=[point])
		return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

class ControllerButton(object):
	def __init__(self, name):
		self.name = name
		self.pressed = 0
		self.should_ignore = 0

	def set_press_status(self, should_press):
		if should_press == 1:
			self.press()
		else:
			self.release()

	def press(self):
		self.pressed = 1
		self.should_ignore = 0

	def release(self):
		self.pressed = 0
		self.should_ignore = 0

	def command_processed(self):
		#self.pressed = 0
		self.should_ignore = 1

	def should_process_command(self):
		if (self.pressed == 1) and (self.should_ignore == 0):
			return True
		else:
			return False

class XboxController(object):
	# 'Xbox' or 'Sony'
	device_type = ""
	# Buttons
	A_BUTTON = -1
	B_BUTTON = -1
	X_BUTTON = -1
	Y_BUTTON = -1
	LB_BUTTON = -1
	RB_BUTTON = -1
	BACK_BUTTON = -1
	START_BUTTON = -1
	LSTICK_BUTTON = -1 
	RSTICK_BUTTON = -1
	XBOX_BUTTON = -1
	# Triggers
	LEFT_TRIGGER = -1
	RIGHT_TRIGGER = -1
	# Sticks
	LEFT_STICK_X = -1
	LEFT_STICK_Y = -1
	RIGHT_STICK_X = -1
	RIGHT_STICK_Y = -1
	# D-pad
	LEFT_DPAD = -1
	RIGHT_DPAD = -1
	UP_DPAD = -1
	DOWN_DPAD = -1

	def __init__(self, device_type):
		# Axes
		self.LeftJoystickY = 0
		self.LeftJoystickX = 0
		self.RightJoystickY = 0
		self.RightJoystickX = 0
		self.LeftTrigger = 0
		self.RightTrigger = 0
		# Buttons
		self.LeftBumper = ControllerButton("LB")
		self.RightBumper = ControllerButton("RB")
		self.A = ControllerButton("A")
		self.X = ControllerButton("X")
		self.Y = ControllerButton("Y")
		self.B = ControllerButton("B")
		self.Back = ControllerButton("Back")
		self.Start = ControllerButton("Start")
		self.Xbox = ControllerButton("Xbox")
		self.LeftThumb = ControllerButton("LStick")
		self.RightThumb = ControllerButton("RStick")
		self.LeftDPad = ControllerButton("LDPad")
		self.RightDPad = ControllerButton("RDPad")
		self.UpDPad = ControllerButton("UDPad")
		self.DownDPad = ControllerButton("DDPad")
		# Set type
		if device_type == "Xbox":
			self.set_as_xbox()
		else:
			self.set_as_sony()

	def ensure_connected(self):
		#InputDevice(DEVICE_PATH)
		return True # TODO

	def launch_reading_thread(self, device):
		self._monitor_thread = threading.Thread(target=self.monitor_controller, daemon=True, args=(device,)) # Daemon threads will auto-die when main thread does
		self._monitor_thread.daemon = True
		self._monitor_thread.start()

	def print_buttons(self):
		_LOGGER.info(" ".join(str(bool(k)) for k in [self.A.pressed, self.B.pressed, self.X.pressed, self.Y.pressed, self.Back.pressed, self.Start.pressed, self.Xbox.pressed, self.LeftBumper.pressed, self.RightBumper.pressed, self.LeftDPad.pressed, self.RightDPad.pressed, self.UpDPad.pressed, self.DownDPad.pressed]))

	def print_axes(self):
		_LOGGER.info(" ".join(str(k) for k in [self.LeftJoystickX, self.LeftJoystickY, self.RightJoystickX, self.RightJoystickY, self.LeftTrigger, self.RightTrigger]))

	def monitor_controller(self, device):
		global reading_active
		try:
			for event in device.read_loop():
				#_LOGGER.info(str(event)) # Event should be of type InputEvent here
				#cat_event = categorize(event) # Figure out it's evdev event type
				key_code = int(event.code)
				key_value = int(event.value)
				'''
					Device axes:
						device.absinfo(ABS_Z) => device.absinfo(2) => device.absinfo(LEFT_TRIGGER) => AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)
						device.absinfo(ABS_X) => device.absinfo(0) => device.absinfo(????) => AbsInfo(value=0, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)
					Button presses are Keyboard events (EV_KEY), but a sychronization event (EV_SYN) accompanies them (with empty values)
					Value: 1 = down, 0 = up
					Code (Wired Xbox controller):
						304 = A
						305 = B
						307 = X
						308 = Y
						310 = Left bumper
						311 = Right bumper
						314 = Select
						315 = Start
						316 = Xbox
					Trigger presses are Absolute Axis events (EV_ABS), but a sychronization event (EV_SYN) accompanies them (with empty values)
					Value: 0 = completely up, 1023 = completely pressed
					Code (Wired Xbox controller):
						2 = Left trigger
						5 = Right trigger
				'''
				if event.type == EV_KEY:
					if key_code == self.A_BUTTON:
						self.A.set_press_status(key_value)
					elif key_code == self.B_BUTTON:
						self.B.set_press_status(key_value)
					elif key_code == self.X_BUTTON:
						self.X.set_press_status(key_value)
					elif key_code == self.Y_BUTTON:
						self.Y.set_press_status(key_value)
					elif key_code == self.LB_BUTTON:
						self.LeftBumper.set_press_status(key_value)
					elif key_code == self.RB_BUTTON:
						self.RightBumper.set_press_status(key_value)
					elif key_code == self.BACK_BUTTON:
						self.Back.set_press_status(key_value)
					elif key_code == self.START_BUTTON:
						self.Start.set_press_status(key_value)
					elif key_code == self.XBOX_BUTTON:
						self.Xbox.set_press_status(key_value)
					elif key_code == self.LSTICK_BUTTON:
						self.LeftThumb.set_press_status(key_value)
					elif key_code == self.RSTICK_BUTTON:
						self.RightThumb.set_press_status(key_value)
				elif event.type == EV_REL:
					#_LOGGER.info("Relative axis event detected")
					pass
				elif event.type == EV_ABS:
					#_LOGGER.info("Absolute Axis event detected")
					if key_code == self.LEFT_STICK_X:
						self.LeftJoystickX = self.map_stick_value(device, key_code)
					elif key_code == self.LEFT_STICK_Y:
						self.LeftJoystickY = self.map_stick_value(device, key_code)
						self.LeftJoystickY = -self.LeftJoystickY # On xbox controller, y-axis inverted
					elif key_code == self.RIGHT_STICK_X:
						self.RightJoystickX = self.map_stick_value(device, key_code)
					elif key_code == self.RIGHT_STICK_Y:
						self.RightJoystickY = self.map_stick_value(device, key_code)
						self.RightJoystickY = -self.RightJoystickY # On xbox controller, y-axis inverted
					elif key_code == self.LEFT_TRIGGER:
						self.LeftTrigger = self.map_trigger_value(device, key_code)
					elif key_code == self.RIGHT_TRIGGER:
						self.RightTrigger = self.map_trigger_value(device, key_code)
					elif key_code == self.LEFT_DPAD or key_code == self.RIGHT_DPAD: # Should be a button but recognized as an axis
						if key_value < 0:
							self.LeftDPad.set_press_status(1)
						elif key_value > 0:
							self.RightDPad.set_press_status(1)
						else:
							self.LeftDPad.set_press_status(0)
							self.RightDPad.set_press_status(0)
					elif key_code == self.UP_DPAD or key_code == self.DOWN_DPAD:
						if key_value < 0:
							self.UpDPad.set_press_status(1)
						elif key_value > 0:
							self.DownDPad.set_press_status(1)
						else:
							self.UpDPad.set_press_status(0)
							self.DownDPad.set_press_status(0)
					#_LOGGER.info(str(event))
				elif event.type == EV_SYN:
					#_LOGGER.info("Sychronization event detected")
					pass
				else:
					#_LOGGER.info("Other event type detected...")
					pass
		except Exception as e:
			_LOGGER.info("Error in monitor_controller function. Killing thread.")
			_LOGGER.info(str(e))
			ensure_device_is_disconnected()
			reading_active = False # Should set signal to kill main thread, thus ending all others and restarting container
			return None	

	# Map stick value from -1 (down) to 1 (up)
	def map_stick_value(self, device, axis):
		absinfo = device.absinfo(axis)
		minimum = absinfo.min
		maximum = absinfo.max
		value = absinfo.value
		# Calculate mapped value
		'''
		mapped_value = 0
		if value >= 0:
			mapped_value = value / maximum
		else:
			mapped_value = -(value / minimum)
		return mapped_value
		'''
		# https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
		axis_range = maximum - minimum
		desired_minimum = -1
		desired_maximum = 1
		desired_range = desired_maximum - desired_minimum
		return ((((value - minimum) * desired_range) / axis_range) + desired_minimum)

	# Map stick value from 0 (uncompressed) to 1 (compressed)
	def map_trigger_value(self, device, axis):
		absinfo = device.absinfo(axis)
		minimum = absinfo.min
		maximum = absinfo.max
		value = absinfo.value
		# Calculate mapped value
		mapped_value = value / (maximum - minimum)
		return mapped_value

	def set_as_xbox(self):
		# Type
		self.device_type = "Xbox"
		# Buttons
		self.A_BUTTON = BTN_A # 304
		self.B_BUTTON = BTN_B # 305
		self.X_BUTTON = BTN_X # 307
		self.Y_BUTTON = BTN_Y # 308
		self.LB_BUTTON = BTN_TL # 310
		self.RB_BUTTON = BTN_TR # 311
		self.BACK_BUTTON = BTN_SELECT # 314
		self.START_BUTTON = BTN_START # 315
		self.LSTICK_BUTTON = BTN_THUMBL # 317
		self.RSTICK_BUTTON = BTN_THUMBR # 318
		self.XBOX_BUTTON = BTN_MODE # 316
		# Triggers
		self.LEFT_TRIGGER = ABS_Z # 2
		self.RIGHT_TRIGGER = ABS_RZ # 5
		# Sticks
		self.LEFT_STICK_X = ABS_X # 0
		self.LEFT_STICK_Y = ABS_Y # 1
		self.RIGHT_STICK_X = ABS_RX # 3
		self.RIGHT_STICK_Y = ABS_RY # 4
		# D-pad - Codes as an axis for some reason - 0 = nothing , 1/-1 = up or down
		self.LEFT_DPAD = ABS_HAT0X # 16
		self.RIGHT_DPAD = ABS_HAT0X # 16
		self.UP_DPAD = ABS_HAT0Y # 17
		self.DOWN_DPAD = ABS_HAT0Y # 17

	def set_as_sony(self):
		# Type
		self.device_type = "Sony"
		# Buttons
		self.A_BUTTON = 305 #BTN_B # 305
		self.B_BUTTON = 306 #BTC_C # 306
		self.X_BUTTON = 304 #BTN_A # 304
		self.Y_BUTTON = 307 #BTN_X # 307
		self.LB_BUTTON = 308 #BTN_Y # 308
		self.RB_BUTTON = 309 #BTN_Z # 309
		self.BACK_BUTTON = 312 #BTN_TL2 # 312
		self.START_BUTTON = 313 #BTN_TR2 # 313
		self.LSTICK_BUTTON = BTN_SELECT # 314
		self.RSTICK_BUTTON = BTN_START # 315
		self.XBOX_BUTTON = 316 #BTN_MODE # 316
		# Triggers
		self.LEFT_TRIGGER = 3 #ABS_RX # 3
		self.RIGHT_TRIGGER = 4 #ABS_RY # 4
		# Sticks
		self.LEFT_STICK_X = 0 #ABS_X # 0
		self.LEFT_STICK_Y = 1 #ABS_Y # 1
		self.RIGHT_STICK_X = 2 #ABS_Z # 2
		self.RIGHT_STICK_Y = 5 #ABS_RZ # 5
		# D-pad
		self.LEFT_DPAD = ABS_HAT0X # 16
		self.RIGHT_DPAD = ABS_HAT0X # 16
		self.UP_DPAD = ABS_HAT0Y # 17
		self.DOWN_DPAD = ABS_HAT0Y # 17


def get_controller():
	devices = []
	device_type = ""
	for d in [InputDevice(path) for path in list_devices()]:
		device_name = d.name.upper()
		_LOGGER.info(device_name)
		'''
		if ("XBOX" in device_name) or ("X-BOX" in device_name) or ("SONY" in device_name) or ("CONTROLLER" in device_name):
			pass # TODO get if SONY or XBOX
			devices.append(d)
		'''
		if ("XBOX" in device_name) or ("X-BOX" in device_name):
			devices.append(d)
			device_type = "Xbox"
		elif ("SONY" in device_name) or ("CONTROLLER" in device_name):
			devices.append(d)
			device_type = "Sony"			
	return devices, device_type
	'''
	devices = []
	for d in [InputDevice(path) for path in list_devices()]:
		try:
			devices.append(d)
		except Exception as e:
			pass
	return devices
	'''

def is_bluetooth_dongle_connected():
	_LOGGER.info("Checking if Bluetooth dongle is connected...")
	connect_results = subprocess.run(["hcitool dev"], capture_output=True, text=True, shell=True) # will return something like just "Devices:" if no bluetooth connected
	if len(connect_results.stdout) < 20: # Basically picked a random number. Should be long enough
		return False
	else:
		return True

# Bluetooth controller was connected previously before container was restarted
def bluetooth_controller_already_connected():
	results = subprocess.run(["bluetoothctl devices"], capture_output=True, text=True, shell=True)
	if results.stdout == '':
		return False
	else:
		return True

def attempt_bluetooth_connection():
	# Discover and read in device info
	_LOGGER.info(f"Discovering bluetooth devices for {BLUETOOTH_SCAN_TIME} seconds...")
	nearby_devices = bluetooth.discover_devices(duration=BLUETOOTH_SCAN_TIME, lookup_names=True)
	if len(nearby_devices) > 0:
		_LOGGER.info("Discovered the following devices when run from Python:")
		primary_device = None
		for d in nearby_devices:
			_LOGGER.info(d)
			if "CONTROLLER" in d[1].upper():
				_LOGGER.info("Python found device: " + d[1])
				primary_device = d
				break
		# No usable device found
		if primary_device == None:
			return False
		# Read device info
		device_mac_address = primary_device[0]
		device_name = primary_device[1]
		'''
		# Remove device if already connected for some reason (if not connected, should still run command and have no real issues)
		run_arg = f"bluetoothctl remove {device_mac_address}"
		connect_results = subprocess.run(
			[run_arg],
			capture_output=True,
			text=True,
			#stdout=subprocess.PIPE,
			shell=True)
		'''
		# Pair and connect with device
		run_arg = f"bluetoothctl --timeout {int(BLUETOOTH_SCAN_TIME)} scan on && bluetoothctl trust {device_mac_address} && bluetoothctl pair {device_mac_address} && bluetoothctl connect {device_mac_address}"
		connect_results = subprocess.run(
			[run_arg],
			capture_output=True,
			text=True,
			#stdout=subprocess.PIPE,
			shell=True)
		_LOGGER.info(connect_results.stdout)
		# Check output from connection terminal command
		if "Connection successful" in connect_results.stdout:
			return True
		else:
			return False
	else:
		return False

def ensure_device_is_disconnected():
	results = subprocess.run(["bluetoothctl devices"], capture_output=True, text=True, shell=True)
	for device in results.stdout.split("\n"):
		if len(device) > 0:
			s = device.split(" ")
			mac_address = s[1].strip()
			results = subprocess.run([f"bluetoothctl remove {mac_address}"], capture_output=True, text=True, shell=True)
	if results.stdout == '':
		return False
	else:
		return True

# An alternative to triggering the service on the tablet
# Adapted from spot-sdk\python\examples\data_acquisition_service\data_acquisition_example.py
def trigger_recording_service(robot):
	global transform_record_service_active
	_LOGGER.info("Calling transform_recorder_service...")
	_LOGGER.info("Existing status: " + str(transform_record_service_active))
	try:
		data_acq_client = robot.ensure_client(DataAcquisitionClient.default_service_name)
		now = robot.time_sync.robot_timestamp_from_local_secs(time.time())
		group_name = f'DataAcquisitionExample_{now.ToJsonString().replace(":", "-")}' # Just going to leave this as is
		acquisition_requests = data_acquisition_pb2.AcquisitionRequestList()
		acquisition_requests.data_captures.extend([data_acquisition_pb2.DataCapture(name='transform')]) # transform_record_service capability name
		# Issue request to start/stop recording
		request_id, action_id = issue_acquire_data_request(data_acq_client, acquisition_requests, group_name, 'AcquisitionsToCancel')
		_LOGGER.info("transform_recorder_service succesfully called.")
		transform_record_service_active = not transform_record_service_active
		_LOGGER.info("New status: " + str(transform_record_service_active))
		return True
	except Exception as e:
		_LOGGER.info(e)
		_LOGGER.info("Unable to call transform_recorder_service.")
		return False

def read_once(device):
	try:
		for event in device.read_loop():
			return event
	except Exception as e:
		_LOGGER.info("Error in read_once function. Exiting...")
		_LOGGER.info(e)
		return None		

def give_time_for_action(action_name, duration):
	_LOGGER.info(f"Giving time for {action_name} action to complete...")
	for i in range(int(duration), 0, -1):
		_LOGGER.info(str(i) + "...")
		time.sleep(1)

def main():
	global reading_active
	global transform_record_service_active

	# Init
	setup_logging()
	import argparse
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	parser.add_argument("-b", '--use-bluetooth', help='Whether controller is wired or not (True/False)',
				default="False")
	'''
	parser.add_argument('--bluetooth', dest='use_bluetooth', action='store_true')
	parser.add_argument('--no-bluetooth', dest='use_bluetooth', action='store_false')
	parser.set_defaults(use_bluetooth=False)
	'''

	options = parser.parse_args()

	# Init robot
	_LOGGER.info("Authenticating Spot...")
	sdk = create_standard_sdk('SENSEable_XboxClient')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)
	robot.start_time_sync()

	# Launch driver
	_LOGGER.info("Launching robot driver...")
	robot_driver = RobotDriver(robot)
	try:
		robot_driver.start()
	except (ResponseError, RpcError) as err:
		_LOGGER.error('Failed to initialize robot communication: %s', err)
		return

	# Get controller information
	_LOGGER.info("Retrieving device information...")
	devices = []
	should_use_bluetooth = options.use_bluetooth.lower() == "true"
	bluetooth_controller_connected = False
	# Wireless controller
	try:
		if should_use_bluetooth:
			_LOGGER.info("Bluetooth use requested.")
			if is_bluetooth_dongle_connected():
				_LOGGER.info("Bluetooth dongle found.")
				if not bluetooth_controller_already_connected(): # Find the controller and connect with it
					search_count = 0 # For reference
					while bluetooth_controller_connected == False:
						search_count = search_count + 1
						_LOGGER.info(f"Starting bluetooth device search #{search_count}")
						bluetooth_controller_connected = attempt_bluetooth_connection()
					_LOGGER.info("Bluetooth controller connected status: " + str(bluetooth_controller_connected))
	except Exception as e:
		_LOGGER.info(e) # No need to return, just run without bluetooth
	# Wired controller
	try:
		devices, device_type = get_controller() 
		if devices == []:
			if bluetooth_controller_connected: # Has to restart for some reason for connected dev/input/event* to appear. Seems to hold connection after restart for some reason, which is useful but confusing. 
				raise Exception("Restarting container to find valid /dev/input/event* path...")
			else:
				raise Exception("Suitable device unable to be found")
		_LOGGER.info("Using device(s): " + str(devices))
	except Exception as e:
		'''
		while True: # TEMP: DELETE THIS
			_LOGGER.info("Debug: hanging in device not found exception...")
			time.sleep(1.0)
		'''
		_LOGGER.info(e)
		_LOGGER.info("Exiting...")
		return 

	# Init controller value object
	_LOGGER.info("Initializing input controller...")
	controller = XboxController(device_type)

	# Issue starting commands with pauses in between to allow for completion
	robot_driver._toggle_estop()
	give_time_for_action("e-stop", 2)
	robot_driver._toggle_power()
	give_time_for_action("power-on", 6)
	robot_driver._stand()
	give_time_for_action("stand", 2)

	# Launch input threads
	_LOGGER.info("Launching controls...")
	for device in devices:
		_LOGGER.info("Launching thread for device with path: " + device.path)
		controller.launch_reading_thread(device)
	try:
		while reading_active:
			# Debug input status
			#_LOGGER.info("Buttons:")
			#controller.print_buttons()
			#_LOGGER.info("Axes:")
			#controller.print_axes()

			# Execute button commands
			if controller.A.should_process_command():
				_LOGGER.info("A")
				controller.A.command_processed()
				robot_driver._stand()
			if controller.B.should_process_command():
				_LOGGER.info("B")
				controller.B.command_processed()
				robot_driver._sit()
			if controller.X.should_process_command():
				_LOGGER.info("X")
				controller.X.command_processed()
				robot_driver._toggle_power()
			if controller.Y.should_process_command():
				_LOGGER.info("Y")
				controller.Y.command_processed()
				robot_driver._toggle_lease()
			if controller.LeftBumper.should_process_command():
				_LOGGER.info("LeftBumper")
				controller.LeftBumper.command_processed()
				robot_driver._self_right()
			if controller.RightBumper.should_process_command():
				_LOGGER.info("RightBumper")
				controller.RightBumper.command_processed()
				robot_driver._toggle_estop()
			if controller.Start.should_process_command():
				_LOGGER.info("Start")
				controller.Start.command_processed()
				robot_driver.toggle_standing_mode()
			if controller.Back.should_process_command():
				_LOGGER.info("Back")
				controller.Back.command_processed()
				robot_driver.display_robot_status()
			if controller.Xbox.should_process_command():
				_LOGGER.info("Xbox")
				controller.Xbox.command_processed()
				#trigger_recording_service(robot)
				pass # Disabling record transform service triggering from study
			if controller.LeftThumb.should_process_command():
				_LOGGER.info("LeftThumb")
				controller.LeftThumb.command_processed()
				pass # TODO assign input
			if controller.RightThumb.should_process_command():
				_LOGGER.info("RightThumb")
				controller.RightThumb.command_processed()
				pass # TODO assign input
			if controller.LeftDPad.should_process_command():
				_LOGGER.info("LeftDPad")
				controller.LeftDPad.command_processed()
				pass # TODO assign input
			if controller.RightDPad.should_process_command():
				_LOGGER.info("RightDPad")
				controller.RightDPad.command_processed()
				pass # TODO assign input
			if controller.UpDPad.should_process_command():
				_LOGGER.info("UpDPad")
				controller.UpDPad.command_processed()
				pass # TODO assign input
			if controller.DownDPad.should_process_command():
				_LOGGER.info("DownDPad")
				controller.DownDPad.command_processed()
				pass # TODO assign input

			# Axes commands should make spot walk around among spot geometry frames - https://dev.bostondynamics.com/docs/concepts/geometry_and_frames
			if not robot_driver.standing_mode:
				# Movement values
				v_x = 0
				v_y = 0
				v_rot = 0
				# Execute axis commands
				if controller.LeftJoystickY > 0 and controller.LeftJoystickY >= STICK_THRESHOLD:
					#robot_driver._move_forward()
					#v_x = 1.0
					v_x = controller.LeftJoystickY
				elif controller.LeftJoystickY < 0 and controller.LeftJoystickY <= -STICK_THRESHOLD:
					#robot_driver._move_backward()
					#v_x = -1.0
					v_x = controller.LeftJoystickY
				if controller.LeftJoystickX > 0 and controller.LeftJoystickX >= STICK_THRESHOLD:
					#robot_driver._strafe_right()
					#v_y = -1.0
					v_y = -controller.LeftJoystickX
				elif controller.LeftJoystickX < 0 and controller.LeftJoystickX <= -STICK_THRESHOLD:
					#robot_driver._strafe_left()
					#v_y = 1.0
					v_y = -controller.LeftJoystickX
				if controller.RightJoystickX > 0 and controller.RightJoystickX >= STICK_THRESHOLD:
					#robot_driver._turn_right()
					#v_rot = -1.0
					v_rot = -controller.RightJoystickX
				elif controller.RightJoystickX < 0 and controller.RightJoystickX <= -STICK_THRESHOLD:
					#robot_driver._turn_left()
					#v_rot = 1.0
					v_rot = -controller.RightJoystickX
				# Apply movement
				if abs(v_x) > 0 or abs(v_y) > 0 or abs(v_rot) > 0:
					v_x = v_x * VELOCITY_BASE_SPEED
					v_y = v_y * VELOCITY_STRAFE_SPEED
					v_rot = v_rot * VELOCITY_BASE_ANGULAR
					robot_driver._velocity_cmd_helper(desc='move', v_x=v_x, v_y=v_y, v_rot=v_rot)
					'''
					self._start_robot_command(
						"move",
						RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
						params = self.mobility_params,
						end_time_secs=time.time() + VELOCITY_CMD_DURATION
						)
					'''
			# Axes commands should make spot contort
			else:
				# Stance values
				yaw = 0
				pitch = 0
				roll = 0
				height = 0
				# Execute axis commands
				#if controller.LeftJoystickY > 0 and controller.LeftJoystickY >= STICK_THRESHOLD:
				if controller.LeftJoystickY > 0:
					height = controller.LeftJoystickY * HEIGHT_MULTIPLIER
				#elif controller.LeftJoystickY < 0 and controller.LeftJoystickY <= -STICK_THRESHOLD:
				elif controller.LeftJoystickY < 0:
					height = controller.LeftJoystickY * HEIGHT_MULTIPLIER
				#if controller.LeftJoystickX > 0 and controller.LeftJoystickX >= STICK_THRESHOLD:
				if controller.LeftJoystickX > 0:
					roll = controller.LeftJoystickX * 0.5
				#elif controller.LeftJoystickX < 0 and controller.LeftJoystickX <= -STICK_THRESHOLD:
				elif controller.LeftJoystickX < 0:
					roll = controller.LeftJoystickX * 0.5
				#if controller.RightJoystickX > 0 and controller.RightJoystickX >= STICK_THRESHOLD:
				if controller.RightJoystickX > 0:
					yaw = -controller.RightJoystickX * 0.5
				#elif controller.RightJoystickX < 0 and controller.RightJoystickX <= -STICK_THRESHOLD:
				elif controller.RightJoystickX < 0:
					yaw = -controller.RightJoystickX * 0.5
				#if controller.RightJoystickY > 0 and controller.RightJoystickY >= STICK_THRESHOLD:
				if controller.RightJoystickY > 0:
					pitch = -controller.RightJoystickY * 0.5
				#elif controller.RightJoystickY < 0 and controller.RightJoystickY <= -STICK_THRESHOLD:
				elif controller.RightJoystickY < 0:
					pitch = -controller.RightJoystickY * 0.5
				# Apply movement
				if abs(yaw) > 0 or abs(pitch) > 0 or abs(roll) > 0 or abs(height) > 0:
					robot_driver.set_stance(yaw, pitch, roll, height)
				else: # Set to default
					robot_driver.set_stance(0, 0, 0, 0)

			# Reset
			time.sleep(1.0 / 30.0)

	except Exception as e:
		_LOGGER.info(e)
		pass
	finally:
		# Shut down transform recording (if it was triggered by this script previously)
		if transform_record_service_active:
			trigger_recording_service(robot)
		# Shut down lease/e-stop check ins
		robot_driver.shutdown()
		# Exit
		_LOGGER.info("Exiting...")
		return 0

if __name__ == '__main__':
	main()
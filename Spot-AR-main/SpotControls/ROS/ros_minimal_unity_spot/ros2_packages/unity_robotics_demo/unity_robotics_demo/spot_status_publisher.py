import argparse
import time
import logging

import bosdyn.client.util
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_state import RobotStateClient
import bosdyn.api.robot_state_pb2 as robot_state_proto
from bosdyn.client.frame_helpers import get_vision_tform_body

_LOGGER = logging.getLogger('status_plugin')

def get_current_status(state_client, lease_client):
	state = state_client.get_robot_state()

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
	else:
		current_power_status = "Changing..."
	#output_msg("Motor Power: " + current_power_status)

	# Lease
	current_lease_owner = "None" 
	leases = lease_client.list_leases_full()
	resources = leases.resources
	for resource in resources:
		resource_name = resource.resource
		owner = resource.lease_owner
		owner_clientname = owner.client_name
		owner_username = owner.user_name
		is_stale = resource.is_stale # Can be taken
		if resource_name == "body":
			current_lease_owner = owner_clientname
		elif resource_name == "mobility":
			pass

	# Battery
	battery_state = state.battery_states[0]
	battery_status = battery_state.Status.Name(battery_state.status)[7:] # Charging status
	battery_percentage = battery_state.charge_percentage.value

	# Communication
	wifi_state = str(state.comms_states[0].wifi_state)[14:-1]

	# Body position
	kinematic_state = state.kinematic_state
	transform_tree = kinematic_state.transforms_snapshot
	vision_transform = get_vision_tform_body(transform_tree)
	vision_position = vision_transform.position # .x, .y, .z - Vec3
	vision_rotation = vision_transform.rotation # .x, .y, .z, and .w
	vision_velocity_linear = kinematic_state.velocity_of_body_in_vision.linear
	vision_velocity_angular = kinematic_state.velocity_of_body_in_vision.angular
	transform_xyz = lambda p : (p.x, p.y, p.z)

	return \
	current_estop, \
	current_power_status, \
	current_lease_owner, \
	battery_status, \
	battery_percentage, \
	wifi_state, \
	transform_xyz(vision_velocity_linear), \
	transform_xyz(vision_velocity_angular)


def main():
	bosdyn.client.util.setup_logging()

	# Get input args
	parser = argparse.ArgumentParser()
	bosdyn.client.util.add_base_arguments(parser)
	options = parser.parse_args()

	# Setup robot
	sdk = create_standard_sdk('SENSEable_StatusClient')
	robot = sdk.create_robot(options.hostname)
	bosdyn.client.util.authenticate(robot)
	robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
	lease_client = robot.ensure_client(LeaseClient.default_service_name)

	while True:
		_LOGGER.info(get_current_status(robot_state_client, lease_client))
		time.sleep(0.1)


if __name__ == '__main__':
    main()
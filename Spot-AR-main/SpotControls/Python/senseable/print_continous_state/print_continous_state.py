# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Simple robot state capture tutorial."""

### SDL: Pass in robot IP
### SDL: Pro tip - If you get lost in trying to figure out the data type being handed back, find it's proto file in spot-sdk/protos/bosdyn

import sys

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient

### SDL
import time
from bosdyn.client.frame_helpers import (get_odom_tform_body, get_vision_tform_body)


def main():
    import argparse

    commands = set(['state', 'hardware', 'metrics'])

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args()

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('RobotStateClient')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    current_frame = 1
    prev_transform = None
    FRAMERATE = 3


    while True:
        # Make a robot state request
        # Field names found in the proto file (spot-sdk/protos/robot_state.proto). This is what is translated to the Python code in the API.
        state = robot_state_client.get_robot_state()

        ## Is powered on (1 = yes)
        power_state = state.power_state.robot_power_state
        #print("Power State:", power_state)

        ## Kinematics data - From the BD source code: "The kinematic state of the robot describes the current estimated positions of the robot body and joints throughout the world."
        kinematic_state = state.kinematic_state
        timestamp_kinematic = kinematic_state.acquisition_timestamp
        print("Timestamp:", timestamp_kinematic.ToDatetime()) # Convert to datetime: https://googleapis.dev/python/protobuf/latest/google/protobuf/timestamp_pb2.html

        transform_tree = kinematic_state.transforms_snapshot
        transform_keys = []
        for key in transform_tree.child_to_parent_edge_map:
            transform_keys.append(key)
        print("Transform Keys:", transform_keys)

        ## Vision frame transform
        '''
        #print(transform_tree.child_to_parent_edge_map)
        #vision_transform = transform_tree.child_to_parent_edge_map["vision"]
        #print("Vision Transform:", vision_transform)
        #vision_parent_name = vision_transform.parent_frame_name
        #vision_pose = vision_transform.parent_tform_child
        #print("Vision Position:\n", vision_pose.position)
        #print("Vision Rotation:\n", vision_pose.rotation)
        '''
        vision_transform = get_vision_tform_body(transform_tree)
        print("Vision Position:\n", vision_transform.position)
        print("Vision Rotation:\n", vision_transform.rotation)

        '''
        ## Framerate check
        if prev_transform == vision_transform:
            print("Duplicate at frame:", current_frame)
        prev_transform = vision_transform
        '''
        
        ## Velocity frame velocity
        '''
        velocity_linear = kinematic_state.velocity_of_body_in_vision.linear
        print("Vision Linear Velocity:\n", velocity_linear)
        velocity_angular = kinematic_state.velocity_of_body_in_vision.angular
        print("Vision Angular Velocity:\n", velocity_angular)
        '''

        '''
        ## Joints
        joint_states = kinematic_state.joint_states
        for joint_state in joint_states:
            print(joint_state.name)
        '''


        print("===========")
        print(transform_tree.child_to_parent_edge_map["vision"])
        print(get_vision_tform_body(transform_tree))
        print(type(get_vision_tform_body(transform_tree).position))

        # Repeat with a small lapse between
        current_frame = current_frame + 1
        time.sleep(1.0 / FRAMERATE)
        #time.sleep(0.5)


if __name__ == '__main__':
    if not main():
        sys.exit(1)

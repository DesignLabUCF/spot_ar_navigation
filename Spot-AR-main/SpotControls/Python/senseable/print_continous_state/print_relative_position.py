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
from bosdyn.api import geometry_pb2
from bosdyn.api import world_object_pb2
from bosdyn.client.frame_helpers import (get_odom_tform_body, get_vision_tform_body, get_a_tform_b, add_edge_to_tree, get_frame_names)
from bosdyn.util import now_timestamp
from bosdyn.client.world_object import WorldObjectClient, make_add_world_object_req

APRIL_TAG_ID = 4


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
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name) # For detecting april tags

    FRAMERATE = 3
    most_recent_april_position = None
    most_recent_april_rotation = None


    while True:
        # Make a robot state request
        state = robot_state_client.get_robot_state()

        # Pull kinematics data - From the BD source code: "The kinematic state of the robot describes the current estimated positions of the robot body and joints throughout the world."
        kinematic_state = state.kinematic_state
        timestamp_kinematic = kinematic_state.acquisition_timestamp
        print("Timestamp:", timestamp_kinematic.ToDatetime()) # Convert to datetime: https://googleapis.dev/python/protobuf/latest/google/protobuf/timestamp_pb2.html

        # Get robot position
        robot_transform_tree = kinematic_state.transforms_snapshot
        vision_transform = get_vision_tform_body(robot_transform_tree)
        vision_position = vision_transform.position
        vision_rotation = vision_transform.rotation

        # An alternativer approach:
        # Add april tag existing frame
        # https://dev.bostondynamics.com/docs/concepts/geometry_and_frames#adding-frames-during-robot-operation
        '''
        time = now_timestamp()
        frame_tree_edges  = {}
        april_frame = SE3Pose(position=Vec3(1, 1, 1), rotation=Quaternion(w=1, x=0, y=0, z=0))
        frame_tree_edges = add_edge_to_tree(frame_tree_edges, april_frame, "vision", "april_frame")
        snapshot = FrameTreeSnapshot(child_to_parent_edge_map=frame_tree_edges)
        world_obj_frame = WorldObject(id=21, name="AprilFrame", transforms_snapshot=snapshot, acquisition_time=time.now())
        world_object_client.mutate_world_objects(mutation_req= make_add_world_object_request(world_obj_frame))
        '''

        # Find fiducual in world objects detected list
        world_objects = world_object_client.list_world_objects().world_objects
        for obj in world_objects:
            if obj.name == ("world_obj_apriltag_" + str(APRIL_TAG_ID)):
                #print("FOUND")
                april_tree = obj.transforms_snapshot
                print(april_tree)
                april_key = "filtered_fiducial_" + str(APRIL_TAG_ID)

                #april_key = "fiducual_" + str(APRIL_TAG_ID)
                april_from_vision = get_a_tform_b(april_tree, "vision", april_key)
                #print(april_from_vision)
                #print(get_frame_names(april_tree))
                #print(april_from_vision)

                most_recent_april_position = april_from_vision.position
                most_recent_april_rotation = april_from_vision.rotation

        ''' # NOTE: Difficult to do in pre-processing, so moving it to the rendered environment (Unity) for now!
        # Determine relative position to april tag
        relative_position = vision_position
        relative_rotation = vision_rotation
        if (most_recent_april_position != None) and (most_recent_april_rotation != None):
            #geom.Vec3(x=.2, y=.2, z=.2)
            #geom.Quaternion(x=.1, y=.1, z=.1, w=.1)
            # Position translation
            relative_position = geometry_pb2.Vec3( \
                x=vision_position.x - most_recent_april_position.x,
                y=vision_position.y - most_recent_april_position.y,
                z=vision_position.z - most_recent_april_position.z \
                )
            # Quaternion difference
            april_quaternion_inverse = geom.Quaternion( # Invert quaternion - http://wiki.ros.org/tf2/Tutorials/Quaternions#Relative_rotations
                x=most_recent_april_rotation.x, \
                y=most_recent_april_rotation.y, \
                z=most_recent_april_rotation.z, \
                w=most_recent_april_rotation.w)
            quaternion_difference =  
            relative_rotation = vision_rotation - most_recent_april_rotation
        '''

        # Print relative position
        print("Vision:")
        print(vision_position)
        print("April:")
        print(most_recent_april_position)
        #print("Relative:")
        #print(relative_position)
        print("=========================")


        # Repeat with a small lapse between
        time.sleep(1.0 / FRAMERATE)


if __name__ == '__main__':
    if not main():
        sys.exit(1)

#!/usr/bin/env python

import random
import rclpy

from rclpy.node import Node

from unity_robotics_demo_msgs.msg import SpotTransform
from unity_robotics_demo_msgs.msg import TagAndCameraTransform


class TransformPublisher(Node):

	def __init__(self):
		super().__init__('transform_publisher')
		self.publisher_ = self.create_publisher(TagAndCameraTransform, 'spot_tagandcamera', 50)
		timer_period = 0.25  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0	
		
	def timer_callback(self):
		# Position of QR/Spot derived from generated file 09-22-2023_20-09-03-435.csv - Timestamp 09-22-2023_20-08-59-367 (csv row 348)
		# Should show spot looking at April tag from slightly off to the right of it, while stanced curiously

		# April Tag
		april_transform = SpotTransform()
		april_transform.pos_x = float(1.1567734423208)
		april_transform.pos_y = float(3.80488406977026)
		april_transform.pos_z = float(0.165363408697202)
		april_transform.rot_x = float(0.523184096209792)
		april_transform.rot_y = float(0.481790601834137)
		april_transform.rot_z = float(0.507527513584768)
		april_transform.rot_w = float(-0.486386718992104)
		# Camera
		camera_transform = SpotTransform()
		camera_transform.pos_x = float(0.568697200473663)
		camera_transform.pos_y = float(5.38291646688248)
		camera_transform.pos_z = float(0.108286363856185)
		camera_transform.rot_x = float(0.115032859146595)
		camera_transform.rot_y = float(-0.257193714380264)
		camera_transform.rot_z = float(-0.495367795228958)
		camera_transform.rot_w = float(0.821723401546478)	
		# Combined
		transform = TagAndCameraTransform()
		transform.tag = april_transform
		transform.camera = camera_transform
		# Publish message
		self.get_logger().info(f'Publishing: {transform}')
		self.publisher_.publish(transform)	
		self.i = self.i + 1

def main(args=None):
	rclpy.init(args=args)

	pub = TransformPublisher()

	rclpy.spin(pub)


if __name__ == '__main__':
	main()

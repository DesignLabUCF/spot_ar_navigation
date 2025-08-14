#!/usr/bin/env python

import random
import rclpy

from rclpy.node import Node

from unity_robotics_demo_msgs.msg import SpotTransform


class TransformPublisher(Node):

	def __init__(self):
		super().__init__('transform_publisher')
		self.publisher_ = self.create_publisher(SpotTransform, 'transform', 50)
		timer_period = 0.25  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0	
		
	def timer_callback(self):
		transform = SpotTransform()
		transform.pos_x = float(0)
		transform.pos_y = float(0)
		transform.pos_z = float(0)
		transform.rot_x = float(self.i % 360)
		transform.rot_y = float(0)
		transform.rot_z = float(0)
		transform.rot_w = float(0)
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

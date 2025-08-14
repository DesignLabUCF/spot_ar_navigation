#!/usr/bin/env python

import random
import rclpy

from rclpy.node import Node

from unity_robotics_demo_msgs.msg import SpotVelocity


class VelocityPublisher(Node):

	def __init__(self):
		super().__init__('velocity_publisher')
		self.publisher_ = self.create_publisher(SpotVelocity, 'velocity', 50)
		timer_period = 0.25  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

	def get_velocity(self):
		velocity = SpotVelocity()
		velocity.command_type = "velocity"
		velocity.v_x = 0.0
		velocity.v_y = 0.0
		velocity.v_rot = 0.0
		return velocity	
		
	def timer_callback(self):
		# Create message
		velocity = self.get_velocity()
		# Publish message
		self.get_logger().info(f'Publishing: {velocity}')
		self.publisher_.publish(velocity)	
		self.i = self.i + 1


def main(args=None):
	rclpy.init(args=args)

	velocity_pub = VelocityPublisher()

	rclpy.spin(velocity_pub)


if __name__ == '__main__':
	main()

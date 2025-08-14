#!/usr/bin/env python

import random
import rclpy

from rclpy.node import Node

from unity_robotics_demo_msgs.msg import HelloTest


class HelloPublisher(Node):

	def __init__(self):
		super().__init__('hello_publisher')
		self.publisher_ = self.create_publisher(HelloTest, 'hello', 50)
		timer_period = 0.25  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		
	def timer_callback(self):
		# Create message
		hello = HelloTest()
		hello.hello_message = "Hello HoloLens!"
		# Publish message
		self.get_logger().info(f'Publishing: {hello}')
		self.publisher_.publish(hello)	
		self.i = self.i + 1


def main(args=None):
	rclpy.init(args=args)

	hello_pub = HelloPublisher()

	rclpy.spin(hello_pub)


if __name__ == '__main__':
	main()

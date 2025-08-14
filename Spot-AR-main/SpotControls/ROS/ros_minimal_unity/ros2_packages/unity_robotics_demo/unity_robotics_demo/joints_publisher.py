#!/usr/bin/env python

import random
import rclpy

from rclpy.node import Node

from unity_robotics_demo_msgs.msg import SpotJoint


class JointsPublisher(Node):

	def __init__(self):
		super().__init__('joints_publisher')
		self.publisher_ = self.create_publisher(SpotJoint, 'joints', 50)
		timer_period = 0.25  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

	def publish_joint(self, name, position, velocity, acceleration, load):
		# Create message
		joint = SpotJoint()
		joint.name = name
		joint.position = position
		joint.velocity = velocity
		joint.acceleration = acceleration
		joint.load = load
		# Publish message
		self.get_logger().info(f'Publishing: {joint}')
		self.publisher_.publish(joint)		
		
	def timer_callback(self):
		self.publish_joint("fl.hx", random.uniform(-30.0, 30.0), random.random(), random.random(), random.random())
		self.publish_joint("fl.hy", random.uniform(0, 90.0), random.random(), random.random(), random.random())
		self.publish_joint("fl.kn", random.uniform(-90.0, 0), random.random(), random.random(), random.random())
		self.publish_joint("fr.hx", random.uniform(-30.0, 30.0), random.random(), random.random(), random.random())
		self.publish_joint("fr.hy", random.uniform(0, 90.0), random.random(), random.random(), random.random())
		self.publish_joint("fr.kn", random.uniform(-90.0, 0), random.random(), random.random(), random.random())
		self.publish_joint("hl.hx", random.uniform(-30.0, 30.0), random.random(), random.random(), random.random())
		self.publish_joint("hl.hy", random.uniform(0, 90.0), random.random(), random.random(), random.random())
		self.publish_joint("hl.kn", random.uniform(-90.0, 0), random.random(), random.random(), random.random())
		self.publish_joint("hr.hx", random.uniform(-30.0, 30.0), random.random(), random.random(), random.random())
		self.publish_joint("hr.hy", random.uniform(0, 90.0), random.random(), random.random(), random.random())
		self.publish_joint("hr.kn", random.uniform(-90.0, 0), random.random(), random.random(), random.random())
		self.i = self.i + 1

def main(args=None):
	rclpy.init(args=args)

	joints_pub = JointsPublisher()

	rclpy.spin(joints_pub)


if __name__ == '__main__':
	main()

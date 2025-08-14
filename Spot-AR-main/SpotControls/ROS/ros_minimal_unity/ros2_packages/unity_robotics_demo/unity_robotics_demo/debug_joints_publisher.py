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
		# Position of joints derived from generated file 09-22-2023_20-09-03-435.csv - Timestamp 09-22-2023_20-08-59-367 (csv row 348)
		# Should show spot looking at April tag from slightly off to the right of it, while stanced curiously

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
		self.publish_joint("fl.hx", -0.449739307165145, random.random(), random.random(), random.random())
		self.publish_joint("fl.hy", 0.728142380714416, random.random(), random.random(), random.random())
		self.publish_joint("fl.kn", -0.734104871749877, random.random(), random.random(), random.random())
		self.publish_joint("fr.hx", -0.511620163917541, random.random(), random.random(), random.random())
		self.publish_joint("fr.hy", 0.925321877002716, random.random(), random.random(), random.random())
		self.publish_joint("fr.kn", -1.11819565296173, random.random(), random.random(), random.random())
		self.publish_joint("hl.hx", -0.470503091812133, random.random(), random.random(), random.random())
		self.publish_joint("hl.hy", 1.36095345020294, random.random(), random.random(), random.random())
		self.publish_joint("hl.kn", -1.77996480464935, random.random(), random.random(), random.random())
		self.publish_joint("hr.hx", -0.559718489646911, random.random(), random.random(), random.random())
		self.publish_joint("hr.hy", 1.45653843879699, random.random(), random.random(), random.random())
		self.publish_joint("hr.kn", -1.9454401731491, random.random(), random.random(), random.random())
		self.i = self.i + 1

def main(args=None):
	rclpy.init(args=args)

	joints_pub = JointsPublisher()

	rclpy.spin(joints_pub)


if __name__ == '__main__':
	main()

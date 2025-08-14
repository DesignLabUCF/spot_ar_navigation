#!/usr/bin/env python

import random
import rclpy

from rclpy.node import Node

from unity_robotics_demo_msgs.msg import UnityColor


class ColorPublisher(Node):

    def __init__(self):
        super().__init__('color_publisher_repeated')
        self.publisher_ = self.create_publisher(UnityColor, 'color', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        #self.do_publish()

    def generate_color(self):
        color = UnityColor()
        color.r = random.randint(0, 255)
        color.g = random.randint(0, 255)
        color.b = random.randint(0, 255)
        color.a = 1
        return color 

    def publish_color(self):
        #if self.i == 0:
        color = self.generate_color()
        self.get_logger().info(f'Publishing: {color}')
        self.publisher_.publish(color)
            
        self.i += 1
        
    def timer_callback(self):
        self.publish_color()     


def main(args=None):
    rclpy.init(args=args)

    color_pub = ColorPublisher()

    '''
    while rclpy.ok():
        rclpy.spin_once(color_pub)
    '''

    rclpy.spin(color_pub)

    #color_pub.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()

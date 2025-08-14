import rclpy
from rclpy.node import Node

from unity_robotics_demo_msgs.msg import SpotVelocity

class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            SpotVelocity,
            "velocity",
            self.listener_callback,
            50)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("v_x:")
        self.get_logger().info(str(msg.v_x))
        self.get_logger().info("v_y:")
        self.get_logger().info(str(msg.v_y))
        self.get_logger().info("v_rot:")
        self.get_logger().info(str(msg.v_rot))
        self.get_logger().info("--------------------------")



def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    rclpy.spin(velocity_subscriber)


if __name__ == '__main__':
    main()
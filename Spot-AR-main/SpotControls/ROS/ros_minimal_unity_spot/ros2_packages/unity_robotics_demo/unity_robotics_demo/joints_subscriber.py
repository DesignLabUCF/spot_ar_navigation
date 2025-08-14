import rclpy
from rclpy.node import Node

from unity_robotics_demo_msgs.msg import SpotJoints

class JointsSubscriber(Node):

    def __init__(self):
        super().__init__('joints_subscriber')
        self.subscription = self.create_subscription(
            SpotJoints,
            "joints",
            self.listener_callback,
            50)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(str(msg))
        self.get_logger().info("--------------------------")



def main(args=None):
    rclpy.init(args=args)

    subscriber = JointsSubscriber()

    rclpy.spin(subscriber)


if __name__ == '__main__':
    main()
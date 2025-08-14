import rclpy
from rclpy.node import Node

from unity_robotics_demo_msgs.msg import HelloTest

class HelloSubscriber(Node):

    def __init__(self):
        super().__init__('hello_subscriber')
        self.subscription = self.create_subscription(
            HelloTest,
            "hello",
            self.listener_callback,
            50)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(str(msg))
        self.get_logger().info("--------------------------")



def main(args=None):
    rclpy.init(args=args)

    subscriber = HelloSubscriber()

    rclpy.spin(subscriber)


if __name__ == '__main__':
    main()
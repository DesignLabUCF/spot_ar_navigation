import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from unity_robotics_demo_msgs.msg import TagAndCameraTransform

class HoloLensSubscriber(Node):

    def __init__(self):
        super().__init__('hololens_subscriber')
        self.subscription = self.create_subscription(
            TagAndCameraTransform,
            "hololens_tagandcamera",
            self.listener_callback,
            50)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("Camera:")
        self.get_logger().info(msg.camera)
        self.get_logger().info("Tag:")
        self.get_logger().info(msg.tag)
        self.get_logger().info("--------------------------")



def main(args=None):
    rclpy.init(args=args)

    hl_subscriber = HoloLensSubscriber()

    rclpy.spin(hl_subscriber)


if __name__ == '__main__':
    main()
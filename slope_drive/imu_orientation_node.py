import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuOrientationNode(Node):
    def __init__(self):
        super().__init__('imu_orientation_node')
        self.get_logger().info('IMU Orientation Node has been started.')

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/camera/imu',
            self.imu_callback,
            10)

        self.orientation_publisher = self.create_publisher(
            Imu,
            '/imu/orientation',
            10)

    def imu_callback(self, msg):
        # Simply relaying the message for now
        self.orientation_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOrientationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
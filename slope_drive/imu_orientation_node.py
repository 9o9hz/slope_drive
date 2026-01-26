import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuOrientationNode(Node):
    def __init__(self):
        super().__init__('imu_orientation_node')
        self.get_logger().info('IMU Orientation Node has been started.')

        # Subscriber to the IMU data from the sensor
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/camera/imu',  # Topic from realsense2_camera
            self.imu_callback,
            10)

        # Publisher for IMU data with orientation
        self.orientation_publisher = self.create_publisher(
            Imu,
            '/imu/orientation',
            10)

    def imu_callback(self, msg):
        # Here you would process the IMU data and calculate orientation
        # For now, we just log and republish it as a placeholder
        self.get_logger().info(f'Received IMU data: {msg.header.stamp}')
        
        # In a real implementation, you would use a filter (e.g., Madgwick)
        # to calculate orientation (msg.orientation) from linear_acceleration and angular_velocity
        
        # Placeholder: just forward the message
        processed_imu_msg = msg
        
        self.orientation_publisher.publish(processed_imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOrientationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Float64MultiArray
import cv2

class BevProjectionNode(Node):
    def __init__(self):
        super().__init__('bev_projection_node')
        self.get_logger().info('BEV Projection Node has been started.')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.cam_info_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/orientation', self.imu_callback, 10)

        # Publishers
        self.bev_image_pub = self.create_publisher(Image, '/bev/image', 10)
        self.homography_pub = self.create_publisher(Float64MultiArray, '/bev/H', 10)
        self.debug_image_pub = self.create_publisher(Image, '/bev/debug_overlay', 10)

        self.camera_matrix = None
        self.dist_coeffs = None

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('Camera info not received yet. Skipping image processing.')
            return
            
        self.get_logger().info(f'Received Image: {msg.header.stamp}')
        # Placeholder for BEV projection logic
        # You will use self.camera_matrix, self.dist_coeffs and IMU data
        # to calculate the homography matrix H(t) and warp the input image.

        # For now, let's just create a dummy black image as BEV
        dummy_bev_image = cv2.cvtColor(cv2.UMat(200, 200, cv2.CV_8UC1, 0), cv2.COLOR_GRAY2BGR)
        # In a real scenario, you'd publish the warped image
        # self.bev_image_pub.publish(bev_image_msg)

    def cam_info_callback(self, msg):
        self.get_logger().info('Received camera info.')
        self.camera_matrix = msg.k.reshape((3, 3))
        self.dist_coeffs = msg.d
        # Unregister subscription to save resources as camera info is static
        self.destroy_subscription(self.cam_info_sub)


    def imu_callback(self, msg):
        # This is where you would update the dynamic part of your homography matrix
        # based on the vehicle's orientation (roll, pitch).
        self.get_logger().info(f'Received IMU Orientation: {msg.header.stamp}')

def main(args=None):
    rclpy.init(args=args)
    node = BevProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

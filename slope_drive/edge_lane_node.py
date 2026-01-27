import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class MaskGenerationNode(Node):
    def __init__(self):
        super().__init__('mask_generation_node')
        self.get_logger().info('Mask Generation Node has been started.')

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber to the BEV image
        self.bev_image_sub = self.create_subscription(
            Image,
            '/bev/image',
            self.bev_image_callback,
            10)

        # Publisher for the drivable area mask
        self.mask_pub = self.create_publisher(Image, '/bev/mask', 10)

    def bev_image_callback(self, msg):
        # self.get_logger().info(f'Received BEV Image: {msg.header.stamp}')
        
        try:
            bev_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # 1. Drivable area mask generation
        gray_image = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        # This threshold will likely need tuning.
        _, mask = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

        # 2. Mask refinement (morphology)
        kernel = np.ones((5,5), np.uint8)
        mask_morphed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Publish the final mask
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_morphed, "mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert and publish mask image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MaskGenerationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class EdgeLaneNode(Node):
    def __init__(self):
        super().__init__('edge_lane_node')
        self.get_logger().info('Edge Lane Node has been started.')

        self.bridge = CvBridge()

        # Subscriber: BEV Image
        self.bev_image_sub = self.create_subscription(
            Image,
            '/bev/image',
            self.bev_image_callback,
            10)

        # Publisher: Mask Image
        self.mask_pub = self.create_publisher(Image, '/bev/mask', 10)

    def bev_image_callback(self, msg):
        try:
            bev_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # 1. Grayscale & Thresholding (도로 색상에 맞춰 200 값 조정 필요)
        gray_image = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

        # 2. Morphology (노이즈 제거)
        kernel = np.ones((5,5), np.uint8)
        mask_morphed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_morphed, "mono8")
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Publish error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EdgeLaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
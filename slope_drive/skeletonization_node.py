import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SkeletonizationNode(Node):
    def __init__(self):
        super().__init__('skeletonization_node')
        self.subscription = self.create_subscription(
            Image,
            'image_mask',  # Input topic from edge_lane_node
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, 'skeleton_image', 10) # Output topic
        self.bridge = CvBridge()
        self.get_logger().info('Skeletonization node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image mask')
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')

        # Skeletonization
        skeleton = cv2.ximgproc.thinning(cv_image, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

        # Publish the skeleton image
        skeleton_msg = self.bridge.cv2_to_imgmsg(skeleton, 'mono8')
        skeleton_msg.header = msg.header
        self.publisher.publish(skeleton_msg)
        self.get_logger().info('Publishing skeleton image')


def main(args=None):
    rclpy.init(args=args)
    skeletonization_node = SkeletonizationNode()
    rclpy.spin(skeletonization_node)
    skeletonization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

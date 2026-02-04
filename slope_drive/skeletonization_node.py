import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.ximgproc

class SkeletonizationNode(Node):
    def __init__(self):
        super().__init__('skeletonization_node')
        self.bridge = CvBridge()
        
        # Subscribe to the mask from EdgeLaneNode
        self.subscription = self.create_subscription(
            Image,
            '/bev/mask',  # Topic matched!
            self.listener_callback,
            10)
            
        self.publisher = self.create_publisher(Image, 'skeleton_image', 10)
        self.get_logger().info('Skeletonization node started.')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            
            # Skeletonization (Zhang-Suen Thinning)
            skeleton = cv2.ximgproc.thinning(cv_image, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)

            skeleton_msg = self.bridge.cv2_to_imgmsg(skeleton, 'mono8')
            skeleton_msg.header = msg.header
            self.publisher.publish(skeleton_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing skeleton: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SkeletonizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
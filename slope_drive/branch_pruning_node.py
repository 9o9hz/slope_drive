import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from slope_drive.utils.pruning import prune_branches

class BranchPruningNode(Node):
    def __init__(self):
        super().__init__('branch_pruning_node')
        
        self.declare_parameter('length_threshold', 10)
        self.length_threshold = self.get_parameter('length_threshold').value
        
        self.subscription = self.create_subscription(
            Image,
            'skeleton_image',
            self.listener_callback,
            10)
            
        self.publisher = self.create_publisher(Image, 'pruned_skeleton', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Branch pruning node started.')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            
            # Update param dynamically
            self.length_threshold = self.get_parameter('length_threshold').value

            pruned_image = prune_branches(cv_image, self.length_threshold)

            pruned_msg = self.bridge.cv2_to_imgmsg(pruned_image, 'mono8')
            pruned_msg.header = msg.header
            self.publisher.publish(pruned_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error pruning branches: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BranchPruningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
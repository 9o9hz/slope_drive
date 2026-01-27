import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from slope_drive.utils.pruning import prune_branches # Import from the new utils package

class BranchPruningNode(Node):
    def __init__(self):
        super().__init__('branch_pruning_node')
        
        # Declare a parameter for the pruning length threshold
        self.declare_parameter('length_threshold', 10) # Default value is 10 pixels
        self.length_threshold = self.get_parameter('length_threshold').value
        
        self.subscription = self.create_subscription(
            Image,
            'skeleton_image',  # Input topic from skeletonization_node
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, 'pruned_skeleton', 10) # Output topic
        self.bridge = CvBridge()
        self.get_logger().info(f'Branch pruning node has been started with length_threshold={self.length_threshold}.')

    def listener_callback(self, msg):
        self.get_logger().info('Receiving skeleton image')
        # Convert ROS Image message to OpenCV image (mono8)
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')

        # Get the latest parameter value in case it was changed dynamically
        self.length_threshold = self.get_parameter('length_threshold').value

        # Call the utility function to perform pruning
        pruned_image = prune_branches(cv_image, self.length_threshold)
        self.get_logger().info(f'Pruning branches with threshold: {self.length_threshold}')

        # Publish the pruned skeleton image
        pruned_msg = self.bridge.cv2_to_imgmsg(pruned_image, 'mono8')
        pruned_msg.header = msg.header
        self.publisher.publish(pruned_msg)
        self.get_logger().info('Publishing pruned skeleton image')

def main(args=None):
    rclpy.init(args=args)
    branch_pruning_node = BranchPruningNode()
    rclpy.spin(branch_pruning_node)
    branch_pruning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

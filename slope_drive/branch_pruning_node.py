import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BranchPruningNode(Node):
    def __init__(self):
        super().__init__('branch_pruning_node')
        self.subscription = self.create_subscription(
            Image,
            'skeleton_image',  # Input topic from skeletonization_node
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, 'pruned_skeleton', 10) # Output topic
        self.bridge = CvBridge()
        self.get_logger().info('Branch pruning node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info('Receiving skeleton image')
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')

        # Branch pruning logic goes here
        # This is a placeholder. A real implementation would be more complex.
        # For example, one could find all endpoints and branch points,
        # and remove short branches.
        pruned_image = self.prune_branches(cv_image)

        # Publish the pruned skeleton image
        pruned_msg = self.bridge.cv2_to_imgmsg(pruned_image, 'mono8')
        pruned_msg.header = msg.header
        self.publisher.publish(pruned_msg)
        self.get_logger().info('Publishing pruned skeleton image')

    def prune_branches(self, skeleton_image):
        # Placeholder function for branch pruning.
        # A simple approach could be to apply morphological opening
        # to remove small spurs, but a more robust method is needed for real pruning.
        self.get_logger().info('Pruning branches (placeholder)...')
        # Creating a copy to avoid modifying the original image in place
        pruned_image = skeleton_image.copy()
        # This is just an example, the actual logic will be more involved
        # For instance, identifying branch points and end points,
        # then removing branches shorter than a certain length.
        return pruned_image

def main(args=None):
    rclpy.init(args=args)
    branch_pruning_node = BranchPruningNode()
    rclpy.spin(branch_pruning_node)
    branch_pruning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

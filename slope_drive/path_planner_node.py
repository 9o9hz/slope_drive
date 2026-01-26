import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.get_logger().info('Path Planner Node has been started.')

        # Subscriber to the edge mask from edge_lane_node
        self.edge_mask_sub = self.create_subscription(
            Image,
            '/bev/edges',
            self.edge_mask_callback,
            10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.target_point_pub = self.create_publisher(PointStamped, '/target_point', 10)
        self.steering_cmd_pub = self.create_publisher(Float32, '/steering_cmd', 10)

    def edge_mask_callback(self, msg):
        self.get_logger().info(f'Received Edge Mask: {msg.header.stamp}')

        # Placeholder for path planning logic
        # - Find lane center from the edge mask
        # - Create a nav_msgs/Path
        # - Determine a target point (e.g., lookahead point)
        # - Calculate a steering command based on the target

        # For now, just create and publish dummy messages
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map" # Or your base_link, odom etc.
        self.path_pub.publish(path_msg)
        
        target_point_msg = PointStamped()
        target_point_msg.header = path_msg.header
        target_point_msg.point.x = 1.0 # Dummy target
        self.target_point_pub.publish(target_point_msg)

        steering_msg = Float32()
        steering_msg.data = 0.0 # Dummy command
        self.steering_cmd_pub.publish(steering_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

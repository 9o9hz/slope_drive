import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.get_logger().info('Path Planner Node has been started.')

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('lookahead_distance_pixels', 50)
        self.declare_parameter('steering_gain', 0.005)
        self.declare_parameter('max_speed', 0.2)

        self.lookahead_dist = self.get_parameter('lookahead_distance_pixels').value
        self.k_p = self.get_parameter('steering_gain').value
        self.max_speed = self.get_parameter('max_speed').value

        # Subscriber (Pruned Skeleton)
        self.edge_mask_sub = self.create_subscription(
            Image,
            'pruned_skeleton',
            self.skeleton_callback,
            10)

        # Publishers
        self.target_pub = self.create_publisher(PointStamped, '/planning/target_point', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steering_debug_pub = self.create_publisher(Float32, '/planning/steering_debug', 10)

    def skeleton_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        h, w = cv_image.shape
        robot_x = w // 2 # Robot is at center-bottom of image
        
        # Find all path pixels
        # argwhere returns [y, x]
        path_pixels = np.argwhere(cv_image > 0)
        
        cmd_msg = Twist()

        if len(path_pixels) < 10:
            self.get_logger().warn('No path detected, stopping.')
            self.cmd_vel_pub.publish(cmd_msg)
            return

        # Simple Lookahead Logic
        # Target Y is 'lookahead_dist' pixels up from the bottom (h)
        target_y_idx = h - self.lookahead_dist
        
        # Find pixel on path closest to target Y
        differences = np.abs(path_pixels[:, 0] - target_y_idx)
        min_idx = np.argmin(differences)
        
        target_y, target_x = path_pixels[min_idx]

        # Steering control (P-Control)
        # Error is difference between robot center and target x
        error_x = robot_x - target_x 
        
        # If target is to the left (smaller x), error is positive.
        # Positive steering usually means turn Left (CCW).
        steering_angle = self.k_p * error_x

        # Speed control (slow down when turning)
        linear_speed = self.max_speed * max(0.1, 1.0 - abs(steering_angle))

        cmd_msg.linear.x = float(linear_speed)
        cmd_msg.angular.z = float(steering_angle)
        
        self.cmd_vel_pub.publish(cmd_msg)

        # Debug Visualization
        p_msg = PointStamped()
        p_msg.header = msg.header
        p_msg.point.x = float(target_x)
        p_msg.point.y = float(target_y)
        self.target_pub.publish(p_msg)
        self.steering_debug_pub.publish(Float32(data=steering_angle))

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
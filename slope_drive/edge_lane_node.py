import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Try to import scipy for spline smoothing
try:
    from scipy.interpolate import splprep, splev
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

def yaw_to_quaternion(yaw):
    """Convert yaw angle to a quaternion."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=np.sin(yaw / 2.0),
        w=np.cos(yaw / 2.0)
    )

class EdgeLaneNode(Node):
    def __init__(self):
        super().__init__('edge_lane_node')
        self.get_logger().info('Edge Lane Node has been started.')
        if not SCIPY_AVAILABLE:
            self.get_logger().warning('Scipy not found. Path will not be smoothed. Consider installing it: pip install scipy')

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber to the BEV image
        self.bev_image_sub = self.create_subscription(
            Image,
            '/bev/image',
            self.bev_image_callback,
            10)

        # Publisher for the processed skeleton image (for debugging)
        self.edge_image_pub = self.create_publisher(Image, '/bev/edges', 10)
        # Publisher for the extracted path
        self.path_pub = self.create_publisher(Path, '/bev/path', 10)

    def bev_image_callback(self, msg):
        # self.get_logger().info(f'Received BEV Image: {msg.header.stamp}')
        
        try:
            bev_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        gray_image = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5,5), np.uint8)
        mask_morphed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        skeleton = np.zeros(mask_morphed.shape, np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        temp = mask_morphed.copy()
        while True:
            eroded = cv2.erode(temp, element)
            dilated = cv2.dilate(eroded, element)
            subset = temp - dilated
            skeleton = cv2.bitwise_or(skeleton, subset)
            temp = eroded.copy()
            if cv2.countNonZero(temp) == 0:
                break
        
        try:
            skeleton_msg = self.bridge.cv2_to_imgmsg(skeleton, "mono8")
            skeleton_msg.header = msg.header
            self.edge_image_pub.publish(skeleton_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert skeleton image: {e}')

        points = cv2.findNonZero(skeleton)
        if points is None:
            # self.get_logger().info('No path points found in skeleton.')
            return

        points = points.reshape(-1, 2)
        sorted_points = points[np.argsort(points[:, 1])[::-1]]
        
        # Path smoothing and orientation calculation
        path_msg = Path()
        path_msg.header = msg.header
        path_msg.header.frame_id = "map"

        if SCIPY_AVAILABLE and len(sorted_points) > 3:
            # Use spline to smooth the path
            x = sorted_points[:, 0]
            y = sorted_points[:, 1]
            
            # Fit spline
            tck, u = splprep([x, y], s=30) # s is a smoothing factor
            
            # Evaluate spline at more points for a smoother path
            u_new = np.linspace(u.min(), u.max(), 100)
            x_new, y_new = splev(u_new, tck, der=0)
            
            # Get derivatives for orientation
            x_deriv, y_deriv = splev(u_new, tck, der=1)
            
            for i in range(len(x_new)):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(x_new[i])
                pose.pose.position.y = float(y_new[i])
                pose.pose.position.z = 0.0
                
                # Calculate yaw from derivatives
                yaw = np.arctan2(y_deriv[i], x_deriv[i])
                pose.pose.orientation = yaw_to_quaternion(yaw)
                
                path_msg.poses.append(pose)
        else:
            # Fallback to unsmoothed path without orientation
            for point in sorted_points:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        # self.get_logger().info(f'Published path with {len(path_msg.poses)} points.')

def main(args=None):
    rclpy.init(args=args)
    node = EdgeLaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

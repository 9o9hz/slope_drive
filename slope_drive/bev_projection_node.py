import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

class BevProjectionNode(Node):
    def __init__(self):
        super().__init__('bev_projection_node')
        self.get_logger().info('BEV Projection Node has been started.')

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_height_m', 0.5),
                ('camera_pitch_offset_deg', 10.0),
                ('camera_roll_offset_deg', 0.0),
                ('bev_img_width', 400),
                ('bev_img_height', 400),
                ('bev_meters_per_pixel', 0.05)
            ])
        self.load_parameters()
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera_node/color/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera_node/color/camera_info', self.cam_info_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/camera_node/imu', self.imu_callback, 10)

        # Publishers
        self.bev_image_pub = self.create_publisher(Image, '/bev/image', 10)
        self.homography_pub = self.create_publisher(Float64MultiArray, '/bev/H', 10)
        self.debug_image_pub = self.create_publisher(Image, '/bev/debug_overlay', 10)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_imu_msg = None
        self.image_to_ground_homography = None

    def load_parameters(self):
        self.camera_height = self.get_parameter('camera_height_m').value
        self.pitch_offset_rad = np.deg2rad(self.get_parameter('camera_pitch_offset_deg').value)
        self.roll_offset_rad = np.deg2rad(self.get_parameter('camera_roll_offset_deg').value)
        self.bev_width = self.get_parameter('bev_img_width').value
        self.bev_height = self.get_parameter('bev_img_height').value
        self.mpp = self.get_parameter('bev_meters_per_pixel').value

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'camera_height_m':
                self.camera_height = param.value
            elif param.name == 'camera_pitch_offset_deg':
                self.pitch_offset_rad = np.deg2rad(param.value)
        
        if self.latest_imu_msg is not None:
            self.update_homography(self.latest_imu_msg)
        return SetParametersResult(successful=True)

    def cam_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.destroy_subscription(self.cam_info_sub)
            self.get_logger().info('Camera info received and subscription destroyed.')

    def imu_callback(self, msg):
        self.latest_imu_msg = msg
        if self.camera_matrix is not None:
            self.update_homography(msg)

    def update_homography(self, imu_msg):
        q = imu_msg.orientation
        R_world_to_imu = Rotation.from_quat([q.x, q.y, q.z, q.w])
        R_imu_to_cam_body = Rotation.from_euler('xyz', [self.roll_offset_rad, self.pitch_offset_rad, 0])
        R_world_to_cam_body = R_imu_to_cam_body * R_world_to_imu
        R_body_to_optical = Rotation.from_matrix([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
        R_world_to_cam_optical = R_body_to_optical * R_world_to_cam_body
        
        R = R_world_to_cam_optical.as_matrix().T
        t_cam_in_world = np.array([0, 0, self.camera_height])
        t_vec = -R @ t_cam_in_world
        
        H_g2i = self.camera_matrix @ np.hstack((R[:, 0:1], R[:, 1:2], t_vec.reshape(3,1)))
        try:
            self.image_to_ground_homography = np.linalg.inv(H_g2i)
        except np.linalg.LinAlgError:
            self.image_to_ground_homography = None

    def image_callback(self, msg):
        if self.camera_matrix is None or self.image_to_ground_homography is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

        M = np.array([
            [1/self.mpp, 0, self.bev_width / 2],
            [0, -1/self.mpp, self.bev_height],
            [0, 0, 1]
        ])

        final_homography = M @ self.image_to_ground_homography
        bev_image = cv2.warpPerspective(undistorted_image, final_homography, (self.bev_width, self.bev_height))

        bev_msg = self.bridge.cv2_to_imgmsg(bev_image, "bgr8")
        bev_msg.header = msg.header
        self.bev_image_pub.publish(bev_msg)
        
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(undistorted_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = BevProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
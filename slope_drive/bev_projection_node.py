import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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

        # CvBridge 초기화
        self.bridge = CvBridge()

        # 파라미터 선언 (카메라 높이, 오프셋, BEV 크기 등)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_height_m', 0.5), # 지면으로부터 카메라 렌즈까지의 높이 (미터)
                ('camera_pitch_offset_deg', 10.0), # 카메라가 아래를 보는 각도 (양수 값)
                ('camera_roll_offset_deg', 0.0), # 카메라의 수평 기울기 오프셋
                ('bev_img_width', 400), # BEV 이미지의 너비 (픽셀)
                ('bev_img_height', 400), # BEV 이미지의 높이 (픽셀)
                ('bev_meters_per_pixel', 0.05) # BEV 이미지에서 1픽셀이 나타내는 실제 거리 (미터)
            ])
        self.load_parameters()
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.cam_info_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/camera/imu', self.imu_callback, 10) # CHANGED: IMU 토픽 변경

        # Publishers
        self.bev_image_pub = self.create_publisher(Image, '/bev/image', 10)
        self.homography_pub = self.create_publisher(Float64MultiArray, '/bev/H', 10)
        self.debug_image_pub = self.create_publisher(Image, '/bev/debug_overlay', 10)

        # 클래스 변수 초기화
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_imu_msg = None
        self.image_to_ground_homography = None # H(t)

    def load_parameters(self):
        """선언된 파라미터 로드"""
        self.camera_height = self.get_parameter('camera_height_m').value
        self.pitch_offset_rad = np.deg2rad(self.get_parameter('camera_pitch_offset_deg').value)
        self.roll_offset_rad = np.deg2rad(self.get_parameter('camera_roll_offset_deg').value)
        self.bev_width = self.get_parameter('bev_img_width').value
        self.bev_height = self.get_parameter('bev_img_height').value
        self.mpp = self.get_parameter('bev_meters_per_pixel').value
        self.get_logger().info("Parameters loaded.")

    def parameters_callback(self, params):
        """파라미터 동적 변경 콜백"""
        for param in params:
            if param.name == 'camera_height_m':
                self.camera_height = param.value
            elif param.name == 'camera_pitch_offset_deg':
                self.pitch_offset_rad = np.deg2rad(param.value)
            # ... 다른 파라미터들도 동일하게 처리
        self.get_logger().info("Parameters updated.")
        # 파라미터 변경 시 호모그래피 행렬을 다시 계산
        if self.latest_imu_msg is not None:
            self.update_homography(self.latest_imu_msg)
        return SetParametersResult(successful=True)


    def cam_info_callback(self, msg):
        """카메라 정보 콜백. 한 번만 수신하고 구독 해제."""
        if self.camera_matrix is None:
            self.get_logger().info('Received camera info.')
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.destroy_subscription(self.cam_info_sub)
            self.get_logger().info('Camera info subscription destroyed.')

    def imu_callback(self, msg):
        """IMU 데이터 콜백. H(t)를 업데이트."""
        self.latest_imu_msg = msg
        if self.camera_matrix is not None:
            self.update_homography(msg)

    def update_homography(self, imu_msg):
        """
        IMU, 카메라 파라미터를 이용해 호모그래피 행렬 H(t)를 계산.
        R_world_to_cam = R_base * R_cam_imu * R_imu
        - Ground Plane (World): X-forward, Y-left, Z-up
        - Camera Optical Frame: Z-forward, X-right, Y-down
        """
        # 1. R_imu: IMU가 측정한 월드 좌표계 대비 IMU의 자세
        q = imu_msg.orientation
        R_world_to_imu = Rotation.from_quat([q.x, q.y, q.z, q.w])

        # 2. R_cam_imu: IMU 좌표계 대비 카메라의 장착 오프셋 (정적 보정)
        R_imu_to_cam_body = Rotation.from_euler('xyz', [self.roll_offset_rad, self.pitch_offset_rad, 0])

        # IMU 자세와 장착 오프셋을 결합하여 월드 좌표계 대비 카메라 몸체의 자세 계산
        R_world_to_cam_body = R_imu_to_cam_body * R_world_to_imu

        # 3. R_base: 카메라 몸체 좌표계(X-fwd, Y-left, Z-up)를
        #    카메라 광학 좌표계(Z-fwd, X-right, Y-down)로 변환하는 정적 회전
        R_body_to_optical = Rotation.from_matrix(
            [[0., -1., 0.],
             [0.,  0.,-1.],
             [1.,  0., 0.]]
        )

        # 최종적으로 월드(지면) 좌표계에서 카메라 광학 좌표계로의 회전 계산
        R_world_to_cam_optical = R_body_to_optical * R_world_to_cam_body

        # 호모그래피 계산을 위해 Scipy Rotation 객체를 Numpy 행렬로 변환.
        # 좌표계 변환(Passive rotation)을 위해 전치 행렬(.T)을 사용.
        R = R_world_to_cam_optical.as_matrix().T

        # 카메라의 월드 좌표계상 위치 벡터 [x, y, z]
        t_cam_in_world = np.array([0, 0, self.camera_height])
        # 월드 -> 카메라 좌표계로의 이동 벡터 계산
        t_vec = -R @ t_cam_in_world

        # 지면(z=0)에서 이미지 평면으로의 호모그래피 H_g2i 계산
        # H = K @ [r1 r2 t]
        H_g2i = self.camera_matrix @ np.hstack((R[:, 0:1], R[:, 1:2], t_vec.reshape(3,1)))

        try:
            # 이미지 평면 -> 지면으로의 역변환 H(t) 계산
            self.image_to_ground_homography = np.linalg.inv(H_g2i)
        except np.linalg.LinAlgError:
            self.get_logger().warn('Failed to compute inverse of homography matrix.')
            self.image_to_ground_homography = None

    def image_callback(self, msg):
        """이미지 콜백. 왜곡 보정, BEV 변환 및 퍼블리시 수행."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Camera info not received yet. Skipping image processing.')
            return

        if self.image_to_ground_homography is None:
            self.get_logger().warn('Homography matrix not computed yet. Skipping image processing.')
            return

        # ROS 이미지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Step A: 왜곡 보정
        undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

        # Step D: warpPerspective로 BEV 생성
        # 미터 단위를 BEV 이미지의 픽셀 단위로 변환하는 행렬 M 구성
        # 차량 바로 앞 지면(0,0)이 BEV 이미지의 (width/2, height)에 오도록 설정
        M = np.array([
            [1/self.mpp, 0, self.bev_width / 2],
            [0, -1/self.mpp, self.bev_height], # Y축 방향을 뒤집어 이미지 위쪽이 전방이 되도록 함
            [0, 0, 1]
        ])

        # 최종 변환 행렬 (이미지 -> BEV)
        final_homography = M @ self.image_to_ground_homography
        
        # BEV 이미지 생성
        bev_size = (self.bev_width, self.bev_height)
        bev_image = cv2.warpPerspective(undistorted_image, final_homography, bev_size, flags=cv2.INTER_LINEAR)

        # BEV 이미지 퍼블리시
        bev_msg = self.bridge.cv2_to_imgmsg(bev_image, "bgr8")
        bev_msg.header = msg.header
        self.bev_image_pub.publish(bev_msg)

        # (Optional) 디버그용 이미지 및 호모그래피 행렬 퍼블리시
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(undistorted_image, "bgr8"))
        h_msg = Float64MultiArray()
        h_msg.data = final_homography.flatten().tolist()
        self.homography_pub.publish(h_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BevProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
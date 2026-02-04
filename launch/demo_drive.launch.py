import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'slope_drive'

    return LaunchDescription([
        # =========================================================
        # 1. RealSense 카메라 공식 드라이버 (교체됨!)
        #    이 노드는 알아서 /camera/color/image_raw 토픽을 발행합니다.
        # =========================================================
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': False,   # 지금은 컬러 영상만 필요하므로 깊이는 끔 (충돌 방지)
                'enable_infra1': False,
                'enable_infra2': False,
                'rgb_camera.profile': '640x480x30' # 해상도/FPS 고정
            }]
        ),

        # =========================================================
        # 2. 알고리즘 노드들
        # =========================================================
        
        # (1) IMU Orientation Node
        Node(
            package=pkg_name,
            executable='imu_orientation_node',
            name='imu_orientation_node',
            output='screen'
        ),

        # (2) BEV Projection Node
        Node(
            package=pkg_name,
            executable='bev_projection_node',
            name='bev_projection_node',
            output='screen',
            parameters=[{
                'camera_height_m': 0.5,
                'bev_img_width': 400,
                'bev_img_height': 400
            }]
        ),

        # (3) Edge/Lane Mask Node
        Node(
            package=pkg_name,
            executable='edge_lane_node',
            name='edge_lane_node',
            output='screen'
        ),

        # (4) Skeletonization Node
        Node(
            package=pkg_name,
            executable='skeletonization_node',
            name='skeletonization_node',
            output='screen'
        ),

        # (5) Branch Pruning Node
        Node(
            package=pkg_name,
            executable='branch_pruning_node',
            name='branch_pruning_node',
            output='screen',
            parameters=[{'length_threshold': 20}]
        ),

        # (6) Path Planner Node
        Node(
            package=pkg_name,
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[{
                'lookahead_distance_pixels': 50,
                'max_speed': 0.2
            }]
        ),

        # =========================================================
        # 3. 시각화 노드
        # =========================================================
        Node(
            package=pkg_name,
            executable='visualizer_node',
            name='visualizer_node',
            output='screen'
        ),
    ])